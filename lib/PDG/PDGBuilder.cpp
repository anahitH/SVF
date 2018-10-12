#include "PDG/PDGBuilder.h"

#include "Util/AnalysisUtil.h"
#include "Util/CPPUtil.h"

#include "llvm/IR/CallSite.h"
#include "llvm/IR/Constant.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/User.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"

namespace pdg {

void PDGBuilder::processCE(const llvm::Value *val)
{
    if (const llvm::Constant* ref = llvm::dyn_cast<llvm::Constant>(val)) {
        if (const llvm::ConstantExpr* gepce = analysisUtil::isGepConstantExpr(ref)) {
            const llvm::Constant* opnd = gepce->getOperand(0);
            LocationSet ls;
            bool constGep = computeGepOffset(gepce, ls);
            // must invoke pag methods here, otherwise it will be a dead recursion cycle
            const llvm::Value* cval = pag->getCurrentValue();
            const llvm::BasicBlock* cbb = pag->getCurrentBB();
            pag->setCurrentLocation(gepce, NULL);
            /*
             * The gep edge created are like constexpr (same edge may appear at multiple callsites)
             * so bb/inst of this edge may be rewritten several times, we treat it as global here.
             */
            pag->addGepEdge(pag->getValueNode(opnd), pag->getValueNode(gepce), ls, constGep);
            pag->setCurrentLocation(cval, cbb);
            // handle recursive constant express case (gep (bitcast (gep X 1)) 1)
            processCE(opnd);
        } else if (const llvm::ConstantExpr* castce = analysisUtil::isCastConstantExpr(ref)) {
            const llvm::Constant* opnd = castce->getOperand(0);
            const llvm::Value* cval = pag->getCurrentValue();
            const llvm::BasicBlock* cbb = pag->getCurrentBB();
            pag->setCurrentLocation(castce, NULL);
            pag->addCopyEdge(pag->getValueNode(opnd), pag->getValueNode(castce));
            pag->setCurrentLocation(cval, cbb);
            processCE(opnd);
        } else if (const llvm::ConstantExpr* selectce = analysisUtil::isSelectConstantExpr(ref)) {
            const llvm::Constant* src1 = selectce->getOperand(1);
            const llvm::Constant* src2 = selectce->getOperand(2);
            const llvm::Value* cval = pag->getCurrentValue();
            const llvm::BasicBlock* cbb = pag->getCurrentBB();
            pag->setCurrentLocation(selectce, NULL);
            NodeID nsrc1 = pag->getValueNode(src1);
            NodeID nsrc2 = pag->getValueNode(src2);
            NodeID nres = pag->getValueNode(selectce);
            pag->addCopyEdge(nsrc1, nres);
            pag->addCopyEdge(nsrc2, nres);
            pag->addPhiNode(pag->getPAGNode(nres),pag->getPAGNode(nsrc1),NULL);
            pag->addPhiNode(pag->getPAGNode(nres),pag->getPAGNode(nsrc2),NULL);
            pag->setCurrentLocation(cval, cbb);
            processCE(src1);
            processCE(src2);
        } else if (const llvm::ConstantExpr* int2Ptrce = analysisUtil::isInt2PtrConstantExpr(ref)) {
            // if we meet a int2ptr, then it points-to black hole
            pag->addGlobalBlackHoleAddrEdge(pag->getValueNode(int2Ptrce), int2Ptrce);
        }
    }
}

void PDGBuilder::InitialGlobal(const llvm::GlobalVariable *gvar,
                               llvm::Constant *C,
                               u32_t offset)
{
    if (C->getType()->isSingleValueType()) {
        NodeID src = getValueNode(C);
        // get the field value if it is avaiable, otherwise we create a dummy field node.
        pag->setCurrentLocation(gvar, NULL);
        NodeID field = getGlobalVarField(gvar, offset);

        if (llvm::isa<llvm::GlobalVariable>(C) || llvm::isa<llvm::Function>(C)) {
            pag->setCurrentLocation(C, NULL);
            pag->addStoreEdge(src, field);
        } else if (llvm::isa<llvm::ConstantExpr>(C)) {
            // add gep edge of C1 itself is a constant expression
            processCE(C);
            pag->setCurrentLocation(C, NULL);
            pag->addStoreEdge(src, field);
        } else {
            //TODO:assert(false,"what else do we have");
        }

    } else if (llvm::isa<llvm::ConstantArray>(C)) {
        if (cppUtil::isValVtbl(gvar) == false)
            for (u32_t i = 0, e = C->getNumOperands(); i != e; i++)
                InitialGlobal(gvar, llvm::cast<llvm::Constant>(C->getOperand(i)), offset);

    } else if (llvm::isa<llvm::ConstantStruct>(C)) {
        const llvm::StructType *sty = llvm::cast<llvm::StructType>(C->getType());
        const std::vector<u32_t>& offsetvect =
            SymbolTableInfo::Symbolnfo()->getStructOffsetVec(sty);
        for (u32_t i = 0, e = C->getNumOperands(); i != e; i++) {
            u32_t off = offsetvect[i];
            InitialGlobal(gvar, llvm::cast<llvm::Constant>(C->getOperand(i)), offset + off);
        }

    } else {
        //TODO:assert(false,"what else do we have");
    }
}

void PDGBuilder::handleDirectCall(llvm::CallSite cs, const llvm::Function *F)
{
    assert(F);

    NodeID dstrec = getValueNode(cs.getInstruction());
    NodeID srcret = getReturnNode(F);
    pag->addRetEdge(srcret, dstrec, cs.getInstruction());
    //Iterators for the actual and formal parameters
    llvm::CallSite::arg_iterator itA = cs.arg_begin();
    llvm::CallSite::arg_iterator ieA = cs.arg_end();
    llvm::Function::const_arg_iterator itF = F->arg_begin();
    llvm::Function::const_arg_iterator ieF = F->arg_end();
    //Go through the fixed parameters.
    for (; itF != ieF; ++itA, ++itF) {
        //Some programs (e.g. Linux kernel) leave unneeded parameters empty.
        if (itA == ieA) {
            break;
        }
        const llvm::Value *AA = *itA;
        const llvm::Value *FA = &*itF; //current actual/formal arg
        NodeID dstFA = getValueNode(FA);
        NodeID srcAA = getValueNode(AA);
        pag->addCallEdge(srcAA, dstFA, cs.getInstruction());
    }
    //Any remaining actual args must be varargs.
    if (F->isVarArg()) {
        NodeID vaF = getVarargNode(F);
        for (; itA != ieA; ++itA) {
            llvm::Value *AA = *itA;
            NodeID vnAA = getValueNode(AA);
            pag->addCallEdge(vnAA,vaF, cs.getInstruction());
        }
    }
    if(itA != ieA) {
        /// FIXME: this assertion should be placed for correct checking except
        /// bug program like 188.ammp, 300.twolf
        // TODO: replace with our logger
        //wrnMsg("too many args to non-vararg func.");
        //wrnMsg("(" + getSourceLoc(cs.getInstruction()) + ")");

    }
}

void PDGBuilder::handleExtCall(llvm::CallSite cs, const llvm::Function *callee)
{
    const llvm::Instruction* inst = cs.getInstruction();
    if (analysisUtil::isHeapAllocOrStaticExtCall(cs)) {
        NodeID obj = getObjectNode(inst);
        // case 1: ret = new obj
        if (analysisUtil::isHeapAllocExtCallViaRet(cs) || analysisUtil::isStaticExtCall(cs)) {
            NodeID val = getValueNode(inst);
            NodeID obj = getObjectNode(inst);
            pag->addAddrEdge(obj, val);
        }
        // case 2: *arg = new obj
        else {
            assert(analysisUtil::isHeapAllocExtCallViaArg(cs) && "Must be heap alloc call via arg.");
            int arg_pos = analysisUtil::getHeapAllocHoldingArgPosition(callee);
            const llvm::Value *arg = cs.getArgument(arg_pos);
            if (arg->getType()->isPointerTy()) {
                NodeID vnArg = getValueNode(arg);
                NodeID dummy = pag->addDummyValNode();
                if (vnArg && dummy && obj) {
                    pag->addAddrEdge(obj, dummy);
                    pag->addStoreEdge(dummy, vnArg);
                }
            } else {
                //wrnMsg("Arg receiving new object must be pointer type");
            }
        }
    } else {
        if(analysisUtil::isExtCall(callee)) {
            ExtAPI::extf_t tF= analysisUtil::extCallTy(callee);
            switch(tF) {
            case ExtAPI::EFT_REALLOC: {
                if(llvm::isa<llvm::ConstantPointerNull>(cs.getArgument(0))) {
                    NodeID val = getValueNode(inst);
                    NodeID obj = getObjectNode(inst);
                    pag->addAddrEdge(obj, val);
                }
                break;
            }
            case ExtAPI::EFT_L_A0:
            case ExtAPI::EFT_L_A1:
            case ExtAPI::EFT_L_A2:
            case ExtAPI::EFT_L_A8: {
                NodeID dstNode = getValueNode(inst);
                Size_t arg_pos;
                switch(tF) {
                case ExtAPI::EFT_L_A1:
                    arg_pos= 1;
                    break;
                case ExtAPI::EFT_L_A2:
                    arg_pos= 2;
                    break;
                case ExtAPI::EFT_L_A8:
                    arg_pos= 8;
                    break;
                default:
                    arg_pos= 0;
                }
                llvm::Value *src= cs.getArgument(arg_pos);
                NodeID srcNode = getValueNode(src);
                pag->addCopyEdge(srcNode, dstNode);
                break;
            }
            case ExtAPI::EFT_L_A0__A0R_A1R: {
                addComplexConsForExt(cs.getArgument(0), cs.getArgument(1));
                //memcpy returns the dest.
                pag->addCopyEdge(getValueNode(cs.getArgument(0)), getValueNode(inst));
                break;
            }
            case ExtAPI::EFT_A1R_A0R:
                addComplexConsForExt(cs.getArgument(1), cs.getArgument(0));
                break;
            case ExtAPI::EFT_A3R_A1R_NS:
                //These func. are never used to copy structs, so the size is 1.
                addComplexConsForExt(cs.getArgument(3), cs.getArgument(1), 1);
                break;
            case ExtAPI::EFT_A1R_A0: {
                NodeID vnD= getValueNode(cs.getArgument(1));
                NodeID vnS= getValueNode(cs.getArgument(0));
                if(vnD && vnS) {
                    pag->addStoreEdge(vnS,vnD);
                }
                break;
            }
            case ExtAPI::EFT_A2R_A1: {
                NodeID vnD= getValueNode(cs.getArgument(2));
                NodeID vnS= getValueNode(cs.getArgument(1));
                if(vnD && vnS) {
                    pag->addStoreEdge(vnS,vnD);
                }
                break;
            }
            case ExtAPI::EFT_A4R_A1: {
                NodeID vnD= getValueNode(cs.getArgument(4));
                NodeID vnS= getValueNode(cs.getArgument(1));
                if(vnD && vnS) {
                    pag->addStoreEdge(vnS,vnD);
                }
                break;
            }
            case ExtAPI::EFT_L_A0__A2R_A0: {
                if(llvm::isa<llvm::PointerType>(inst->getType())) {
                    //Do the L_A0 part if the retval is used.
                    NodeID vnD= getValueNode(inst);
                    llvm::Value *src= cs.getArgument(0);
                    NodeID vnS= getValueNode(src);
                    if(vnS) {
                        pag->addCopyEdge(vnS,vnD);
                    }
                }
                //Do the A2R_A0 part.
                NodeID vnD= getValueNode(cs.getArgument(2));
                NodeID vnS= getValueNode(cs.getArgument(0));
                if(vnD && vnS) {
                    pag->addStoreEdge(vnS,vnD);
                }
                break;
            }
            case ExtAPI::EFT_A0R_NEW:
            case ExtAPI::EFT_A1R_NEW:
            case ExtAPI::EFT_A2R_NEW:
            case ExtAPI::EFT_A4R_NEW:
            case ExtAPI::EFT_A11R_NEW: {
                assert(!"Alloc via arg cases are not handled here.");
                break;
            }
            case ExtAPI::EFT_ALLOC:
            case ExtAPI::EFT_NOSTRUCT_ALLOC:
            case ExtAPI::EFT_STAT:
            case ExtAPI::EFT_STAT2:
            case ExtAPI::EFT_NOOP:
            case ExtAPI::EFT_FREE:
                break;
            case ExtAPI::EFT_STD_RB_TREE_INSERT_AND_REBALANCE: {
                llvm::Value *vArg1 = cs.getArgument(1);
                llvm::Value *vArg3 = cs.getArgument(3);

                // We have vArg3 points to the entry of _Rb_tree_node_base { color; parent; left; right; }.
                // Now we calculate the offset from base to vArg3
                NodeID vnArg3 = pag->getValueNode(vArg3);
                Size_t offset = pag->getLocationSetFromBaseNode(vnArg3).getOffset();

                // We get all flattened fields of base
                std::vector<LocationSet> fields;
                const llvm::Type *type = getBaseTypeAndFlattenedFields(vArg3, fields);
                assert(fields.size() >= 4 && "_Rb_tree_node_base should have at least 4 fields.\n");

                // We summarize the side effects: arg3->parent = arg1, arg3->left = arg1, arg3->right = arg1
                // Note that arg0 is aligned with "offset".
                for (int i = offset + 1; i <= offset + 3; ++i) {
                    NodeID vnD = pag->getGepValNode(vArg3, fields[i], type, i);
                    NodeID vnS = getValueNode(vArg1);
                    if(vnD && vnS)
                        pag->addStoreEdge(vnS,vnD);
                }
                break;
            }
            case ExtAPI::EFT_STD_RB_TREE_INCREMENT: {
                NodeID vnD = pag->getValueNode(inst);

                llvm::Value *vArg = cs.getArgument(0);
                NodeID vnArg = pag->getValueNode(vArg);
                Size_t offset = pag->getLocationSetFromBaseNode(vnArg).getOffset();

                // We get all fields
                std::vector<LocationSet> fields;
                const llvm::Type *type = getBaseTypeAndFlattenedFields(vArg,fields);
                assert(fields.size() >= 4 && "_Rb_tree_node_base should have at least 4 fields.\n");

                // We summarize the side effects: ret = arg->parent, ret = arg->left, ret = arg->right
                // Note that arg0 is aligned with "offset".
                for (int i = offset + 1; i <= offset + 3; ++i) {
                    NodeID vnS = pag->getGepValNode(vArg, fields[i], type, i);
                    if(vnD && vnS)
                        pag->addStoreEdge(vnS,vnD);
                }
                break;
            }
            case ExtAPI::EFT_STD_LIST_HOOK: {
                llvm::Value *vSrc = cs.getArgument(0);
                llvm::Value *vDst = cs.getArgument(1);
                NodeID src = pag->getValueNode(vSrc);
                NodeID dst = pag->getValueNode(vDst);
                pag->addStoreEdge(src, dst);
                break;
            }
            case ExtAPI::CPP_EFT_A0R_A1: {
                SymbolTableInfo* symTable = SymbolTableInfo::Symbolnfo();
                if (symTable->getModelConstants()) {
                    NodeID vnD = pag->getValueNode(cs.getArgument(0));
                    NodeID vnS = pag->getValueNode(cs.getArgument(1));
                    pag->addStoreEdge(vnS, vnD);
                }
                break;
            }
            case ExtAPI::CPP_EFT_A0R_A1R: {
                SymbolTableInfo* symTable = SymbolTableInfo::Symbolnfo();
                if (symTable->getModelConstants()) {
                    NodeID vnD = getValueNode(cs.getArgument(0));
                    NodeID vnS = getValueNode(cs.getArgument(1));
                    assert(vnD && vnS && "dst or src not exist?");
                    NodeID dummy = pag->addDummyValNode();
                    pag->addLoadEdge(vnS,dummy);
                    pag->addStoreEdge(dummy,vnD);
                }
                break;
            }
            case ExtAPI::CPP_EFT_A1R: {
                SymbolTableInfo* symTable = SymbolTableInfo::Symbolnfo();
                if (symTable->getModelConstants()) {
                    NodeID vnS = getValueNode(cs.getArgument(1));
                    assert(vnS && "src not exist?");
                    NodeID dummy = pag->addDummyValNode();
                    pag->addLoadEdge(vnS,dummy);
                }
                break;
            }
            case ExtAPI::CPP_EFT_DYNAMIC_CAST: {
                llvm::Value *vArg0 = cs.getArgument(0);
                llvm::Value *retVal = cs.getInstruction();
                NodeID src = getValueNode(vArg0);
                assert(src && "src not exist?");
                NodeID dst = getValueNode(retVal);
                assert(dst && "dst not exist?");
                pag->addCopyEdge(src, dst);
                break;
            }
            case ExtAPI::EFT_CXA_BEGIN_CATCH:
                break;
            case ExtAPI::EFT_OTHER: {
                std::string str;
                llvm::raw_string_ostream rawstr(str);
                rawstr << "function " << callee->getName() << " not in the external function summary list";
            }
            }
        }

        /// create inter-procedural PAG edges for thread forks
        if(analysisUtil::isThreadForkCall(inst)) {
            if(const llvm::Function* forkedFun = analysisUtil::getLLVMFunction(analysisUtil::getForkedFun(inst)) ) {
                forkedFun = analysisUtil::getDefFunForMultipleModule(forkedFun);
                const llvm::Value* actualParm = analysisUtil::getActualParmAtForkSite(inst);
                /// pthread_create has 1 arg.
                /// apr_thread_create has 2 arg.
                assert((forkedFun->arg_size() <= 2) && "Size of formal parameter of start routine should be one");
                if(forkedFun->arg_size() <= 2 && forkedFun->arg_size() >= 1) {
                    const llvm::Argument* formalParm = &(*forkedFun->arg_begin());
                    /// Connect actual parameter to formal parameter of the start routine
                    pag->addThreadForkEdge(pag->getValueNode(actualParm), pag->getValueNode(formalParm),inst);
                }
            }
            else {
                /// handle indirect calls at pthread create APIs e.g., pthread_create(&t1, NULL, fp, ...);
                ///const llvm::Value* fun = ThreadAPI::getThreadAPI()->getForkedFun(inst);
                ///if(!isa<Function>(fun))
                ///    pag->addIndirectCallsites(cs,pag->getValueNode(fun));
            }
            /// If forkedFun does not pass to spawnee as function type but as void pointer
            /// remember to update inter-procedural callgraph/PAG/SVFG etc. when indirect call targets are resolved
            /// We don't connect the callgraph here, further investigation is need to hanle mod-ref during SVFG construction.
        }

        /// create inter-procedural PAG edges for hare_parallel_for calls
        else if(analysisUtil::isHareParForCall(inst)) {
            if(const llvm::Function* taskFunc = analysisUtil::getLLVMFunction(analysisUtil::getTaskFuncAtHareParForSite(inst)) ) {
                /// The task function of hare_parallel_for has 3 args.
                assert((taskFunc->arg_size() == 3) && "Size of formal parameter of hare_parallel_for's task routine should be 3");
                const llvm::Value* actualParm = analysisUtil::getTaskDataAtHareParForSite(inst);
                const llvm::Argument* formalParm = &(*taskFunc->arg_begin());
                /// Connect actual parameter to formal parameter of the start routine
                pag->addThreadForkEdge(pag->getValueNode(actualParm), pag->getValueNode(formalParm),inst);
            }
            else {
                /// handle indirect calls at hare_parallel_for (e.g., hare_parallel_for(..., fp, ...);
                ///const llvm::Value* fun = ThreadAPI::getThreadAPI()->getForkedFun(inst);
                ///if(!isa<Function>(fun))
                ///    pag->addIndirectCallsites(cs,pag->getValueNode(fun));
            }
        }

        /// TODO: inter-procedural PAG edges for thread joins
    }
}

void PDGBuilder::visitAllocaInst(llvm::AllocaInst &AI)
{

    // AllocaInst should always be a pointer type
    assert(llvm::isa<llvm::PointerType>(AI.getType()));

    NodeID dst = getValueNode(&AI);
    NodeID src = getObjectNode(&AI);
    pag->addAddrEdge(src, dst);
}

void PDGBuilder::visitPHINode(llvm::PHINode &I)
{
    NodeID dst = getValueNode(&I);

    for (Size_t i = 0; i < I.getNumIncomingValues(); ++i) {
        NodeID src = getValueNode(I.getIncomingValue(i));
        const llvm::BasicBlock* bb = I.getIncomingBlock(i);
        pag->addCopyEdge(src, dst);
        pag->addPhiNode(pag->getPAGNode(dst),pag->getPAGNode(src),bb);
    }
}

void PDGBuilder::visitLoadInst(llvm::LoadInst &inst)
{
    pag->loadInstNum++;
    NodeID dst = getValueNode(&inst);
    NodeID src = getValueNode(inst.getPointerOperand());
    pag->addLoadEdge(src, dst);
}

void PDGBuilder::visitStoreInst(llvm::StoreInst &I)
{
    pag->storeInstNum++;
    // StoreInst itself should always not be a pointer type
    assert(!llvm::isa<llvm::PointerType>(I.getType()));
    NodeID dst = getValueNode(I.getPointerOperand());
    NodeID src = getValueNode(I.getValueOperand());
    pag->addStoreEdge(src, dst);
}

void PDGBuilder::visitIntToPtrInst(llvm::IntToPtrInst &inst)
{
    NodeID dst = getValueNode(&inst);
    pag->addBlackHoleAddrEdge(dst);
}

void PDGBuilder::visitCastInst(llvm::CastInst &I)
{
    NodeID dst = getValueNode(&I);
    llvm::Value * opnd = I.getOperand(0);
    if (!llvm::isa<llvm::PointerType>(opnd->getType())) {
        opnd = analysisUtil::stripAllCasts(opnd);
    } else if (llvm::isa<llvm::PointerType>(opnd->getType())) {
        NodeID src = getValueNode(opnd);
        pag->addCopyEdge(src, dst);
    } else {
        assert(llvm::isa<llvm::IntToPtrInst>(&I) && "what else do we have??");
        pag->addBlackHoleAddrEdge(dst);
    }
}

void PDGBuilder::visitReturnInst(llvm::ReturnInst &I)
{
    // ReturnInst itself should always not be a pointer type
    assert(!llvm::isa<llvm::PointerType>(I.getType()));

    //ignore void and non-ptr return statements
    if (I.getNumOperands()) {
        llvm::Value *src = I.getReturnValue();
        llvm::Function *F = I.getParent()->getParent();
        NodeID rnF = getReturnNode(F);
        NodeID vnS = getValueNode(src);
        //vnS may be null if src is a null ptr
        pag->addCopyEdge(vnS, rnF);
    }
}

void PDGBuilder::visitExtractValueInst(llvm::ExtractValueInst &I)
{
    NodeID dst = getValueNode(&I);
    pag->addBlackHoleAddrEdge(dst);
}

} // namespace pdg


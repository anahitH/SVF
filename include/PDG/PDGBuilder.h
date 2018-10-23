#pragma once

#include "MemoryModel/PAGBuilder.h"

#include <cstdint>

namespace llvm {
    class AllocaInst;
    class CallSite;
    class CastInst;
    class Constant;
    class ExtractValueInst;
    class Function;
    class GlobalVariable;
    class IntToPtrInst;
    class LoadInst;
    class PHINode;
    class ReturnInst;
    class StoreInst;
    class User;
}

namespace svfg {

/**
 * \class PDGBuilder
 * Class to build Program Dependence Graph.
 */
class PDGBuilder : public PAGBuilder
{
public:
    void processCE(const llvm::Value *val);
    void InitialGlobal(const llvm::GlobalVariable *gvar,
                       llvm::Constant *C,
                       u32_t offset);
    void handleDirectCall(llvm::CallSite cs, const llvm::Function *F);
    virtual void handleExtCall(llvm::CallSite cs, const llvm::Function *F) override;

    virtual void visitAllocaInst(llvm::AllocaInst &AI) override;
    void visitPHINode(llvm::PHINode &I) override;
    void visitLoadInst(llvm::LoadInst &inst) override;
    void visitStoreInst(llvm::StoreInst &I) override;
    void visitIntToPtrInst(llvm::IntToPtrInst &inst) override;
    void visitCastInst(llvm::CastInst &I) override;
    void visitReturnInst(llvm::ReturnInst &I) override;
    void visitExtractValueInst(llvm::ExtractValueInst &EVI) override;
    virtual void visitInstruction(llvm::Instruction &I) override;
}; //class PDGBuilder

} // namespace pdg

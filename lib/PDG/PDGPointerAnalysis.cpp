#include "PDG/PDGPointerAnalysis.h"

#include "MemoryModel/PAGBuilder.h"
#include "PDG/PDGBuilder.h"

#include "SVF/Util/SVFModule.h"
#include "SVF/MemoryModel/CHA.h"
#include "SVF/MemoryModel/PTAType.h"

namespace pdg {

void PDGAndersenWaveDiff::initialize(SVFModule svfModule)
{
    resetData();

    SymbolTableInfo* symTable = SymbolTableInfo::Symbolnfo();
    symTable->buildMemModel(svfModule);
    pdg::PDGBuilder pdgBuilder;
    pag = pdgBuilder.build(svfModule);
    chgraph = new CHGraph(svfModule);
    chgraph->buildCHG();
    typeSystem = new TypeSystem(pag);
    ptaCallGraph = new PTACallGraph(svfModule);
    callGraphSCCDetection();
    svfMod = svfModule;

    consCG = new ConstraintGraph(pag);
    setGraph(consCG);
    /// Create statistic class
    stat = new AndersenStat(this);
}

} // namespace pdg


#include "PDG/PDGPointerAnalysis.h"

#include "MemoryModel/PAGBuilder.h"
#include "PDG/PDGBuilder.h"

#include "Util/ThreadCallGraph.h"
#include "Util/SVFModule.h"
#include "MemoryModel/CHA.h"
#include "MemoryModel/PTAType.h"


//#include "SVF/Util/SVFModule.h"
//#include "SVF/MemoryModel/CHA.h"
//#include "SVF/MemoryModel/PTAType.h"

namespace svfg {

void PDGAndersenWaveDiff::initialize(SVFModule svfModule)
{
    resetData();

    SymbolTableInfo* symTable = SymbolTableInfo::Symbolnfo();
    symTable->buildMemModel(svfModule);
    PDGBuilder pdgBuilder;
    pag = pdgBuilder.build(svfModule);
    chgraph = new CHGraph(svfModule);
    chgraph->buildCHG();
    typeSystem = new TypeSystem(pag);
    /// initialise pta call graph for every pointer analysis instance
    if(EnableThreadCallGraph)
        ptaCallGraph = new ThreadCallGraph(svfModule);
    else
        ptaCallGraph = new PTACallGraph(svfModule);

    callGraphSCCDetection();

    svfMod = svfModule;

    consCG = new ConstraintGraph(pag);
    setGraph(consCG);
    /// Create statistic class
    stat = new AndersenStat(this);
}

} // namespace pdg


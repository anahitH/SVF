set(SOURCES
       cuddAddNeg.c
       cuddBddCorr.c
       cuddDecomp.c
       cuddInit.c
       cuddRead.c
       cuddSubsetSP.c
       cuddZddIsop.c
       datalimit.c
       dddmpNodeBdd.c
       safe_mem.c
       cpu_stats.c
       cuddAddWalsh.c
       cuddBddIte.c
       cuddEssent.c
       cuddInteract.c
       cuddRef.c
       cuddSymmetry.c
       cuddZddLin.c
       dddmpBinary.c
       dddmpNodeCnf.c
       mtrBasic.c
       saveimage.c
       cpu_time.c
       cuddAndAbs.c
       cuddBridge.c
       cuddExact.c
       cuddLCache.c
       cuddReorder.c
       cuddTable.c
       cuddZddMisc.c
       dddmpConvert.c
       dddmpStoreAdd.c
       mtrGroup.c
       state.c
       cuddAddAbs.c
       cuddAnneal.c
       cuddCache.c
       cuddExport.c
       cuddLevelQ.c
       cuddSat.c
       cuddUtil.c
       cuddZddPort.c
       dddmpDbg.c
       dddmpStoreBdd.c
       pathsearch.c
       st.c
       texpand.c
       cuddAddApply.c
       cuddApa.c
       cuddCheck.c
       cuddGenCof.c
       cuddLinear.c
       cuddSign.c
       cuddWindow.c
       cuddZddReord.c
       dddmpDdNodeBdd.c
       dddmpStoreCnf.c
       pipefork.c
       strsav.c
       cuddAddFind.c
       cuddAPI.c
       cuddClip.c
       cuddGenetic.c
       cuddLiteral.c
       cuddSolve.c
       cuddZddCount.c
       cuddZddSetop.c
       dddmpLoad.c
       dddmpStoreMisc.c
       prtime.c
       stub.c
       cuddAddInv.c
       cuddApprox.c
       cuddCof.c
       cuddGroup.c
       cuddMatMult.c
       cuddSplit.c
       cuddZddFuncs.c
       cuddZddSymm.c
       dddmpLoadCnf.c
       dddmpUtil.c
       ptime.c
       cuddAddIte.c
       cuddBddAbs.c
       cuddCompose.c
       cuddHarwell.c
       cuddPriority.c
       cuddSubsetHB.c
       cuddZddGroup.c
       cuddZddUtil.c
       dddmpNodeAdd.c
       epd.c
       restart.c
)

#add_llvm_loadable_module(Cudd ${SOURCES})
add_llvm_library(Cudd ${SOURCES})
target_include_directories(Cudd PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../../include>
                           $<INSTALL_INTERFACE:include/SVF>
                           PRIVATE
                           ${LLVM_INCLUDE_DIRS}
)
target_compile_features(Cudd PUBLIC cxx_range_for cxx_auto_type)
target_compile_options(Cudd PRIVATE -fPIC -g -Wno-format -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast -O3 -w -DHAVE_IEEE_754 -DSIZEOF_VOID_P=8 -DSIZEOF_LONG=8)
if(DEFINED IN_SOURCE_BUILD)
    add_dependencies(Cudd intrinsics_gen)
endif()
if (APPLE)
    set_target_properties(Cudd PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif (APPLE)

add_library(svf::Cudd ALIAS Cudd)
install(TARGETS Cudd
        EXPORT svfCuddTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Cudd)
install(DIRECTORY ../../include/CUDD
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/SVF/CUDD
        COMPONENT Cudd)
install(
        EXPORT svfCuddTargets
        NAMESPACE svf::
        DESTINATION ${INSTALL_CONFIGDIR}
        COMPONENT Cudd
)
export(
        EXPORT svfCuddTargets
        NAMESPACE svf::
        FILE ${CMAKE_CURRENT_BINARY_DIR}/svfCuddTargets.cmake
)
export(PACKAGE Cudd)


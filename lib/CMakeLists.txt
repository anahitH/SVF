add_subdirectory(CUDD)

# Due to a mutual dependencies, all the sub projects of the SVG are merged here
# Otherwise it is impossible to load the dependencies in opt
# NOTE: if the SVF should be linked into opt, we should probably use the individual sub projects here, rather than the combined project 
set(SVF_SOURCES
    Util/AnalysisUtil.cpp
    Util/CPPUtil.cpp
    Util/DataFlowUtil.cpp
    Util/PTACallGraph.cpp
    Util/RaceAnnotator.cpp
    Util/ThreadCallGraph.cpp
    Util/BreakConstantExpr.cpp
    Util/Conditions.cpp
    Util/ExtAPI.cpp
    Util/PathCondAllocator.cpp
    Util/PTAStat.cpp
    Util/ThreadAPI.cpp
    Util/SVFModule.cpp
    MemoryModel/ConsG.cpp
    MemoryModel/LocationSet.cpp
    MemoryModel/LocMemModel.cpp
    MemoryModel/MemModel.cpp
    MemoryModel/PAGBuilder.cpp
    MemoryModel/PAG.cpp
    MemoryModel/CHA.cpp
    MemoryModel/PointerAnalysis.cpp
    MSSA/MemPartition.cpp
    MSSA/MemRegion.cpp
    MSSA/MemSSA.cpp
    MSSA/SVFGBuilder.cpp
    MSSA/SVFG.cpp 
    MSSA/SVFGOPT.cpp
    MSSA/SVFGStat.cpp
    SABER/DoubleFreeChecker.cpp
    SABER/FileChecker.cpp
    SABER/LeakChecker.cpp
    SABER/ProgSlice.cpp
    SABER/SaberAnnotator.cpp
    SABER/SaberCheckerAPI.cpp
    SABER/SaberSVFGBuilder.cpp
    SABER/SrcSnkDDA.cpp
    WPA/Andersen.cpp
    WPA/AndersenLCD.cpp
    WPA/AndersenStat.cpp
    WPA/AndersenWave.cpp
    WPA/AndersenWaveDiff.cpp
    WPA/AndersenWaveDiffWithType.cpp
    WPA/FlowSensitive.cpp
    WPA/FlowSensitiveStat.cpp
    WPA/TypeAnalysis.cpp
    WPA/WPAPass.cpp
    )

add_llvm_loadable_module(Svf ${SVF_SOURCES})
#add_llvm_library(Svf ${SVF_SOURCES})
target_include_directories(Svf PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include>
                           $<INSTALL_INTERFACE:include/SVF/>
                           PRIVATE
                           ${LLVM_INCLUDE_DIRS}
)

llvm_map_components_to_libnames(llvm_libs bitwriter core ipo irreader instcombine instrumentation target linker analysis scalaropts support )
if(DEFINED IN_SOURCE_BUILD)
    add_dependencies(Svf intrinsics_gen)
endif()

target_link_libraries(Svf PRIVATE svf::Cudd)
target_compile_features(Svf PUBLIC cxx_range_for cxx_auto_type)
target_compile_options(Svf PRIVATE -fno-rtti -g)

if (APPLE)
    set_target_properties(Svf PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif (APPLE)

install(TARGETS Svf
        EXPORT svfSvfTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Svf)
install(DIRECTORY ../include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/SVF
        COMPONENT Svf)
install(DIRECTORY ../include/CUDD
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/SVF/CUDD
        COMPONENT Svf)
install(
        EXPORT svfSvfTargets
        NAMESPACE svf::
        DESTINATION ${INSTALL_CONFIGDIR}
        COMPONENT Svf
)
export(
        EXPORT svfSvfTargets
        NAMESPACE svf::
        FILE ${CMAKE_CURRENT_BINARY_DIR}/svfSvfTargets.cmake
)
export(PACKAGE Svf)

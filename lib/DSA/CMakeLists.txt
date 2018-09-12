set(SOURCES
  AddressTakenAnalysis.cpp
  AllocatorIdentification.cpp
  Basic.cpp
  BottomUpClosure.cpp
  CallTargets.cpp
  CompleteBottomUp.cpp
  DSCallGraph.cpp
  DSGraph.cpp
  DSTest.cpp
  DataStructure.cpp
  DataStructureStats.cpp
  EntryPointAnalysis.cpp
  EquivClassGraphs.cpp
  GraphChecker.cpp
  Local.cpp
  Printer.cpp
  SanityCheck.cpp
  StdLibPass.cpp
  TopDownClosure.cpp
  TypeSafety.cpp
  SteensgaardAA.cpp
  Steensgaard.cpp
  FormatFunctions.cpp
  )

# Build both shared and static libs
# Also, drop the 'lib' suffix to match how
# the Makefile-driven version functions.

if( NOT WIN32 AND LLVM_ENABLE_PIC )
  add_llvm_loadable_module(LLVMDataStructure ${SOURCES})
  set_property(TARGET LLVMDataStructure PROPERTY OUTPUT_NAME "LLVMDataStructure")
  set_property(TARGET LLVMDataStructure PROPERTY PREFIX "")
  set(DSA_STATIC_TARGET LLVMDataStructure_static)
  add_dependencies(LLVMDataStructure intrinsics_gen)
else()
  set(DSA_STATIC_TARGET LLVMDataStructure)
endif()

if( NOT BUILD_SHARED_LIBS )
  add_llvm_library(${DSA_STATIC_TARGET} ${SOURCES})
  set_property(TARGET ${DSA_STATIC_TARGET} PROPERTY OUTPUT_NAME "LLVMDataStructure")
  set_property(TARGET ${DSA_STATIC_TARGET} PROPERTY PREFIX "")
  add_dependencies(${DSA_STATIC_TARGET} intrinsics_gen)
endif()

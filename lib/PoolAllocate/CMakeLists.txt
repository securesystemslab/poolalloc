set(LLVM_LINK_COMPONENTS
  DataStructure_static
  )


set(SOURCES
  AccessTrace.cpp
  AllHeapNodesHeuristic.cpp
  AllNodesHeuristic.cpp
  Heuristic.cpp
  PAMultipleGlobalPool.cpp
  PASimple.cpp
  PointerCompress.cpp
  PoolAllocate.cpp
  PoolOptimize.cpp
  RunTimeAssociate.cpp
  TransformFunctionBody.cpp
)

# Build both shared and static libs
# Also, drop the 'lib' suffix to match how
# the Makefile-driven version functions.

if( NOT WIN32 AND LLVM_ENABLE_PIC )
  add_llvm_loadable_module(poolalloc ${SOURCES})
  set_property(TARGET poolalloc PROPERTY OUTPUT_NAME "poolalloc")
  set_property(TARGET poolalloc PROPERTY PREFIX "")
  set(PA_STATIC_TARGET poolalloc_static)
  add_dependencies(poolalloc intrinsics_gen)
else()
  set(PA_STATIC_TARGET poolalloc)
endif()

if( NOT BUILD_SHARED_LIBS )
  add_llvm_library(${PA_STATIC_TARGET} ${SOURCES})
  set_property(TARGET ${PA_STATIC_TARGET} PROPERTY OUTPUT_NAME "poolalloc")
  set_property(TARGET ${PA_STATIC_TARGET} PROPERTY PREFIX "")
  add_dependencies(${PA_STATIC_TARGET} intrinsics_gen)
endif()

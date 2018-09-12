option(POOLALLOC_USE_DYNCALL "Use dyncall library" OFF)

add_llvm_library( poolalloc_rt PoolAllocator.cpp )
set_property(
   TARGET poolalloc_rt
   PROPERTY COMPILE_DEFINITIONS
   )

if (POOLALLOC_USE_DYNCALL)
  include_directories(/localhome/simmon12/progs/dyncall-0.5/dyncall)
  link_directories(/localhome/simmon12/progs/dyncall-0.5/dyncall/build_out/linux_x86_gcc_release)
  target_link_libraries( poolalloc_rt dyncall_s )
  add_definitions(-DUSE_DYNCALL=1)
endif()

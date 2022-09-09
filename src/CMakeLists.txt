cmake_minimum_required(VERSION 3.13)


set(LP_SCHEDULER_HEADERS 
    "${CMAKE_CURRENT_SOURCE_DIR}/scheduler/disjoint_interval_set.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/scheduler/feasible_scheduler.hpp")

# Currently this is a generic programming library and involves only headers
# similar to STL or BOOST. Change this to STATIC/SHARED if any explicit
# specializations are necessary and include appropriate *.cpp files.
add_library(lp_scheduler INTERFACE)


target_sources(lp_scheduler INTERFACE "${LP_SCHEDULER_HEADERS}")
set_property(TARGET lp_scheduler APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>)


find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})  

################### GOOGLE UNIT TESTS ###################################
if (BUILD_UNIT_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)
  
  include_directories(${GTEST_INCLUDE_DIRS})

  file(GLOB SCHEDULER_UNIT_TEST_SRCS 
       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR}/scheduler/*_test.cpp )

  add_executable(scheduler_unit_tests ${SCHEDULER_UNIT_TEST_SRCS})
  target_link_libraries(scheduler_unit_tests PRIVATE ${GTEST_LIBRARIES}
    lp_scheduler gtest_main Threads::Threads)

#  find_package(benchmark REQUIRED)
#  file(GLOB SCHEDULER_MICRO_BENCHMARK_SRCS
#       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
#       ${CMAKE_CURRENT_SOURCE_DIR}/*_micro_benchmarks.cpp )
#  add_executable(scheduler_micro_benchmarks ${SCHEDULER_MICRO_BENCHMARK_SRCS} )
#  target_link_libraries(scheduler_micro_benchmarks ${GTEST_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT} cmg-abraca benchmark::benchmark 
#          OpenMP::OpenMP_CXX ${MKL_LIBS} ${CMAKE_DL_LIBS})
endif()
################################################################################

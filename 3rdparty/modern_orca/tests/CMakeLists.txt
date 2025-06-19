find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE 
  modern_orca::modern_orca 
  GTest::gtest 
  GTest::gtest_main 
  Threads::Threads
)

# add_executable(test_agents test_agents.cpp)
# target_link_libraries(test_agents PRIVATE modern_orca::modern_orca GTest::gtest GTest::gtest_main Threads::Threads)

# add_executable(test_constraints test_constraints.cpp)
# target_link_libraries(test_constraints PRIVATE modern_orca::modern_orca GTest::gtest GTest::gtest_main Threads::Threads)

# add_executable(test_solver test_solver.cpp)
# target_link_libraries(test_solver PRIVATE modern_orca::modern_orca GTest::gtest GTest::gtest_main Threads::Threads)

# add_executable(test_simulator test_simulator.cpp)
# target_link_libraries(test_simulator PRIVATE modern_orca::modern_orca GTest::gtest GTest::gtest_main Threads::Threads)

if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(test_types PRIVATE OpenMP::OpenMP_CXX)
  # target_link_libraries(test_agents PRIVATE OpenMP::OpenMP_CXX)
  # target_link_libraries(test_constraints PRIVATE OpenMP::OpenMP_CXX)
  # target_link_libraries(test_solver PRIVATE OpenMP::OpenMP_CXX)
  # target_link_libraries(test_simulator PRIVATE OpenMP::OpenMP_CXX)
endif()

include(CTest)
include(GoogleTest)
gtest_discover_tests(test_types)
# gtest_discover_tests(test_agents)
# gtest_discover_tests(test_constraints)
# gtest_discover_tests(test_solver)
# gtest_discover_tests(test_simulator)

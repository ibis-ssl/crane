find_package(Threads REQUIRED)

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(Catch2)

add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE modern_orca::modern_orca Catch2::Catch2WithMain Threads::Threads)

add_executable(test_agents test_agents.cpp)
target_link_libraries(test_agents PRIVATE modern_orca::modern_orca Catch2::Catch2WithMain Threads::Threads)

add_executable(test_constraints test_constraints.cpp)
target_link_libraries(test_constraints PRIVATE modern_orca::modern_orca Catch2::Catch2WithMain Threads::Threads)

add_executable(test_solver test_solver.cpp)
target_link_libraries(test_solver PRIVATE modern_orca::modern_orca Catch2::Catch2WithMain Threads::Threads)

add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE modern_orca::modern_orca Catch2::Catch2WithMain Threads::Threads)

if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(test_types PRIVATE OpenMP::OpenMP_CXX)
  target_link_libraries(test_agents PRIVATE OpenMP::OpenMP_CXX)
  target_link_libraries(test_constraints PRIVATE OpenMP::OpenMP_CXX)
  target_link_libraries(test_solver PRIVATE OpenMP::OpenMP_CXX)
  target_link_libraries(test_simulator PRIVATE OpenMP::OpenMP_CXX)
endif()

include(CTest)
include(Catch)
catch_discover_tests(test_types)
catch_discover_tests(test_agents)
catch_discover_tests(test_constraints)
catch_discover_tests(test_solver)
catch_discover_tests(test_simulator)

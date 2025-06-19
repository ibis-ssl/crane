find_package(Threads REQUIRED)

add_executable(basic_example basic_example.cpp)
target_link_libraries(basic_example 
  modern_orca::modern_orca 
  Threads::Threads
  $<$<TARGET_EXISTS:OpenMP::OpenMP_CXX>:OpenMP::OpenMP_CXX>
)

add_executable(custom_constraints_example custom_constraints_example.cpp)
target_link_libraries(custom_constraints_example 
  modern_orca::modern_orca 
  Threads::Threads
  $<$<TARGET_EXISTS:OpenMP::OpenMP_CXX>:OpenMP::OpenMP_CXX>
)

add_executable(crane_integration_example crane_integration_example.cpp)
target_link_libraries(crane_integration_example 
  modern_orca::modern_orca 
  Threads::Threads
  $<$<TARGET_EXISTS:OpenMP::OpenMP_CXX>:OpenMP::OpenMP_CXX>
)

add_executable(simple_test simple_test.cpp)
target_link_libraries(simple_test 
  modern_orca::modern_orca 
  Threads::Threads
)
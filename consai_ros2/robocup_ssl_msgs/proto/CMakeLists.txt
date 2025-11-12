# Protobuf generation for robocup_ssl_msgs

# List all proto files
set(PROTO_FILES
  grSim_Commands.proto
  grSim_Packet.proto
  grSim_Replacement.proto
  grSim_Robotstatus.proto
  ssl_gc_common.proto
  ssl_gc_game_event.proto
  ssl_gc_geometry.proto
  ssl_gc_referee_message.proto
  ssl_vision_geometry.proto
  ssl_vision_detection.proto
  ssl_vision_detection_tracked.proto
  ssl_vision_wrapper.proto
  ssl_vision_wrapper_tracked.proto
  ssl_simulation_config.proto
  ssl_simulation_control.proto
  ssl_simulation_error.proto
  ssl_simulation_robot_control.proto
  ssl_simulation_robot_feedback.proto
  ssl_simulation_synchronous.proto
)

# Generate Python protobuf code
protobuf_generate(
  LANGUAGE python
  OUT_VAR proto_py_sources
  PROTOS ${PROTO_FILES}
)

set(PROTO_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Python library target (used as dependency for proto2ros_generate)
add_custom_target(${PROJECT_NAME}_proto_gen ALL DEPENDS ${proto_py_sources})

# Generate C++ protobuf code
protobuf_generate(
  LANGUAGE cpp
  OUT_VAR proto_cpp_sources
  PROTOS ${PROTO_FILES}
)

# C++ library target
add_library(${PROJECT_NAME}_proto SHARED ${proto_cpp_sources})
target_link_libraries(${PROJECT_NAME}_proto protobuf::libprotobuf)
target_include_directories(${PROJECT_NAME}_proto PUBLIC
  $<BUILD_INTERFACE:${PROTO_OUT_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install generated headers
install(
  DIRECTORY ${PROTO_OUT_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h"
)

# Install library
install(
  TARGETS ${PROJECT_NAME}_proto
  EXPORT ${PROJECT_NAME}_proto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

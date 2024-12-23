find_package(Protobuf REQUIRED)

protobuf_generate(
        LANGUAGE python
        OUT_VAR proto_py_sources
        PROTOS ObjectsArray.proto
)

protobuf_generate(
        LANGUAGE cpp
        OUT_VAR proto_cpp_sources
        PROTOS ObjectsArray.proto
)

add_library(${PROJECT_NAME}_proto SHARED ${proto_cpp_sources} )
#target_include_directories(${PROJECT_NAME}_proto PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_options(${PROJECT_NAME}_proto PUBLIC -Wno-deprecated-declarations)
target_link_libraries(${PROJECT_NAME}_proto protobuf::libprotobuf)

add_custom_target(
        ${PROJECT_NAME}_proto_gen ALL DEPENDS
        ${proto_py_sources}
        ${proto_cpp_sources}
)

set(PROTO_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}" PARENT_SCOPE)

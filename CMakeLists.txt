cmake_minimum_required (VERSION 3.8)

project (3ds_Max_Plugin_glTF CXX)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT 3ds_Max_Plugin_glTF)

add_subdirectory("./App")

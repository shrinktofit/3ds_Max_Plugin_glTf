cmake_minimum_required (VERSION 3.8)

project (3ds_Max_Plugin_glTF CXX)

include ("./3dsMax.CMakeLists.txt")
message ("ASDK_3DS_MAX_HOME: ${ASDK_3DS_MAX_HOME}")
message ("ASDK_3DS_MAX_SDK_HOME: ${ASDK_3DS_MAX_SDK_HOME}")
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	set (3DSMAX_SDK_LIB_DIR ${ASDK_3DS_MAX_SDK_HOME}/lib/x64/Release)
else ()
	set (3DSMAX_SDK_LIB_DIR ${ASDK_3DS_MAX_SDK_HOME}/lib)
endif ()

add_subdirectory("./Apricot")
add_subdirectory("./Plugin")

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT 3ds_Max_Plugin_glTF)

# Copy plugin into 3ds Max's home after build.
add_custom_target (InstallPlugin ALL)
add_dependencies (InstallPlugin 3ds_Max_Plugin_glTF Apricot)
set (PLUGIN_DIRECTORY "${ASDK_3DS_MAX_HOME}/plugins/3ds_Max_Plugin_glTF")
add_custom_command(
    TARGET InstallPlugin
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PLUGIN_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:3ds_Max_Plugin_glTF>" ${PLUGIN_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_PDB_FILE:3ds_Max_Plugin_glTF>" ${PLUGIN_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:Apricot>" ${PLUGIN_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_PDB_FILE:Apricot>" ${PLUGIN_DIRECTORY}
)

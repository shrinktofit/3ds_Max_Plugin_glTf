cmake_minimum_required (VERSION 3.13.0)

project (3ds_Max_Plugin_glTf CXX)

set (MAIN_TARGET_NAME ${PROJECT_NAME})

file (STRINGS "../.3dsmaxstuffs/3DS_MAX_HOME" ASDK_3DS_MAX_HOME)
message ("ASDK_3DS_MAX_HOME: ${ASDK_3DS_MAX_HOME}")
file (STRINGS "../.3dsmaxstuffs/3DS_MAX_SDK_HOME" ASDK_3DS_MAX_SDK_HOME)
message ("ASDK_3DS_MAX_SDK_HOME: ${ASDK_3DS_MAX_SDK_HOME}")

file (GLOB_RECURSE sourceFiles "./Source/*.h" "./Source/*.cpp")
file (GLOB_RECURSE resourceFiles "./Resource/*.rc")
file (GLOB_RECURSE defFiles "./*.def")

# ENABLE_LANGUAGE (RC)
set_source_files_properties (${resourceFiles} PROPERTIES LANGUAGE RC)

add_library (${MAIN_TARGET_NAME} SHARED ${sourceFiles} ${defFiles} ${resourceFiles})

target_compile_definitions (
	${PROJECT_NAME}
	PRIVATE
	WIN32
	_WINDOWS
	_UNICODE
	UNICODE
	PUGIXML_WCHAR_MODE
	_USRDLL
	NOMINMAX
)

# set_target_properties (${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
target_compile_options(${PROJECT_NAME} PUBLIC "/std:c++latest")

set_target_properties(${PROJECT_NAME} PROPERTIES RESOURCE ${resourceFiles} )

set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".dle")

target_include_directories (${PROJECT_NAME} PRIVATE "./Source")
target_include_directories (${PROJECT_NAME} PRIVATE "./Resource")

target_include_directories (${MAIN_TARGET_NAME} PRIVATE ${ASDK_3DS_MAX_SDK_HOME}/include)
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	set (3DSMAX_SDK_LIB_DIR ${ASDK_3DS_MAX_SDK_HOME}/lib/x64/Release)
else ()
	set (3DSMAX_SDK_LIB_DIR ${ASDK_3DS_MAX_SDK_HOME}/lib)
endif ()
target_link_libraries(
	${MAIN_TARGET_NAME}
	${3DSMAX_SDK_LIB_DIR}/core.lib
	${3DSMAX_SDK_LIB_DIR}/geom.lib
	${3DSMAX_SDK_LIB_DIR}/gfx.lib
	${3DSMAX_SDK_LIB_DIR}/mesh.lib
	${3DSMAX_SDK_LIB_DIR}/maxutil.lib
	${3DSMAX_SDK_LIB_DIR}/maxscrpt.lib
	${3DSMAX_SDK_LIB_DIR}/paramblk2.lib
	${3DSMAX_SDK_LIB_DIR}/igame.lib
)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} nlohmann_json)

# Copy plugin into 3ds Max's home after build.

set (PLUGIN_DIRECTORY "${ASDK_3DS_MAX_HOME}/plugins")

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${MAIN_TARGET_NAME}>" ${PLUGIN_DIRECTORY}
)

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_PDB_FILE:${MAIN_TARGET_NAME}>" ${PLUGIN_DIRECTORY}
)

# Setup Visual Studio debugger.

file (TO_NATIVE_PATH "${ASDK_3DS_MAX_HOME}/3dsmax.exe" ADSK_3DS_MAX_EXECUTABLE)

set_target_properties(${MAIN_TARGET_NAME} PROPERTIES
	VS_DEBUGGER_COMMAND ${ADSK_3DS_MAX_EXECUTABLE}
	)
cmake_minimum_required (VERSION 3.8)

project (3ds_Max_Plugin_glTf CXX)

file (GLOB_RECURSE sourceFiles "./Source/*.h" "./Source/*.cpp")
file (GLOB_RECURSE resourceFiles "./Resource/*.rc")
file (GLOB_RECURSE defFiles "./*.def")

# ENABLE_LANGUAGE (RC)
set_source_files_properties (${resourceFiles} PROPERTIES LANGUAGE RC)

add_library (${PROJECT_NAME} SHARED ${sourceFiles} ${defFiles} ${resourceFiles})

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

if (DEFINED ENV{ADSK_3DSMAX_SDK_2019})
	set (3DSMAX_SDK_HOME $ENV{ADSK_3DSMAX_SDK_2019})
endif ()

if (DEFINED 3DSMAX_SDK_HOME)
	message(STATUS "Found 3ds Max SDK: ${3DSMAX_SDK_HOME}")
else ()
	message(FATAL_ERROR "Could not find 3ds Max SDK.")
endif ()

target_include_directories(
	${PROJECT_NAME}
	PRIVATE
	${3DSMAX_SDK_HOME}/include
)

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	set (3DSMAX_SDK_LIB_DIR ${3DSMAX_SDK_HOME}/lib/x64/Release)
else ()
	set (3DSMAX_SDK_LIB_DIR ${3DSMAX_SDK_HOME}/lib)
endif ()

target_link_libraries(
	${PROJECT_NAME}
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

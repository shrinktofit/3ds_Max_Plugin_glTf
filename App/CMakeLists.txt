cmake_minimum_required (VERSION 3.13.0)

include ("../3dsMax.CMakeLists.txt")
message ("ASDK_3DS_MAX_HOME: ${ASDK_3DS_MAX_HOME}")
message ("ASDK_3DS_MAX_SDK_HOME: ${ASDK_3DS_MAX_SDK_HOME}")

file (GLOB_RECURSE sourceFiles "./Source/*.h" "./Source/*.cpp")
file (GLOB_RECURSE resourceFiles "./Resource/*.rc")
file (GLOB_RECURSE defFiles "./*.def")

## ENABLE_LANGUAGE (RC)
## set_source_files_properties (${resourceFiles} PROPERTIES LANGUAGE RC)

add_library (3ds_Max_Plugin_glTF SHARED ${sourceFiles} ${defFiles} ${resourceFiles})

# https://github.com/Microsoft/vcpkg/issues/6279
target_compile_definitions (
	3ds_Max_Plugin_glTF
	PRIVATE
	WIN32
	_WINDOWS
	_UNICODE
	UNICODE
	PUGIXML_WCHAR_MODE
	_USRDLL
	NOMINMAX
	WXUSINGDLL
)

set(rc_flags "/D _MSC_VER")
set(CMAKE_RC_FLAGS ${rc_flags})

# set_target_properties (${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
target_compile_options(3ds_Max_Plugin_glTF PUBLIC "/std:c++latest")

set_target_properties(3ds_Max_Plugin_glTF PROPERTIES RESOURCE ${resourceFiles} )
set_target_properties(3ds_Max_Plugin_glTF PROPERTIES SUFFIX ".dle")

target_include_directories (3ds_Max_Plugin_glTF PRIVATE "./Source")
target_include_directories (3ds_Max_Plugin_glTF PRIVATE "./Resource")
target_include_directories (3ds_Max_Plugin_glTF PRIVATE ${ASDK_3DS_MAX_SDK_HOME}/include)

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	set (3DSMAX_SDK_LIB_DIR ${ASDK_3DS_MAX_SDK_HOME}/lib/x64/Release)
else ()
	set (3DSMAX_SDK_LIB_DIR ${ASDK_3DS_MAX_SDK_HOME}/lib)
endif ()
target_link_libraries(
	3ds_Max_Plugin_glTF
	${3DSMAX_SDK_LIB_DIR}/core.lib
    ${3DSMAX_SDK_LIB_DIR}/bmm.lib
	${3DSMAX_SDK_LIB_DIR}/geom.lib
	${3DSMAX_SDK_LIB_DIR}/gfx.lib
	${3DSMAX_SDK_LIB_DIR}/mesh.lib
	${3DSMAX_SDK_LIB_DIR}/maxutil.lib
	${3DSMAX_SDK_LIB_DIR}/maxscrpt.lib
	${3DSMAX_SDK_LIB_DIR}/paramblk2.lib
	${3DSMAX_SDK_LIB_DIR}/igame.lib
)

find_package(glm CONFIG REQUIRED)
target_link_libraries(3ds_Max_Plugin_glTF glm)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(3ds_Max_Plugin_glTF nlohmann_json)

# https://github.com/microsoft/vcpkg/issues/4756
# find_package(wxWidgets CONFIG REQUIRED)
# target_link_libraries(3ds_Max_Plugin_glTF wxwidget)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
	find_library (WX_BASE_LIBRARY "wxbase31u.lib")
	find_library (WX_CORE_LIBRARY "wxmsw31u_core.lib")
	find_library (WX_UI_LIBRARY "wxmsw31u_aui.lib")
else ()
	find_library (WX_BASE_LIBRARY "wxbase31ud.lib")
	find_library (WX_CORE_LIBRARY "wxmsw31ud_core.lib")
	find_library (WX_UI_LIBRARY "wxmsw31ud_aui.lib")
endif ()
## message("wxWidgets libraries: ${WX_BASE_LIBRARY} ${WX_CORE_LIBRARY} ${WX_UI_LIBRARY}")
target_link_libraries(3ds_Max_Plugin_glTF ${WX_BASE_LIBRARY} ${WX_CORE_LIBRARY} ${WX_UI_LIBRARY})

# Copy plugin into 3ds Max's home after build.

set (PLUGIN_DIRECTORY "${ASDK_3DS_MAX_HOME}/plugins")

add_custom_command(
    TARGET 3ds_Max_Plugin_glTF
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:3ds_Max_Plugin_glTF>" ${PLUGIN_DIRECTORY}
)

add_custom_command(
    TARGET 3ds_Max_Plugin_glTF
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_PDB_FILE:3ds_Max_Plugin_glTF>" ${PLUGIN_DIRECTORY}
)

# Setup Visual Studio debugger.

file (TO_NATIVE_PATH "${ASDK_3DS_MAX_HOME}/3dsmax.exe" ADSK_3DS_MAX_EXECUTABLE)
file (TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../export_test.ms" BOOT_MAXSCRIPT)

set_target_properties(3ds_Max_Plugin_glTF PROPERTIES
	VS_DEBUGGER_COMMAND ${ADSK_3DS_MAX_EXECUTABLE}
    VS_DEBUGGER_COMMAND_ARGUMENTS "\"-silent\" \"-U\" \"MAXScript\" \"${BOOT_MAXSCRIPT}\""
	)
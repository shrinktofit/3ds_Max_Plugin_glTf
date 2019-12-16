

file (GLOB_RECURSE PluginSouceFiles "./Source/*.h" "./Source/*.cpp")
file (GLOB_RECURSE PluginResourceFiles "./Resource/*.rc")
file (GLOB_RECURSE PluginDefFiles "./*.def")

add_library (3ds_Max_Plugin_glTF SHARED ${PluginSouceFiles} ${PluginResourceFiles} ${PluginDefFiles})
# set_target_properties (3ds_Max_Plugin_glTF PROPERTIES CXX_STANDARD 17)
target_compile_options(3ds_Max_Plugin_glTF PUBLIC "/std:c++latest")

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
)

set(rc_flags "/D _MSC_VER")
set(CMAKE_RC_FLAGS ${rc_flags})
## ENABLE_LANGUAGE (RC)
## set_source_files_properties (${resourceFiles} PROPERTIES LANGUAGE RC)

set_target_properties(3ds_Max_Plugin_glTF PROPERTIES RESOURCE ${PluginResourceFiles} )
set_target_properties(3ds_Max_Plugin_glTF PROPERTIES SUFFIX ".dle")

target_include_directories (3ds_Max_Plugin_glTF PRIVATE "./Source")
target_include_directories (3ds_Max_Plugin_glTF PRIVATE "./Resource")
target_include_directories (3ds_Max_Plugin_glTF PRIVATE "../Apricot/Source")
target_include_directories (3ds_Max_Plugin_glTF PRIVATE ${ASDK_3DS_MAX_SDK_HOME}/include)

target_link_libraries(
	3ds_Max_Plugin_glTF
	Apricot
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





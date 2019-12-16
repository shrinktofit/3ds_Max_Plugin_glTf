
cmake_minimum_required (VERSION 3.13.0)

file (GLOB_RECURSE ApricotSourceFiles "./Source/*.h" "./Source/*.cpp")

add_library (Apricot SHARED ${ApricotSourceFiles})

set_target_properties(Apricot PROPERTIES LINKER_LANGUAGE CXX)

# set_target_properties (Apricot PROPERTIES CXX_STANDARD 17)
target_compile_options(Apricot PUBLIC "/std:c++latest")

# https://github.com/Microsoft/vcpkg/issues/6279
target_compile_definitions (
	Apricot
	PRIVATE
	APRICOT_EXPORTS_API
	WIN32
	_WINDOWS
	_UNICODE
	UNICODE
	PUGIXML_WCHAR_MODE
	_USRDLL
	NOMINMAX
)

target_include_directories (Apricot PRIVATE "./Source")
target_include_directories (Apricot PRIVATE ${ASDK_3DS_MAX_SDK_HOME}/include)

target_link_libraries(
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

find_package(glm CONFIG REQUIRED)
target_link_libraries(Apricot glm)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(Apricot nlohmann_json)

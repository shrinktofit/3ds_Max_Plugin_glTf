add_requires("nlohmann_json")
add_requires("vcpkg::ms-gsl", {alias = "ms-mgsl"})
target("3dsMax-glTF-plugin")
    set_kind("shared")
    set_default(true)
    add_files("App/Source/**.cpp")
    set_languages("cxx17")
    add_defines("UNICODE", "_UNICODE", "_USRDLL", "NOMINMAX", "PUGIXML_WCHAR_MODE")
    add_cxxflags("/std:c++latest") -- C++ 20 is needed
    add_includedirs("App/Source", "App/Resource")
    add_packages("nlohmann_json", "ms-mgsl")
    on_load(function (target)
        import("find3dsMaxSDK")
        local adsk3dsMaxSDKHome = find3dsMaxSDK.find()
        if adsk3dsMaxSDKHome ~= nil then 
            print('3ds Max SDK: %s', adsk3dsMaxSDKHome)
            local adsk3dsMaxIncludeDir = path.join(adsk3dsMaxSDKHome, 'include')
            local adsk3dsMaxLibDir = path.join(adsk3dsMaxSDKHome, 'lib', 'x64', 'Release')
            target:add("includedirs", adsk3dsMaxIncludeDir)
            target:add("linkdirs", adsk3dsMaxLibDir)
            target:add("links", 'core')
        end
    end)
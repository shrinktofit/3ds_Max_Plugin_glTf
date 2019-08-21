add_requires("nlohmann_json")
add_requires("vcpkg::ms-gsl")
target("3dsMax-glTF-plugin")
    set_kind("shared")
    set_default(true)
    add_files(path.join("App", "Source", "**.cpp"))
    set_languages("cxx17")
    add_defines("UNICODE", "_UNICODE", "_USRDLL", "NOMINMAX", "PUGIXML_WCHAR_MODE")
    add_cxxflags("/std:c++latest") -- C++ 20 is needed
    add_includedirs(path.join("App", "Source"), path.join("App", "Resource"))
    add_packages("nlohmann_json", "ms-gsl")
    on_load(function (target)
        import("find3dsMaxSDK")
        adsk3dsMaxSDKHome = find3dsMaxSDK.find()
        if adsk3dsMaxSDKHome ~= nil then 
            print('3ds Max SDK: ' .. adsk3dsMaxSDKHome)
            adsk3dsMaxIncludeDir = path.join(adsk3dsMaxSDKHome, 'include')
            adsk3dsMaxLibDir = path.join(adsk3dsMaxSDKHome, 'lib', 'x64', 'Release')
            target:add("includedirs", adsk3dsMaxIncludeDir)
            target:add("linkdirs", adsk3dsMaxLibDir)
            target:add("links", 'core')
        end

        import("lib.detect.find_path")
        local gslIncludeDir = find_path("gsl/span")
        print("gslIncludeDir " .. gslIncludeDir)
    end)
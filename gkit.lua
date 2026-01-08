workspace "gKit3"
    configurations { "debug", "release" }

    filter { "configurations:debug" }
        targetdir "bin/debug"
        defines { "DEBUG" }
        symbols "on"
    
    filter { "configurations:release" }
        targetdir "bin/release"
--~ 		defines { "NDEBUG" }
--~ 		defines { "GK_RELEASE" }
        optimize "full"
        
    filter { "system:linux" }
        cppdialect "c++14"
        buildoptions { "-mtune=native -march=native" }
        buildoptions { "-Wall -Wsign-compare -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable", "-pipe" }
    
    filter { "system:linux", "configurations:release" }
        buildoptions { "-flto"}
        linkoptions { "-flto=auto"}
    
    filter { "system:windows" }
        location "build"
        debugdir "."
        
        defines { "WIN32", "_USE_MATH_DEFINES", "_CRT_SECURE_NO_WARNINGS" }
        defines { "NOMINMAX" } -- allow std::min() and std::max() in vc++ :(((

    filter { "system:windows", "action:vs*" }
        location "build"
        debugdir "."
        
        system "Windows"
        architecture "x64"
        disablewarnings { "4244", "4305" }
        flags { "MultiProcessorCompile", "NoMinimalRebuild" }
        
        cppdialect "c++14"
        vectorextensions "avx2"
        floatingpoint "strict"
        
    filter { "system:macosx" }
        buildoptions { "-Wno-deprecated-declarations" }
        
        cppdialect "c++14"
        frameworks= "-F /Library/Frameworks/"
    

    project "libgkit3"
        language "C++"
        kind "StaticLib"
        targetdir "bin"
        files { "src/*.cpp", "src/*.h" }
        includedirs { ".", "src" }
        
        filter { "configurations:release" }
            openmp "on"

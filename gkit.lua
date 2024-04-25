workspace "gKit3"
    configurations { "debug", "release" }

    filter "configurations:debug"
        targetdir "bin/debug"
        defines { "DEBUG" }
		symbols "on"
    
    filter "configurations:release"
        targetdir "bin/release"
--~ 		defines { "NDEBUG" }
--~ 		defines { "GK_RELEASE" }
		optimize "speed"
        
    filter "system:linux"
        buildoptions { "-mtune=native -march=native" }
        buildoptions { "-std=c++11" }
        buildoptions { "-W -Wall -Wextra -Wsign-compare -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable", "-pipe" }
    
    filter { "system:linux", "configurations:debug" }
        buildoptions { "-g"}
        linkoptions { "-g"}
    
    filter { "system:linux", "configurations:release" }
        buildoptions { "-fopenmp" }
        linkoptions { "-fopenmp" }
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
        
--~         includedirs { "extern/visual/include" }
--~         libdirs { "extern/visual/lib" }
--~         links { "opengl32", "glew32", "SDL2", "SDL2main", "SDL2_image" }
    
    filter "system:macosx"
        frameworks= "-F /Library/Frameworks/"
        buildoptions { "-std=c++14 -Wno-deprecated-declarations" }
--~         defines { "GK_MACOS" }
        buildoptions { frameworks }
--~         linkoptions { frameworks .. " -framework OpenGL -framework SDL2 -framework SDL2_image" }
    

--~ 	includedirs { ".", "src" }
--~ 	gkit_files = { "src/*.cpp", "src/*.h" }

	project "libgkit3"
		language "C++"
		kind "StaticLib"
		targetdir "bin"
		files { "src/*.cpp", "src/*.h" }
		includedirs { ".", "src" }


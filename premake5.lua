-- Index-Physics build script
-- Generate project files with: Vendor\Bin\premake5.exe vs2022   (or run Setup.bat)

workspace "Index-Physics"
    architecture "x86_64"
    startproject "Tests"

    configurations
    {
        "Debug",
        "Release",
        "Dist"
    }

    flags
    {
        "MultiProcessorCompile"
    }

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "Index-Physics"
    location "."
    kind "StaticLib"
    language "C++"
    cppdialect "C++latest"
    cdialect "C17"
    staticruntime "off"
    warnings "Extra"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir    ("bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "Include/**.hpp",
        "Include/**.h",
        "Src/**.cpp",
        "Src/**.hpp",
        "Src/**.h"
    }

    includedirs
    {
        "include",
        "external/include"
    }

    filter "system:windows"
        systemversion "latest"
        defines { "_CRT_SECURE_NO_WARNINGS" }

    filter "configurations:Debug"
        defines { "INDEX_PHYS_DEBUG", "_DEBUG" }
        runtime "Debug"
        symbols "on"
        optimize "off"

    filter "configurations:Release"
        defines { "INDEX_PHYS_RELEASE", "NDEBUG" }
        runtime "Release"
        symbols "on"
        optimize "on"

    filter "configurations:Dist"
        defines { "INDEX_PHYS_DIST", "NDEBUG" }
        runtime "Release"
        symbols "off"
        optimize "full"

project "Tests"
    location "Tests"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++latest"
    cdialect "C17"
    staticruntime "off"
    warnings "Extra"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir    ("bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "Tests/**.cpp",
        "Tests/**.hpp",
        "Tests/**.h"
    }

    includedirs
    {
        "Include",
        "External/Include",
        "Tests"
    }

    links
    {
        "Index-Physics"
    }

    filter "system:windows"
        systemversion "latest"
        defines { "_CRT_SECURE_NO_WARNINGS" }

    filter "configurations:Debug"
        defines { "INDEX_PHYS_DEBUG", "_DEBUG" }
        runtime "Debug"
        symbols "on"
        optimize "off"

    filter "configurations:Release"
        defines { "INDEX_PHYS_RELEASE", "NDEBUG" }
        runtime "Release"
        symbols "on"
        optimize "on"

    filter "configurations:Dist"
        defines { "INDEX_PHYS_DIST", "NDEBUG" }
        runtime "Release"
        symbols "off"
        optimize "full"

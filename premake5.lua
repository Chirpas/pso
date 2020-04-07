-- premake file for project to generate either a gnu make file or a vsproject solution
workspace "PSO"
	architecture "x86_64"
	language "C"
	configurations {"Debug", "Release"}
	startproject "test"

	filter { "configurations:Debug" }
		symbols "On"
	filter { "configurations:Release" }
		optimize "On"
	filter { }
	
	-- output filepath
	outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"	
	targetdir ("bin/" ..outputdir.."/%{wks.name}")
	objdir ("bin-obj/" ..outputdir.."/%{wks.name}")
	
	--static pso library
	project "PSOlib"
		kind "StaticLib"
	
		files {
			"src/pso/pso.c",
			"src/pso/pso.h"
		}
		
	project "test"
		kind "ConsoleApp"

		includedirs "src/pso"
		links{"PSOlib"}
		
		files {
			"src/test.c"
		}
		


		
	
	
	-- output filepath
--outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"
--	targetdir ("bin/" ..outputdir.."/pso")
--	objdir ("bin-obj/" ..outputdir.."/pso")
--	debugdir "bin/%{outputdir}/pso"

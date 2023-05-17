add_rules("mode.debug", "mode.release")
add_requires("yaml-cpp", "eigen", "fmt", "onetbb", "openmp")	--to get the package 'name'
set_languages("cxxlatest")	--set the configure language

if is_mode("debug") then
	-- set debug option
	add_cxflags("-g")
	add_ldflags("-g")
	-- set compile level
	set_optimize("none") -- none, fast, faster, fastest, smallest, aggressive
end

includes("physx-common/xmake.lua")

target("aero")	--set target
	set_kind("binary") --[option]: "binary", "static", "shared"
	add_files("app/*.cpp")
	add_includedirs("physx-common/Util")
	add_packages("yaml-cpp", "eigen", "fmt")
	if is_mode("release") then
        add_packages("onetbb", "openmp")
    end
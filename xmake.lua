add_rules("mode.debug", "mode.release")
set_languages("c99", "c++23")

if is_plat("linux") then 
    add_cxflags("-std=c++2b")
    add_cxxflags("-std=c++2b")
end

if is_plat("windows") then
    add_requires("catch2")
end
-- add_requires("xlnt", {configs = {shared = false}})

-- Resolve third_party paths from xmake.lua, not the shell cwd.
-- `xmake f` from third_party/cadical would otherwise miss Z3/CaDiCaL/HiGHS.
local function third_party_path(...)
    return path.join(os.projectdir(), "third_party", ...)
end

local function first_existing_dir(candidates)
    for _, dir in ipairs(candidates) do
        if dir and dir ~= "" and os.isdir(dir) then
            return dir
        end
    end
    return nil
end

-- Prefix installs on Linux often use lib64 instead of lib.
-- Prefer the directory that actually contains the named library.
local function find_lib_dir(home, libname)
    home = path.absolute(home)
    local function has_lib(dir)
        return os.isfile(path.join(dir, "lib" .. libname .. ".so"))
            or os.isfile(path.join(dir, "lib" .. libname .. ".dylib"))
            or os.isfile(path.join(dir, "lib" .. libname .. ".a"))
    end
    for _, sub in ipairs({"lib", "lib64"}) do
        local dir = path.join(home, sub)
        if has_lib(dir) then
            return dir
        end
    end
    for _, sub in ipairs({"lib", "lib64"}) do
        local dir = path.join(home, sub)
        if os.isdir(dir) then
            return dir
        end
    end
    return path.join(home, "lib")
end

local function add_z3_dependency()
    if not has_config("z3") then
        return false
    end
    local z3_home = os.getenv("Z3_HOME")
    if not z3_home or z3_home == "" then
        z3_home = os.getenv("Z3_ROOT")
    end
    if not z3_home or z3_home == "" then
        local candidates = {third_party_path("z3", "install")}
        if is_host("macosx") then
            table.insert(candidates, "/opt/homebrew/opt/z3")
            table.insert(candidates, "/usr/local/opt/z3")
        end
        z3_home = first_existing_dir(candidates)
    end
    if not z3_home or z3_home == "" then
        return false
    end
    z3_home = path.absolute(z3_home)
    local lib_dir = find_lib_dir(z3_home, "z3")
    add_includedirs(path.join(z3_home, "include"))
    add_linkdirs(lib_dir)
    add_rpathdirs(lib_dir)
    add_links("z3")
    add_defines("USE_Z3")
    if is_plat("linux") then
        add_syslinks("pthread", "dl")
    end
    return true
end

local function add_highs_dependency()
    local highs_home = os.getenv("HIGHS_HOME")
    if not highs_home or highs_home == "" then
        highs_home = os.getenv("HIGHS_ROOT")
    end
    if not highs_home or highs_home == "" then
        local candidates = {}
        if is_host("macosx") then
            table.insert(candidates, third_party_path("HiGHS", "install-macos"))
        end
        table.insert(candidates, third_party_path("HiGHS", "install"))
        highs_home = first_existing_dir(candidates)
    end
    if not highs_home or highs_home == "" then
        return false
    end
    highs_home = path.absolute(highs_home)
    local lib_dir = find_lib_dir(highs_home, "highs")
    add_includedirs(path.join(highs_home, "include", "highs"))
    add_linkdirs(lib_dir)
    add_rpathdirs(lib_dir)
    add_links("highs")
    add_defines("USE_HIGHS")
    if is_plat("linux") then
        add_syslinks("pthread", "dl", "m")
    end
    return true
end

local function add_cadical_dependency()
    if not has_config("cadical") then
        return false
    end
    local src_dir = third_party_path("cadical", "src")
    local build_dir = third_party_path("cadical", "build")
    local macos_build = third_party_path("cadical", "build-macos")
    if is_plat("macosx") and os.isfile(path.join(macos_build, "libcadical.a")) then
        build_dir = macos_build
    end
    add_defines("USE_CADICAL")
    add_includedirs(src_dir)
    add_linkdirs(build_dir)
    add_links("cadical")
    if is_plat("linux") then
        add_syslinks("pthread")
    end
    return true
end

rule("qt.opengl")
    on_config(function (target) 
        import("detect.sdks.find_qt")
        local qt = assert(find_qt(), "Qt SDK not found!")
        local major_version = tonumber(string.match(qt.sdkver, "^(%d+)"))
        if major_version >= 6 then
            target:add("frameworks", "QtOpenGL", "QtOpenGLWidgets")
        else
            target:add("frameworks", "QtOpenGL")
        end
    end)

-- PR_tool Task!!!

target("PR_tool")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    -- add_packages("xlnt")
    add_includedirs("source", "source/global")
    add_files("source/**.cc", "source/widget/**.h", "resource/resource.qrc")
    add_rules("qt.widgetapp", "qt.opengl")

-- Tool Application 

-- Simple tool for get cob index map 
target("cobmap")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global")
    add_files("tools/cobmap.cc")
    add_files("source/hardware/**.cc")
    add_files("source/global/**.cc")

-- Load config, run P&R and view result in 2D view
target("view2d")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    -- add_packages("xlnt")
    add_includedirs("source", "source/global")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc",
        "source/widget/view2d/**.cc",
        "source/widget/view2d/**.h",
        "source/widget/frame/**.h",
        "source/widget/frame/**.cc",
        "tools/view2d.cc"
    )
    add_rules("qt.widgetapp", "qt.opengl")

-- Load config, run P&R and view result in 3D view
target("view3d")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    -- add_packages("xlnt")
    add_includedirs("source", "source/global")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc",
        "source/widget/view3d/**.cc",
        "source/widget/view3d/**.h",
        "source/widget/frame/**.h",
        "source/widget/frame/**.cc",
        "resource/resource.qrc",
        "tools/view3d.cc"
    )
    add_rules("qt.widgetapp", "qt.opengl")

-- Test Tasks

target("module_test")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    -- add_packages("xlnt")
    add_includedirs("source", "source/global", "test/module_test")
    add_files("test/module_test/**.cc")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("regression_test")
    set_kind("binary")
    set_targetdir("./output")
    set_default(true)
    if is_plat("windows") then
        add_packages("catch2")
    elseif is_plat("linux") then
        add_includedirs("$(env CONDA_PREFIX)/include")
        add_linkdirs("$(env CONDA_PREFIX)/lib")
        add_links("Catch2", "Catch2Main")
        add_rpathdirs("$(env CONDA_PREFIX)/lib")
    end
    add_includedirs("source", "source/global", "test/regression_test")
    -- add_includedirs("$CONDA_PREFIX/include")
    add_files("test/regression_test/**.cc")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("transform_format")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "test/transform_format")
    add_files("test/transform_format/**.cc")
    add_files(
       "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("parse_controlbits")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global")
    add_files("tools/parse_controlbits.cc")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("test_ILP")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "algorithm/test_ILP")
    add_files(
        "algorithm/test_ILP/main.cc",
        "algorithm/test_ILP/test_ilp_cli.cc",
        "algorithm/test_ILP/scope/build_routing_nets.cc",
        "algorithm/test_ILP/scope/scope_bbox.cc",
        "algorithm/test_ILP/scope/pair_routing_state.cc",
        "algorithm/test_ILP/graph/unified_routing_graph.cc",
        "algorithm/test_ILP/global_route_v17/global_router.cc",
        "algorithm/test_ILP/common/cob_unit_mask.cc",
        "algorithm/test_ILP/delay/pair_delay_precompute.cc",
        "algorithm/test_ILP/sat/unified_sat_scope.cc",
        "algorithm/test_ILP/sat/sat_constraint_kits.cc",
        "algorithm/test_ILP/sat/sat_encoding_stats.cc",
        "algorithm/test_ILP/sat/unified_sat_encoder.cc",
        "algorithm/test_ILP/sat/node_occupancy.cc",
        "algorithm/test_ILP/sat/encode_tob_special.cc",
        "algorithm/test_ILP/sat/encode_bus_sync.cc",
        "algorithm/test_ILP/sat/solve_unified_sat.cc",
        "algorithm/test_ILP/sat/routing_feedback.cc",
        "algorithm/test_ILP/sat/z3_routing_feedback.cc",
        "algorithm/test_ILP/sat/routing_round_diagnostics.cc",
        "algorithm/test_ILP/sat/ideal_shortest_wirelength.cc",
        "algorithm/test_ILP/sat/routing_solution_validate.cc",
        "algorithm/test_ILP/sat/sat_solution_extract.cc",
        "algorithm/test_ILP/sat/routing_path_log.cc",
        "algorithm/test_ILP/sat_allocation/cadical_solver.cc"
    )
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    add_cadical_dependency()
    if add_z3_dependency() then
        add_files("algorithm/test_ILP/sat_allocation/z3_optimize_solver.cc")
    end
    add_highs_dependency()

target("test_ILP_unit")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "algorithm/test_ILP")
    add_files(
        "algorithm/test_ILP/test/unit_main.cc",
        "algorithm/test_ILP/test_ilp_cli.cc",
        "algorithm/test_ILP/scope/build_routing_nets.cc",
        "algorithm/test_ILP/scope/scope_bbox.cc",
        "algorithm/test_ILP/scope/pair_routing_state.cc",
        "algorithm/test_ILP/graph/unified_routing_graph.cc",
        "algorithm/test_ILP/global_route_v17/global_router.cc",
        "algorithm/test_ILP/common/cob_unit_mask.cc",
        "algorithm/test_ILP/delay/pair_delay_precompute.cc",
        "algorithm/test_ILP/sat/unified_sat_scope.cc",
        "algorithm/test_ILP/sat/sat_constraint_kits.cc",
        "algorithm/test_ILP/sat/sat_encoding_stats.cc",
        "algorithm/test_ILP/sat/unified_sat_encoder.cc",
        "algorithm/test_ILP/sat/node_occupancy.cc",
        "algorithm/test_ILP/sat/encode_tob_special.cc",
        "algorithm/test_ILP/sat/encode_bus_sync.cc",
        "algorithm/test_ILP/sat/solve_unified_sat.cc",
        "algorithm/test_ILP/sat/routing_feedback.cc",
        "algorithm/test_ILP/sat/z3_routing_feedback.cc",
        "algorithm/test_ILP/sat/routing_round_diagnostics.cc",
        "algorithm/test_ILP/sat/ideal_shortest_wirelength.cc",
        "algorithm/test_ILP/sat/routing_solution_validate.cc",
        "algorithm/test_ILP/sat/sat_solution_extract.cc",
        "algorithm/test_ILP/sat/routing_path_log.cc",
        "algorithm/test_ILP/sat_allocation/cadical_solver.cc"
    )
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    add_cadical_dependency()
    if add_z3_dependency() then
        add_files("algorithm/test_ILP/sat_allocation/z3_optimize_solver.cc")
    end
    add_highs_dependency()

local function add_weighted_maxsat_sources()
    add_includedirs("source", "source/global", "algorithm/test_ILP", "algorithm/weighted_maxsat")
    add_files(
        "algorithm/weighted_maxsat/wmaxsat_cli.cc",
        "algorithm/weighted_maxsat/wmaxsat_router.cc",
        "algorithm/test_ILP/scope/build_routing_nets.cc",
        "algorithm/test_ILP/scope/scope_bbox.cc",
        "algorithm/test_ILP/scope/pair_routing_state.cc",
        "algorithm/test_ILP/graph/unified_routing_graph.cc",
        "algorithm/test_ILP/common/cob_unit_mask.cc",
        "algorithm/test_ILP/delay/pair_delay_precompute.cc",
        "algorithm/test_ILP/sat/unified_sat_scope.cc",
        "algorithm/test_ILP/sat/sat_constraint_kits.cc",
        "algorithm/test_ILP/sat/sat_encoding_stats.cc",
        "algorithm/test_ILP/sat/unified_sat_encoder.cc",
        "algorithm/test_ILP/sat/encode_tob_special.cc",
        "algorithm/test_ILP/sat/encode_bus_sync.cc",
        "algorithm/test_ILP/sat_allocation/cadical_solver.cc",
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    add_cadical_dependency()
end

target("weighted_maxsat")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_files("algorithm/weighted_maxsat/main.cc")
    add_weighted_maxsat_sources()

target("weighted_maxsat_unit")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_files("algorithm/weighted_maxsat/test/unit_main.cc")
    add_weighted_maxsat_sources()

target("weighted_maxsat_integration")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_files("algorithm/weighted_maxsat/test/integration_main.cc")
    add_weighted_maxsat_sources()

target("FPIA_RRR")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "algorithm/FPIA-RRR")
    add_files(
        "algorithm/FPIA-RRR/main.cc",
        "algorithm/FPIA-RRR/rrr_cli.cc",
        "algorithm/FPIA-RRR/net_adapter.cc",
        "algorithm/FPIA-RRR/hardware_graph.cc",
        "algorithm/FPIA-RRR/route_log.cc",
        "algorithm/FPIA-RRR/resource_model.cc",
        "algorithm/FPIA-RRR/maze_search.cc",
        "algorithm/FPIA-RRR/rrr_router.cc",
        "algorithm/FPIA-RRR/sync_equalize.cc",
        "algorithm/FPIA-RRR/route_validate.cc"
    )
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("FPIA_RRR_unit")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "algorithm/FPIA-RRR")
    add_files(
        "algorithm/FPIA-RRR/test/unit_main.cc",
        "algorithm/FPIA-RRR/test/tob_mux_fanout.cc",
        "algorithm/FPIA-RRR/rrr_cli.cc",
        "algorithm/FPIA-RRR/net_adapter.cc",
        "algorithm/FPIA-RRR/hardware_graph.cc",
        "algorithm/FPIA-RRR/route_log.cc",
        "algorithm/FPIA-RRR/resource_model.cc",
        "algorithm/FPIA-RRR/maze_search.cc",
        "algorithm/FPIA-RRR/rrr_router.cc",
        "algorithm/FPIA-RRR/sync_equalize.cc",
        "algorithm/FPIA-RRR/route_validate.cc"
    )
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("FPIA_RRR_mux_test")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "algorithm/FPIA-RRR")
    add_files(
        "algorithm/FPIA-RRR/test/tob_mux_cases.cc",
        "algorithm/FPIA-RRR/test/tob_mux_fanout.cc",
        "algorithm/FPIA-RRR/rrr_cli.cc",
        "algorithm/FPIA-RRR/net_adapter.cc",
        "algorithm/FPIA-RRR/hardware_graph.cc",
        "algorithm/FPIA-RRR/route_log.cc",
        "algorithm/FPIA-RRR/resource_model.cc",
        "algorithm/FPIA-RRR/maze_search.cc",
        "algorithm/FPIA-RRR/rrr_router.cc",
        "algorithm/FPIA-RRR/sync_equalize.cc",
        "algorithm/FPIA-RRR/route_validate.cc"
    )
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )

target("wirelength_study")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs(
        "algorithm/test_ILP",
        "source",
        "source/global")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc")
    local gurobi_home = os.getenv("GUROBI_HOME")
    if not gurobi_home or gurobi_home == "" then
      if is_plat("linux") then
        gurobi_home = "/opt/gurobi1302/linux64"
      else
        gurobi_home = "/Library/gurobi1302/macos_universal2"
      end
    end
    add_includedirs(gurobi_home .. "/include")
    add_linkdirs(gurobi_home .. "/lib")
    add_rpathdirs(gurobi_home .. "/lib")
    if is_plat("linux") then
      add_links("pthread", "dl", "m")
    end
    add_links("gurobi_c++", "gurobi130")

target("tob_sat_assign")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "algorithm/test_ILP")
    add_files(
        "algorithm/TOBindex/tob_sat_assign.cc",
        "algorithm/test_ILP/sat_allocation/cadical_solver.cc",
        "algorithm/test_ILP/sat/sat_constraint_kits.cc",
        "algorithm/test_ILP/sat/sat_encoding_stats.cc",
        "source/global/**.cc"
    )
    add_cadical_dependency()

-- tools

option("cadical")

    set_default(true)

    set_showmenu(true)

    set_description("Enable CaDiCaL SAT solver")

option("z3")

    set_default(true)

    set_showmenu(true)

    set_description("Enable Z3 Optimize backend when a Z3 installation is found")


-- xmake project -k compile_commands

--
-- If you want to known more usage about xmake, please see https://xmake.io
--
-- ## FAQ
--
-- You can enter the project directory firstly before building project.
--
--   $ cd projectdir
--
-- 1. How to build project?
--
--   $ xmake
--
-- 2. How to configure project?
--
--   $ xmake f -p [macosx|linux|iphoneos ..] -a [x86_64|i386|arm64 ..] -m [debug|release]
--
-- 3. Where is the build output directory?
--
--   The default output directory is `./build` and you can configure the output directory.
--
--   $ xmake f -o outputdir
--   $ xmake
--
-- 4. How to run and debug target after building project?
--
--   $ xmake run [targetname]
--   $ xmake run -d [targetname]
--
-- 5. How to install target to the system directory or other output directory?
--
--   $ xmake install
--   $ xmake install -o installdir
--
-- 6. Add some frequently-used compilation flags in xmake.lua
--
-- @code
--    -- add debug and release modes
--    add_rules("mode.debug", "mode.release")
--
--    -- add macro definition
--    add_defines("NDEBUG", "_GNU_SOURCE=1")
--
--    -- set warning all as error
--    set_warnings("all", "error")
--
--    -- set language: c99, c++11
--    set_languages("c99", "c++11")
--
--    -- set optimization: none, faster, fastest, smallest
--    set_optimize("fastest")
--
--    -- add include search directories
--    add_includedirs("/usr/include", "/usr/local/include")
--
--    -- add link libraries and search directories
--    add_links("tbox")
--    add_linkdirs("/usr/local/lib", "/usr/lib")
--
--    -- add system link libraries
--    add_syslinks("z", "pthread")
--
--    -- add compilation and link flags
--    add_cxflags("-stdnolib", "-fno-strict-aliasing")
--    add_ldflags("-L/usr/local/lib", "-lpthread", {force = true})
--
-- @endcode
--

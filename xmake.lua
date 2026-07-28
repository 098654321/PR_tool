add_rules("mode.debug", "mode.release")
set_languages("c99", "c++20")

if is_plat("linux") then 
    set_languages("c++20")
end

if is_plat("windows", "macosx") then
    add_requires("catch2")
end
-- add_requires("xlnt", {configs = {shared = false}})

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

local function add_sat_ilp_deps()
    if has_config("cadical") then
        add_defines("USE_CADICAL")
        add_includedirs("third_party/cadical/src")
        add_linkdirs("third_party/cadical/build")
        add_links("cadical")
        if is_plat("linux") then
            add_syslinks("pthread")
        end
    end
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
end

-- PR_tool Task!!!

target("PR_tool")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    -- add_packages("xlnt")
    add_includedirs("source", "source/global")
    add_files("source/**.cc", "source/widget/**.h", "resource/resource.qrc")
    add_rules("qt.widgetapp", "qt.opengl")
    if has_config("sat_router") then
        add_defines("PR_TOOL_HAS_SAT_ROUTER=1")
        add_includedirs("source/algo/router/sat_ilp")
        add_sat_ilp_deps()
    else
        remove_files("source/algo/router/sat_ilp/**.cc")
        remove_files("source/algo/router/backend/sat_backend.cc")
        remove_files("source/algo/router/sat_ilp/commit_paths.cc")
        add_defines("PR_TOOL_HAS_SAT_ROUTER=0")
    end

target("PR_tool_cli")
    set_kind("binary")
    set_targetdir("./output")
    set_basename("PR_tool_cli")
    set_default(false)
    add_defines("PR_TOOL_CLI_ONLY")
    add_includedirs("source", "source/global")
    add_files(
        "source/app/main.cc",
        "source/app/PR_tool.cc",
        "source/app/cli/**.cc",
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    if has_config("sat_router") then
        add_defines("PR_TOOL_HAS_SAT_ROUTER=1")
        add_includedirs("source/algo/router/sat_ilp")
        add_sat_ilp_deps()
    else
        remove_files("source/algo/router/sat_ilp/**.cc")
        remove_files("source/algo/router/backend/sat_backend.cc")
        remove_files("source/algo/router/sat_ilp/commit_paths.cc")
        add_defines("PR_TOOL_HAS_SAT_ROUTER=0")
    end

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
    remove_files("source/algo/router/sat_ilp/**.cc")
    remove_files("source/algo/router/backend/sat_backend.cc")
    add_defines("PR_TOOL_HAS_SAT_ROUTER=0")
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
    remove_files("source/algo/router/sat_ilp/**.cc")
    remove_files("source/algo/router/backend/sat_backend.cc")
    add_defines("PR_TOOL_HAS_SAT_ROUTER=0")
    add_rules("qt.widgetapp", "qt.opengl")

-- Test Tasks

target("module_test")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_deps("PR_tool_cli")
    -- add_packages("xlnt")
    add_includedirs("source", "source/global", "test/module_test", "test/module_test/test_unit")
    add_files("test/module_test/test_unit/**.cc")
    add_files(
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    add_defines("PR_TOOL_HAS_SAT_ROUTER=1")
    add_includedirs("source/algo/router/sat_ilp")
    add_sat_ilp_deps()

target("regression_test")
    set_kind("binary")
    set_targetdir("./output")
    set_default(true)
    if is_plat("windows", "macosx") then
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
    remove_files("source/algo/router/sat_ilp/**.cc")
    remove_files("source/algo/router/backend/sat_backend.cc")
    add_defines("PR_TOOL_HAS_SAT_ROUTER=0")

target("txt2json")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "test/transform_format")
    add_files("test/transform_format/txt2json.cc")
    add_files(
       "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    remove_files("source/algo/router/sat_ilp/**.cc")
    remove_files("source/algo/router/backend/sat_backend.cc")
    add_defines("PR_TOOL_HAS_SAT_ROUTER=0")

target("json2txt")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "test/transform_format")
    add_files("test/transform_format/json2txt.cc")
    add_files(
       "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    remove_files("source/algo/router/sat_ilp/**.cc")
    remove_files("source/algo/router/backend/sat_backend.cc")
    add_defines("PR_TOOL_HAS_SAT_ROUTER=0")

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
    remove_files("source/algo/router/sat_ilp/**.cc")
    remove_files("source/algo/router/backend/sat_backend.cc")
    add_defines("PR_TOOL_HAS_SAT_ROUTER=0")

target("test_ILP")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "source/algo/router/sat_ilp", "algorithm/test_ILP")
    add_files(
        "algorithm/test_ILP/main.cc",
        "algorithm/test_ILP/test_ilp_cli.cc"
    )
    add_files(
        "source/algo/**.cc|router/sat_ilp/**",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    add_files(
        "source/algo/router/sat_ilp/scope/build_routing_nets.cc",
        "source/algo/router/sat_ilp/scope/scope_bbox.cc",
        "source/algo/router/sat_ilp/scope/pair_routing_state.cc",
        "source/algo/router/sat_ilp/graph/unified_routing_graph.cc",
        "source/algo/router/sat_ilp/common/cob_unit_mask.cc",
        "source/algo/router/sat_ilp/delay/pair_delay_precompute.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_prepare.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_domain.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_segment.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_model.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_extract.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_validate.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_optimizer.cc",
        "source/algo/router/sat_ilp/sat/unified_sat_scope.cc",
        "source/algo/router/sat_ilp/sat/sat_constraint_kits.cc",
        "source/algo/router/sat_ilp/sat/sat_encoding_stats.cc",
        "source/algo/router/sat_ilp/sat/unified_sat_encoder.cc",
        "source/algo/router/sat_ilp/sat/encode_tob_special.cc",
        "source/algo/router/sat_ilp/sat/encode_bus_sync.cc",
        "source/algo/router/sat_ilp/sat/solve_unified_sat.cc",
        "source/algo/router/sat_ilp/sat/routing_feedback.cc",
        "source/algo/router/sat_ilp/sat/routing_round_diagnostics.cc",
        "source/algo/router/sat_ilp/sat/ideal_shortest_wirelength.cc",
        "source/algo/router/sat_ilp/sat/routing_solution_validate.cc",
        "source/algo/router/sat_ilp/sat/sat_solution_extract.cc",
        "source/algo/router/sat_ilp/sat/routing_path_log.cc",
        "source/algo/router/sat_ilp/sat_allocation/cadical_solver.cc"
    )
    add_sat_ilp_deps()

target("test_ILP_unit")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs("source", "source/global", "source/algo/router/sat_ilp", "algorithm/test_ILP")
    add_files(
        "algorithm/test_ILP/test/unit_main.cc",
        "algorithm/test_ILP/test_ilp_cli.cc"
    )
    add_files(
        "source/algo/**.cc|router/sat_ilp/**",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
    add_files(
        "source/algo/router/sat_ilp/scope/build_routing_nets.cc",
        "source/algo/router/sat_ilp/scope/scope_bbox.cc",
        "source/algo/router/sat_ilp/scope/pair_routing_state.cc",
        "source/algo/router/sat_ilp/graph/unified_routing_graph.cc",
        "source/algo/router/sat_ilp/common/cob_unit_mask.cc",
        "source/algo/router/sat_ilp/delay/pair_delay_precompute.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_prepare.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_domain.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_segment.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_model.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_extract.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_validate.cc",
        "source/algo/router/sat_ilp/ilp_v15/v15_ilp_optimizer.cc",
        "source/algo/router/sat_ilp/sat/unified_sat_scope.cc",
        "source/algo/router/sat_ilp/sat/sat_constraint_kits.cc",
        "source/algo/router/sat_ilp/sat/sat_encoding_stats.cc",
        "source/algo/router/sat_ilp/sat/unified_sat_encoder.cc",
        "source/algo/router/sat_ilp/sat/encode_tob_special.cc",
        "source/algo/router/sat_ilp/sat/encode_bus_sync.cc",
        "source/algo/router/sat_ilp/sat/solve_unified_sat.cc",
        "source/algo/router/sat_ilp/sat/routing_feedback.cc",
        "source/algo/router/sat_ilp/sat/routing_round_diagnostics.cc",
        "source/algo/router/sat_ilp/sat/ideal_shortest_wirelength.cc",
        "source/algo/router/sat_ilp/sat/routing_solution_validate.cc",
        "source/algo/router/sat_ilp/sat/sat_solution_extract.cc",
        "source/algo/router/sat_ilp/sat/routing_path_log.cc",
        "source/algo/router/sat_ilp/sat_allocation/cadical_solver.cc"
    )
    add_sat_ilp_deps()

target("wirelength_study")
    set_kind("binary")
    set_targetdir("./output")
    set_default(false)
    add_includedirs(
        "source/algo/router/sat_ilp",
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
    remove_files("source/algo/router/sat_ilp/**.cc")
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


-- tools

option("sat_router")
    set_default(true)
    set_showmenu(true)
    set_description("Build SAT/ILP router backend into PR_tool / PR_tool_cli")

option("cadical")

    set_default(true)

    set_showmenu(true)

    set_description("Enable CaDiCaL SAT solver")


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

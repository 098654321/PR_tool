#include <debug/debug.hh>

using namespace PR_tool;

void test_debug_main() {
    debug::initial_log("E:/PR_tool/PR_tool/PR_tool.log");

    debug::debug_fmt("Hello {}!", "PR_tool");
    debug::info_fmt("Hello {}!", "PR_tool");
    debug::warning_fmt("Hello {}!", "PR_tool");
    debug::error_fmt("Hello {}!", "PR_tool");
    // debug::fatal_fmt("Hello {}!", "PR_tool");
}
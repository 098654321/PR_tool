#include <exception>
#include <std/string.hh>
#include <std/collection.hh>
#include <global/debug/debug.hh>
#include <global/debug/console.hh>
#include <cassert>
#include <sys/types.h>


using TestFunction = void(*)(void);

extern void test_cob_main();
extern void test_interposer_main();
extern void test_tob_main();
extern void test_router_main();
extern void test_placer_main();
extern void test_debug_main();
extern void test_config_main();
extern void test_comparator_main();
extern void test_path_length_main();

#define REGISTER_TEST(test_name)\
functions.emplace(#test_name, & test_##test_name##_main);\
if (target == #test_name) {\
    PR_tool::console::println_fmt("Run test '{}'", #test_name);\
    test_##test_name##_main();\
    return 0;\
}\

int main(int argc, char** argv) 
try {
    assert(argc == 2);

    auto functions = std::HashMap<std::StringView, TestFunction>{};
    auto target = std::StringView{argv[1]};
    PR_tool::debug::set_debug_level(PR_tool::debug::DebugLevel::Info);
    PR_tool::debug::initial_log("./debug.log");

    REGISTER_TEST(cob)
    REGISTER_TEST(tob)
    REGISTER_TEST(interposer)
    REGISTER_TEST(router)
    REGISTER_TEST(placer)
    REGISTER_TEST(debug)
    REGISTER_TEST(config)
    REGISTER_TEST(comparator)
    REGISTER_TEST(path_length)

    if (target == "all") {
        for (auto [test_name, test_func] : functions) {
            PR_tool::console::println_fmt("Run test '{}'", test_name);
            test_func();
            PR_tool::console::println("");
        }
        return 0;
    }

    PR_tool::console::println_fmt("No exit test target '{}'", target);
    return 0;
}
catch (const std::exception& err) {
    PR_tool::console::error_fmt("Error in test: {}", err.what());
}
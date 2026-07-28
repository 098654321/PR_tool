#include <parse/comparator/controlbits_parser.hh>


using namespace PR_tool;


static void test_comparint_two_mode_controlbits() {
    for (int c = 1; c <= 5 ; c++) {
        std::string file1 = "..\\algorithm\\incremental_routing\\2026.01.21_test\\test1_cycles\\8\\" + std::to_string(c) + ".txt";
        std::string file2 = "..\\algorithm\\incremental_routing\\2026.01.21_test\\test1_cycles\\mode2_standard.txt";
        parse::compare(file1, file2);
    }
}

void test_comparator_main() {
    test_comparint_two_mode_controlbits();
}






#include <cstdio>
#include <vector>
#include <string>
#include "ipasir_smt.h"

#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"

// Print test result in color
void print_result(const std::string &test_name, int result, int expected) {
    printf("%sTest: %s - ", (result == expected) ? COLOR_GREEN : COLOR_RED, test_name.c_str());
    printf("Solver returned: %d (expected: %d)%s\n", result, expected, COLOR_RESET);
}

// run_test runs one test
void run_test(const std::string &test_name,
              const std::vector<std::vector<int>> &clauses,
              int expected_result,
              const std::vector<int> &vars) {
    printf("===========================================\n");
    printf("Running: %s\n", test_name.c_str());

    void *solver = ipasir_init();

    // Add clauses
    for (const auto &clause : clauses) {
        for (int lit : clause) {
            ipasir_add(solver, lit);
        }
        ipasir_add(solver, 0); // Terminate clause
    }

    // Solve
    int result = ipasir_solve(solver);
    print_result(test_name, result, expected_result);

    // If SAT, print assignments
    if (result == 10 && !vars.empty()) {
        printf("Assignment:\n");
        for (int var : vars) {
            int val = ipasir_val(solver, var);
            printf("  Variable %d = %s\n", var, (val > 0 ? "true" : "false"));
        }
    }

    ipasir_release(solver);
    printf("===========================================\n\n");
}

// Incremental solving test
void incremental_test() {
    printf("===========================================\n");
    printf("Incremental test\n");

    void *solver = ipasir_init();

    ipasir_add(solver, 1);
    ipasir_add(solver, 0);
    int result = ipasir_solve(solver);
    print_result("Incremental test (Step 1)", result, 10);

    ipasir_add(solver, -1);
    ipasir_add(solver, 0);
    result = ipasir_solve(solver);
    print_result("Incremental test (Step 2)", result, 20);

    ipasir_release(solver);
    printf("===========================================\n\n");
}

// Assumptions test
void assumptions_test() {
    printf("===========================================\n");
    printf("Assumptions test\n");

    void *solver = ipasir_init();
    ipasir_add(solver, 1);
    ipasir_add(solver, 0);

    ipasir_assume(solver, 1);
    int result = ipasir_solve(solver);
    print_result("Assumptions test (Step 1)", result, 10);

    ipasir_assume(solver, -1);
    result = ipasir_solve(solver);
    print_result("Assumptions test (Step 2)", result, 20);

    result = ipasir_solve(solver);
    print_result("Assumptions test (Step 3)", result, 10);

    ipasir_release(solver);
    printf("===========================================\n\n");
}

int main() {
    run_test("Empty clause test", { {} }, 20, {});
    run_test("Single literal test", { {1} }, 10, {1});
    run_test("Tautology test", { {1, -1} }, 10, {1});
    run_test("Contradiction test", { {1}, {-1} }, 20, {});
    run_test("Mixed clause test", { {1, 2}, {-1} }, 10, {1, 2});
    run_test("Contradiction 2 test", { {1, 2}, {-1}, {-2} }, 20, {});

    incremental_test();
    assumptions_test();

    return 0;
}

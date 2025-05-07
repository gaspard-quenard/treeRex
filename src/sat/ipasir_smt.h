#ifndef ipasir_h_INCLUDED
#define ipasir_h_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Return the name and the version of the incremental SAT
 * solving library.
 */
const char * ipasir_signature ();

/**
 * Construct a new solver and return a pointer to it.
 * Use the returned pointer as the first parameter in each
 * of the following functions.
 *
 * Required state: N/A
 * State after: INPUT
 */
void * ipasir_init ();

/**
 * Release the solver, i.e., all its resources and
 * allocated memory (destructor). The solver pointer
 * cannot be used for any purposes after this call.
 *
 * Required state: INPUT or SAT or UNSAT
 * State after: undefined
 */
void ipasir_release (void * solver);

/**
 * Add the given literal into the currently added clause
 * or finalize the clause with a 0.  Clauses added this way
 * cannot be removed. The addition of removable clauses
 * can be simulated using activation literals and assumptions.
 *
 * Required state: INPUT or SAT or UNSAT
 * State after: INPUT
 *
 * Literals are encoded as (non-zero) integers as in the
 * DIMACS formats.
 */
void ipasir_add (void * solver, int lit_or_zero);

/**
 * Add an assumption for the next SAT search (the next call
 * of ipasir_solve). After calling ipasir_solve all the
 * previously added assumptions are cleared.
 *
 * Required state: INPUT or SAT or UNSAT
 * State after: INPUT
 */
void ipasir_assume (void * solver, int lit);

/**
 * Solve the formula with specified clauses under the specified assumptions.
 * If the formula is satisfiable the function returns 10 and the state of the solver is changed to SAT.
 * If the formula is unsatisfiable the function returns 20 and the state of the solver is changed to UNSAT.
 * If the search is interrupted (see ipasir_set_terminate) the function returns 0 and the state of the solver remains INPUT.
 * This function can be called in any defined state of the solver.
 *
 * Required state: INPUT or SAT or UNSAT
 * State after: INPUT or SAT or UNSAT
 */
int ipasir_solve (void * solver);

/**
 * Get the truth value of the given literal in the found satisfying
 * assignment. Return 'lit' if True, '-lit' if False, and 0 if not important.
 * This function can only be used if ipasir_solve has returned 10
 * and no 'ipasir_add' nor 'ipasir_assume' has been called since then, i.e., the state of the solver is SAT.
 *
 * Required state: SAT
 * State after: SAT
 */
int ipasir_val (void * solver, int lit);

/**
 * Check if the given assumption literal was used to prove the
 * unsatisfiability of the formula under the assumptions
 * used for the last SAT search. Return 1 if so, 0 otherwise.
 * This function can only be used if ipasir_solve has returned 20 and
 * no ipasir_add or ipasir_assume has been called since then, i.e., the state of the solver is UNSAT.
 *
 * Required state: UNSAT
 * State after: UNSAT
 */
int ipasir_failed (void * solver, int lit);

/**
 * Set a callback function used to indicate a termination requirement to the
 * solver. The solver will periodically call this function and check its return
 * value during the search.
 *
 * Required state: INPUT or SAT or UNSAT
 * State after: INPUT or SAT or UNSAT
 */
void ipasir_set_terminate (void * solver, void * state, int (*terminate)(void * state));

void ipasir_set_learn (void * solver, void * state, int max_length, void (*learn)(void * state, int * clause));

// NON STANDARD FUNCTIONS.
void ipasir_set_seed (void * s, int seed);
void ipasir_set_phase (void * s, unsigned int v, bool phase);
void ipasir_set_decision_var (void * s, unsigned int v, bool decision_var);

// New functions for integer variables and constraints
void ipasir_create_int_var(void* solver, int var_id);
void ipasir_add_int_constraint(void* solver, int var_id1, int op, int var_id2);
void ipasir_add_int_constraint_const(void* solver, int var_id, int op, int const_val);
int ipasir_get_int_value(void* solver, int var_id);
void ipasir_add_int_diff_const(void* solver, int var1, int var2, int op, int const_val); 
void ipasir_add_linear_constraint(void* solver,
    int         nTerms,
    const int*  vars,
    const int*  coeffs,
    int         op,
    int         constant);

#ifdef __cplusplus
}
#endif

#endif

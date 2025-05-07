// ───────────────────────────── ipasir_z3.cpp (patched) ──────────────────────
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>          // ←  added
#include "z3++.h"
#include "ipasir_smt.h"

// ╭──────────────────────────  internal struct  ──────────────────────────────╮
struct Z3Solver {
    z3::context ctx;
    z3::solver  solver;

    std::unordered_map<int,z3::expr> bool_map;
    std::unordered_map<int,z3::expr> int_map;

    std::vector<z3::expr> current_clause;
    std::vector<z3::expr> assumptions;
    std::vector<z3::expr> guard_stack;
    int                   next_guard_id = 1'000'000;

    /* unsat-core bookkeeping */
    bool                           last_unsat = false;
    std::unordered_set<Z3_ast>     core_cache;

    Z3Solver(): ctx(), solver(ctx) {}
};
// ╰────────────────────────────────────────────────────────────────────────────╯

// ── helpers ──────────────────────────────────────────────────────────────────
static z3::expr ensure_bool(Z3Solver* s,int id){
    auto it=s->bool_map.find(id);
    if(it==s->bool_map.end()){
        it=s->bool_map.emplace(id, s->ctx.bool_const(("x"+std::to_string(id)).c_str())).first;
    }
    return it->second;
}
static z3::expr get_lit(Z3Solver* s,int lit){
    z3::expr b = ensure_bool(s,std::abs(lit));
    return (lit<0)? !b : b;
}
static z3::expr ensure_int(Z3Solver* s,int id){
    auto it=s->int_map.find(id);
    if(it==s->int_map.end()){
        it=s->int_map.emplace(id, s->ctx.int_const(("i"+std::to_string(id)).c_str())).first;
    }
    return it->second;
}


// ── C-API ────────────────────────────────────────────────────────────────────
extern "C" {

const char* ipasir_signature(){ return "ipasir_z3 0.2"; }

/* init / release */
void* ipasir_init(){ return new Z3Solver(); }
void  ipasir_release(void* p){ delete static_cast<Z3Solver*>(p); }

/* push / pop with selector guards */
void ipasir_push(void* p){
    auto* s=static_cast<Z3Solver*>(p);
    z3::expr g = ensure_bool(s, s->next_guard_id++);
    s->guard_stack.push_back(g);
    s->assumptions.push_back(g);             // activate
}
void ipasir_pop(void* p){
    auto* s=static_cast<Z3Solver*>(p);
    assert(!s->guard_stack.empty());
    z3::expr g = s->guard_stack.back(); s->guard_stack.pop_back();
    s->assumptions.push_back(!g);            // deactivate
}

/* add literals / clause finish (=0) */
void ipasir_add(void* p,int lit){
    auto* s = static_cast<Z3Solver*>(p);
    if(lit != 0){                       // still building a clause
        s->current_clause.push_back(get_lit(s,lit));
        return;
    }

    /* ---------- lit == 0  →  finish the clause ------------------ */
    if(s->current_clause.empty()){
        // empty clause  ⇒  false  (optionally guarded)
        z3::expr fals = s->ctx.bool_val(false);
        if(!s->guard_stack.empty())
            fals = z3::implies(s->guard_stack.back(), fals);
        s->solver.add(fals);
    }else{
        z3::expr_vector tmp(s->ctx);
        for(auto& e : s->current_clause) tmp.push_back(e);
        z3::expr disj = z3::mk_or(tmp);
        if(!s->guard_stack.empty())
            disj = z3::implies(s->guard_stack.back(), disj);
        s->solver.add(disj);
    }
    s->current_clause.clear();
}

/* assumptions */
void ipasir_assume(void* p,int lit){
    static_cast<Z3Solver*>(p)->assumptions.push_back(get_lit(static_cast<Z3Solver*>(p),lit));
}

/* helper: flush unterminated clause before solve */
static void flush_open_clause(Z3Solver* s){
    if(s->current_clause.empty()) return;
    z3::expr_vector tmp(s->ctx);
    for(auto& e:s->current_clause) tmp.push_back(e);
    z3::expr disj = z3::mk_or(tmp);
    if(!s->guard_stack.empty()) disj=z3::implies(s->guard_stack.back(),disj);
    s->solver.add(disj);
    s->current_clause.clear();
}

/* solve */
int ipasir_solve(void* p){
    auto* s=static_cast<Z3Solver*>(p);
    flush_open_clause(s);

    z3::check_result r = s->solver.check(
            static_cast<unsigned>(s->assumptions.size()), s->assumptions.data());

    s->core_cache.clear();
    if(r==z3::unsat){
        z3::expr_vector core = s->solver.unsat_core();
        for(unsigned i=0;i<core.size();++i) s->core_cache.insert(core[i]);
    }
    s->last_unsat = (r==z3::unsat);
    s->assumptions.clear();

    if(r==z3::sat)   return 10;
    if(r==z3::unsat) return 20;
    return 0;                                 // unknown / interrupted
}

/* model queries */
int ipasir_val(void* p,int lit){
    auto* s=static_cast<Z3Solver*>(p);
    z3::expr v=s->solver.get_model().eval(get_lit(s,lit),true);
    if(z3::eq(v,s->ctx.bool_val(true)))  return std::abs(lit);
    if(z3::eq(v,s->ctx.bool_val(false))) return -std::abs(lit);
    return 0;
}

/* failed assumption */
int ipasir_failed(void* p,int lit){
    auto* s=static_cast<Z3Solver*>(p);
    return (s->last_unsat && s->core_cache.count(get_lit(s,lit))) ? 1 : 0;
}

/* termination callback – stub for now */
void ipasir_set_terminate(void*,void*,int(*)(void*)) { }

/* seed */
void ipasir_set_seed(void* p,int seed){
    auto* s=static_cast<Z3Solver*>(p);
    s->ctx.set(":random-seed", seed);
    s->solver.set("sat.random_seed", static_cast<unsigned>(seed));
}

/* ---------------- integer helpers  ---------------- */
void ipasir_create_int_var(void* p,int id){ ensure_int(static_cast<Z3Solver*>(p),id); }

static void add_int_cmp(z3::solver& slv,z3::expr a,int op,z3::expr b){
    switch(op){
        case 1: slv.add(a >  b); break;
        case 2: slv.add(a >= b); break;
        case 3: slv.add(a <  b); break;
        case 4: slv.add(a <= b); break;
        case 5: slv.add(a == b); break;
        case 6: slv.add(a != b); break;
    }
}
void ipasir_add_int_constraint(void* p,int id1,int op,int id2){
    auto* s=static_cast<Z3Solver*>(p);
    add_int_cmp(s->solver, ensure_int(s,id1), op, ensure_int(s,id2));
}
void ipasir_add_int_constraint_const(void* p,int id,int op,int c){
    auto* s=static_cast<Z3Solver*>(p);
    add_int_cmp(s->solver, ensure_int(s,id), op, s->ctx.int_val(c));
}

int ipasir_get_int_value(void* p,int id){
    auto* s=static_cast<Z3Solver*>(p);
    z3::expr v=s->solver.get_model().eval(ensure_int(s,id),true);
    return v.is_numeral() ? v.get_numeral_int() : 0;
}
/* ----  ---------------------------------------------
   Encode   (var1 - var2)  OP  const
  
----------------------------------------------------------------*/
void ipasir_add_int_diff_const(void* p, int var1, int var2, int op,   int constant){
    auto* s = static_cast<Z3Solver*>(p);
    z3::expr a = ensure_int(s,var1);
    z3::expr b = ensure_int(s,var2);
    z3::expr diff = a - b;                         // (var1 - var2)
    add_int_cmp(s->solver, diff, op, s->ctx.int_val(constant));
}

/* ----  ---------------------------------------------
 example: 2⋅fuel  +1⋅capacity ≤10
you pass n=2, vars=[fuelId,capId], coeffs=[2,1], op=4 (≤), k=10.*/
void ipasir_add_linear_constraint(void* p,
    int n,
    const int* vars,
    const int* coeffs,
    int op,
    int k)
{
auto* s = static_cast<Z3Solver*>(p);
z3::expr lin = s->ctx.int_val(0);          // start with 0
for(int i=0;i<n;++i) {
z3::expr v  = ensure_int(s, vars[i]);
z3::expr ci = s->ctx.int_val(coeffs[i]);
lin = lin + ci * v;                    // accumulate
}
add_int_cmp(s->solver, lin, op, s->ctx.int_val(k));
}


} // extern "C"

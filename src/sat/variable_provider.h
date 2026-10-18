#ifndef VARIABLE_PROVIDER_H
#define VARIABLE_PROVIDER_H

class VariableProvider
{

private:
    static int _running_var_id;

public:
    static int nextVar();
    // Reserve n consecutive variables and return the first one
    static int nextVars(int n);
    static int getMaxVar();
};

// The fact variables of a position: one SAT variable per predicate.
// They are created together with VariableProvider::nextVars, so they are consecutive:
// we only store the first one and the variable of predicate p is first_var + p.
// (Much lighter than storing a std::vector<int> with one entry per predicate in every node.)
struct FactVariables
{
    int first_var = 0;

    int operator[](int pred_idx) const { return first_var + pred_idx; }
};

#endif // VARIABLE_PROVIDER_H

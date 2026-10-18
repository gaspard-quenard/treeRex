#include "sat/variable_provider.h"

int VariableProvider::_running_var_id = 1;

int VariableProvider::nextVar()
{
    return _running_var_id++;
}

int VariableProvider::nextVars(int n)
{
    int first_var = _running_var_id;
    _running_var_id += n;
    return first_var;
}

int VariableProvider::getMaxVar()
{
    return _running_var_id - 1;
}

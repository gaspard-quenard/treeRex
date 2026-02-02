#ifndef VARIABLE_PROVIDER_H
#define VARIABLE_PROVIDER_H

class VariableProvider
{

private:
    static int _running_var_id;

public:
    static int nextVar();
    static int getMaxVar();
    static void reset() { _running_var_id = 0; }
};

#endif // VARIABLE_PROVIDER_H
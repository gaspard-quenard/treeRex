#ifndef PLANNER_H
#define PLANNER_H

#include "data/htn_instance.h"
#include "sat/encoding.h"

class Planner
{

private:
    HtnInstance &_htn;
    Encoding _enc;

public:
    Planner(HtnInstance& htn): _htn(htn), _enc(_htn) {}

    int findPlan();
};


#endif // PLANNER_H
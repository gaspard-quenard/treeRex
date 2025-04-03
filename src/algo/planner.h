#ifndef PLANNER_H
#define PLANNER_H

#include "data/htn_instance.h"
#include "sat/encoding.h"
#include "data/pdt_node.h"
#include "algo/plan_manager.h"


class Planner
{

private:
    HtnInstance &_htn;
    Encoding _enc;
    PdtNode* _root_node;
    PlanManager _plan_manager;

    const bool _write_plan;
    const bool _print_var_names;
    const bool _verify_plan;

    /**
     * Expand the structure with a new layer.
     */
    void expandLayer();

public:
    Planner(HtnInstance& htn): 
    _htn(htn), 
    _enc(_htn), 
    _plan_manager(_htn), 
    _print_var_names(_htn.getParams().isNonzero("pvn")),
    _verify_plan(_htn.getParams().isNonzero("vp")),
    _write_plan(_htn.getParams().isNonzero("wp")) {}
    ~Planner() { delete _root_node; }

    int findPlan();
};


#endif // PLANNER_H
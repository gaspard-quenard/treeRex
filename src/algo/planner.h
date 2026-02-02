#ifndef PLANNER_H
#define PLANNER_H

#include "data/htn_instance.h"
#include "sat/encoding.h"
#include "data/pdt_node.h"
#include "algo/plan_manager.h"
#include "algo/retroactive_pruning.h"
#include "sat/dag_encoding.h"


class Planner
{

private:
    HtnInstance &_htn;
    Encoding _enc;
    DAG_Encoding _dag_enc; // For the LTP version
    PdtNode* _root_node;
    PlanManager _plan_manager;
    RetroactivePruning _retroactive_pruning;
    Statistics& _stats;

    const bool _write_plan;
    const bool _print_var_names;
    const bool _verify_plan;
    const bool _compute_poss_facts_at_each_pos;

    const bool _sibylsat_expansion;

    /**
     * Expand the structure with a new layer.
     */
    void expandLayer();

public:
    Planner(HtnInstance& htn): 
    _htn(htn), 
    _stats(Statistics::getInstance()),
    _enc(_htn), 
    _dag_enc(_htn),
    _root_node(nullptr),
    _plan_manager(_htn), 
    _retroactive_pruning(_htn, _enc),
    _print_var_names(_htn.getParams().isNonzero("pvn")),
    _verify_plan(_htn.getParams().isNonzero("vp")),
    _write_plan(_htn.getParams().isNonzero("wp")),
    _compute_poss_facts_at_each_pos(_htn.getParams().isNonzero("computePossFactsAtEachPos")),
    _sibylsat_expansion(_htn.getParams().isNonzero("sibylsat")) {}


    void debug_write_all_paths_in_file(std::vector<DagNode *> &dag, std::string nameFile);
    ~Planner() { if (_root_node) delete _root_node; }

    int findPlan();
    int findPlanLtp();
};


#endif // PLANNER_H
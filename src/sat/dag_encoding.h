#ifndef DAG_ENCODING_H
#define DAG_ENCODING_H

#include "data/htn_instance.h"
#include "sat/sat_interface.h"
#include "data/dag_node.h"

class DAG_Encoding
{
private:
    HtnInstance &_htn;
    SatInterface _sat;
    Statistics& _stats;

    const bool _encode_prec_and_effs_methods;
    const bool _encode_mutexes;
    const bool _reuse_fact_vars;

    // void encodeInitialState(const std::vector<int> &all_pred_vars, const std::unordered_set<int> &init_state);
    // void encodeGoalState(const std::vector<int> &all_pred_vars, const std::unordered_set<int> &goal_state);
    // void encodeActions(const std::unordered_map<int, int> &map_action_idx_to_var, const std::vector<int> &current_fact_vars, const std::vector<int> &next_fact_vars, std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by);
    // void encodeMethods(const std::unordered_map<int, int> &map_method_idx_to_var, const std::vector<int> &current_fact_vars, const std::vector<int> &next_fact_vars, std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by);
    // void encodePrimitivenessOps(const std::unordered_map<int, int> &map_action_idx_to_var, const std::unordered_map<int, int> &map_method_idx_to_var, const int &prim_var);
    // void encodeFrameAxioms(PdtNode *current, PdtNode *right, const std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, const std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by);
    // void encodeAtMostOneOp(const std::unordered_map<int, int> &map_op_idx_to_var);
    void encodeAtMostOne(const std::vector<int> &vars);
    // void encodeHierarchy(const PdtNode *cur_node, const PdtNode *parentNode);

    // void encodeMutexes(const std::vector<int> &next_fact_vars, std::unordered_set<int> &mutex_groups_to_encode);

public:
    DAG_Encoding(HtnInstance &htn) : _htn(htn),
                                 _stats(Statistics::getInstance()),
                                 _sat(htn.getParams()),
                                 _encode_prec_and_effs_methods(htn.getParams().isNonzero("precsEffs")),
                                 _encode_mutexes(htn.getParams().isNonzero("mutex")),
                                 _reuse_fact_vars(htn.getParams().isNonzero("reuseFactVars")) {}

    // void initalEncode(PdtNode *root);
    void encode(std::vector<DagNode *> &prim_nodes);

    // void writeFormula(std::string filename)
    // {
    //     _sat.print_formula(filename);
    // }

    // // Indicate for each node, which action or method is true
    // void setOpsTrueInTree(PdtNode *node, bool accept_relaxed_solution = false);

    // void addUnitConstraint(int var) { _sat.addClause({var}); }

    // void addAssumptions(const std::vector<int> &assumptions);
    int solve();
    void printPlanFromDagNodes(const std::vector<DagNode *> &dag_nodes);
};

#endif // DAG_ENCODING_H
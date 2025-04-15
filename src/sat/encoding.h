#ifndef ENCODING_H
#define ENCODING_H

#include "data/htn_instance.h"
#include "sat/sat_interface.h"
#include "data/pdt_node.h"

class Encoding
{
private:
    HtnInstance &_htn;
    SatInterface _sat;

    void encodeInitialState(const std::vector<int> &all_pred_vars, const std::unordered_set<int> &init_state);
    void encodeGoalState(const std::vector<int> &all_pred_vars, const std::unordered_set<int> &goal_state);
    void encodeActions(const std::unordered_map<int, int> &map_action_idx_to_var, const std::vector<int> &current_fact_vars, const std::vector<int> &next_fact_vars, std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by);
    void encodeMethods(const std::unordered_map<int, int> &map_method_idx_to_var, const std::vector<int> &current_fact_vars, const std::vector<int> &next_fact_vars);
    void encodePrimitivenessOps(const std::unordered_map<int, int> &map_action_idx_to_var, const std::unordered_map<int, int> &map_method_idx_to_var, const int &prim_var);
    void encodeFrameAxioms(const std::vector<int> &current_fact_vars, const std::vector<int> &next_fact_vars, const int &prim_var, const std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, const std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by);
    void encodeAtMostOneOp(const std::unordered_map<int, int> &map_op_idx_to_var);
    void encodeHierarchy(const PdtNode *cur_node, const PdtNode *parentNode);

public:
    Encoding(HtnInstance &htn) : _htn(htn), _sat(htn.getParams()) {}

    void initalEncode(PdtNode *root);
    void encode(std::vector<PdtNode *> &leaf_nodes);

    void writeFormula(std::string filename)
    {
        _sat.print_formula(filename);
    }

    // Indicate for each node, which action or method is true
    void setOpsTrueInTree(PdtNode *node);

    void addAssumptions(const std::vector<int> &assumptions);
    int solve();
};

#endif // ENCODING_H
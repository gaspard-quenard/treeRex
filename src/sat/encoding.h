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
    Statistics &_stats = Statistics::getInstance();

    const bool _encode_mutexes = _htn.getParams().isNonzero("mutex");
    // Encoding of the at-most-one constraints: pairwise (O(n^2) clauses) or commander (O(n) clauses)
    const bool _amo_pairwise = _htn.getParams().getParam("amo") == "pairwise";

    void encodeInitialState(const FactVariables &all_pred_vars, const std::unordered_set<int> &init_state);
    void encodeGoalState(const FactVariables &all_pred_vars, const std::unordered_set<int> &goal_state);
    void encodeActions(const std::unordered_map<int, int> &map_action_idx_to_var, const FactVariables &current_fact_vars, const FactVariables &next_fact_vars, std::vector<std::vector<int>> &positive_effs_can_be_implied_by, std::vector<std::vector<int>> &negative_effs_can_be_implied_by);
    void encodeMethods(const std::unordered_map<int, int> &map_method_idx_to_var, const FactVariables &current_fact_vars, const FactVariables &next_fact_vars);
    void encodePrimitivenessOps(const std::unordered_map<int, int> &map_action_idx_to_var, const std::unordered_map<int, int> &map_method_idx_to_var, const int &prim_var);
    void encodeFrameAxioms(const FactVariables &current_fact_vars, const FactVariables &next_fact_vars, const int &prim_var, const std::vector<std::vector<int>> &positive_effs_can_be_implied_by, const std::vector<std::vector<int>> &negative_effs_can_be_implied_by);
    void encodeAtMostOneOp(const std::unordered_map<int, int> &map_op_idx_to_var);
    void encodeAtMostOne(const std::vector<int> &vars);
    void encodeAtMostOnePairwise(const std::vector<int> &vars);
    void encodeAtMostOneCommander(const std::vector<int> &vars);
    void encodeHierarchy(const PdtNode *parent_node);

public:
    Encoding(HtnInstance &htn) : _htn(htn), _sat(htn.getParams())
    {
        std::string amo = _htn.getParams().getParam("amo");
        if (amo != "pairwise" && amo != "commander")
        {
            Log::e("Unknown at-most-one encoding '%s' (use pairwise or commander)\n", amo.c_str());
            exit(1);
        }
    }

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
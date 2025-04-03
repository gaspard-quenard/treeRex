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
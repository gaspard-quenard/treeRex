#ifndef PDT_NODE_H
#define PDT_NODE_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <utility>    // For std::pair
#include <functional> // For std::hash

#include "data/htn_instance.h"

enum class OpType
{
    METHOD,
    ACTION
};

// Define a custom hash function for std::pair<int, OpType>
// Needed because std::unordered_set requires a hash function for its elements.
struct PairHash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const
    {
        // Hash the first element (int)
        auto hash1 = std::hash<T1>{}(p.first);
        // Hash the second element (OpType enum). Need to cast enum to underlying type (int)
        auto hash2 = std::hash<std::underlying_type_t<T2>>{}(static_cast<std::underlying_type_t<T2>>(p.second));

        // Combine the two hash values. Using boost::hash_combine logic:
        return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

class PdtNode
{
private:
    int _layer;
    int _pos;
    int _offset;

    std::unordered_set<int> _methods_idx;
    std::unordered_set<int> _actions_idx;
    std::unordered_set<int> _actions_repetition_idx;

    std::unordered_map<int, std::unordered_set<int>> _parents_of_method;                              // Key: method_idx, Value: set of parent_method_idx
    std::unordered_map<int, std::unordered_set<std::pair<int, OpType>, PairHash>> _parents_of_action; // Key: action_idx, Value: set of {parent_idx, parent_type}

    std::unordered_map<int, std::unordered_set<std::pair<int, OpType>, PairHash>> _expansions_of_parent_method; // Key: method_idx of the parent node, Value: set of {parent_idx, parent_type} corresponding to the children actions/methods on this node

    std::unordered_map<int, int> _method_variables;
    std::unordered_map<int, int> _action_variables;
    std::vector<int> _fact_variables; // Indexed by predicate ID
    int _prim_var;

    // TODO IMPLEMENT THAT
    std::unordered_set<int> _fact_true; // Set of predicates that are mandatory true in this node
    std::unordered_set<int> _fact_false; // Set of predicates that are mandatory false in this node

    bool _sat_var_assigned = false; // Indicates if the SAT variables have been assigned
    bool _node_encoded = false; // Indicates if the node has been encoded

    // For the solution, get the type and id of the op solution
    std::pair<int, OpType> _op_solution; // {id, type}

    const PdtNode *_parent;
    std::vector<PdtNode *> _children;

public:
    PdtNode(const PdtNode *parent)
        : _parent(parent)
    {
        if (parent != nullptr)
        {
            _layer = parent->_layer + 1;
            _offset = parent->_children.size();
            _pos = parent->_pos + _offset;
        }
        else
        {
            _layer = 0;
            _pos = 0;
            _offset = 0;
        }
    }
    ~PdtNode()
    {
        for (PdtNode *child : _children)
            delete child;
    }

    void addMethodIdx(int method_idx);
    void addActionIdx(int action_idx);
    void addActionRepetitionIdx(int action_idx);
    void addParentOfMethod(int method_idx, int parent_method_idx);
    void addParentOfAction(int action_idx, int parent_idx, OpType parent_type);
    const std::unordered_set<int> &getMethodsIdx() const;
    const std::unordered_set<int> &getActionsIdx() const;
    const std::unordered_set<int> &getOpsIdx(const OpType& type) const;
    const std::unordered_set<int> &getActionsRepetitionIdx() const;
    std::vector<PdtNode *> &getChildren();
    const PdtNode *getParent() const;
    const std::unordered_map<int, std::unordered_set<int>> &getParentsOfMethodMap() const; // Renamed for clarity
    const std::unordered_map<int, std::unordered_set<std::pair<int, OpType>, PairHash>> &getParentsOfActionMap() const; // Renamed for clarity
    const std::unordered_set<std::pair<int, OpType>, PairHash> getParentsOfOp(int op_idx, OpType op_type) const; // New helper

    const std::vector<int> &getFactVariables() const;
    const std::unordered_map<int, int> &getMethodAndVariables() const;
    const std::unordered_map<int, int> &getActionAndVariables() const;
    const int getPrimVariable() const;
    const std::string getPositionString() const;
    const std::unordered_map<int, std::unordered_set<std::pair<int, OpType>, PairHash>> &getExpansionsOfParentMethodMap() const; // Getter for expansions

    // Getters and Setters for the solution
    void setOpSolution(int id, OpType type);
    const std::pair<int, OpType> &getOpSolution() const;

    void assignSatVariables(const HtnInstance &htn, const bool print_var_names);

    void updateFactVariable(int pred_id, int var_id) { _fact_variables[pred_id] = var_id; }

    size_t computeNumberOfChildren(HtnInstance &htn);
    void expand(HtnInstance &htn);

    const int getNumMethods() const { return _methods_idx.size(); }
    const int getNumActions() const { return _actions_idx.size(); }

    const bool isNodeAlreadyEncoded() const { return _node_encoded; }
    void setNodeEncoded() { _node_encoded = true; }

    std::unordered_set<std::pair<int, OpType>, PairHash> getAllImpossibleOps(HtnInstance &htn, const std::vector<bool> &poss_facts);
    void updatePossFacts(HtnInstance &htn, std::vector<bool> &poss_facts);

    // Pruning related methods
    bool removeOp(int op_idx, OpType op_type); // New helper
};

#endif // PDT_NODE_H

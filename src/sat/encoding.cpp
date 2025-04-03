#include "sat/encoding.h"
#include "util/names.h"

void Encoding::encode(std::vector<PdtNode *> &leaf_nodes)
{
    for (int i = 0; i < leaf_nodes.size(); ++i)
    {
        // Log::i("Encode node %d/%d: %s\n", i, leaf_nodes.size(), TOSTR(*leaf_nodes[i]));
        PdtNode *node = leaf_nodes[i];
        const PdtNode *parent_node = node->getParent();

        const std::vector<int> &current_fact_vars = node->getFactVariables();
        const std::vector<int> &next_fact_vars = i + 1 < leaf_nodes.size() ? leaf_nodes[i + 1]->getFactVariables() : _htn.getFactVarsGoal();

        // TODO ENCODE THE NODE
    }
}

void Encoding::initalEncode(PdtNode *root_node)
{
    // Encoding the initial state
    encodeInitialState(root_node->getFactVariables(), _htn.getInitState());

    // Encoding the goal state
    encodeGoalState(_htn.getFactVarsGoal(), _htn.getGoalState());

    // Root method must be true
    if (root_node->getActionsIdx().size() != 0 || root_node->getMethodsIdx().size() != 1)
    {
        Log::e("Root node must have only one method and no action");
        exit(1);
    }

    int var_root_method = root_node->getMethodAndVariables().begin()->second;
    _sat.addClause(var_root_method);
}

void Encoding::encodeInitialState(const std::vector<int> &all_pred_vars, const std::unordered_set<int> &init_state)
{
    for (int i = 0; i < _htn.getNumPredicates(); i++)
    {
        if (init_state.find(i) != init_state.end())
        {
            _sat.addClause(all_pred_vars[i]);
        }
        else
        {
            _sat.addClause(-all_pred_vars[i]);
        }
    }
}

void Encoding::encodeGoalState(const std::vector<int> &all_pred_vars, const std::unordered_set<int> &goal_state)
{
    for (int i = 0; i < _htn.getNumPredicates(); i++)
    {
        if (goal_state.find(i) != goal_state.end())
        {
            _sat.addClause(all_pred_vars[i]);
        }
    }
}

void Encoding::addAssumptions(const std::vector<int> &assumptions)
{
    for (int assumption : assumptions)
    {
        _sat.assume(assumption);
    }
}

int Encoding::solve()
{
    return _sat.solve();
}

void Encoding::setOpsTrueInTree(PdtNode *node)
{
    // Log::i("For node %s\n", TOSTR(*node));
    // Get the op true in the node
    int num_ops_true = 0;
    for (const auto &[op_idx, op_var] : node->getMethodAndVariables())
    {
        if (_sat.holds(op_var))
        {
            // If it is a leaf node, return an error
            if (node->getChildren().size() == 0)
            {
                throw std::runtime_error("An op true in the leaf node cannot be a method");
            }
            node->setOpSolution(op_idx, OpType::METHOD);
            // Log::i("  Method %s is true\n", TOSTR(_htn.getMethodById(op_idx)));
            num_ops_true++;
        }
    }
    for (const auto &[op_idx, op_var] : node->getActionAndVariables())
    {
        if (_sat.holds(op_var))
        {
            node->setOpSolution(op_idx, OpType::ACTION);
            // Log::i("  Action %s is true\n", TOSTR(_htn.getActionById(op_idx)));
            num_ops_true++;
        }
    }
    if (num_ops_true != 1)
    {
        throw std::runtime_error("There must be one and only one op true in the node");
    }
    // Recursively extract the op true for each child
    for (PdtNode *child : node->getChildren())
    {
        setOpsTrueInTree(child);
    }
}
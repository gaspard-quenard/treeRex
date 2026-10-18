#include "sat/encoding.h"
#include "util/names.h"

void Encoding::encode(std::vector<PdtNode *> &leaf_nodes)
{
    for (int i = 0; i < leaf_nodes.size(); ++i)
    {
        // Log::i("Encode node %d/%d: %s\n", i, leaf_nodes.size(), TOSTR(*leaf_nodes[i]));
        PdtNode *node = leaf_nodes[i];
        const PdtNode *parent_node = node->getParent();

        const FactVariables &current_fact_vars = node->getFactVariables();
        const FactVariables &next_fact_vars = i + 1 < leaf_nodes.size() ? leaf_nodes[i + 1]->getFactVariables() : _htn.getFactVarsGoal();

        std::unordered_map<int, std::unordered_set<int>> positive_effs_can_be_implied_by;
        std::unordered_map<int, std::unordered_set<int>> negative_effs_can_be_implied_by;

        // Action implies precondition and effects, get as well all the predicates that can be changed and the actions that can change them
        _stats.begin(STAGE_ACTIONS);
        encodeActions(node->getActionAndVariables(), current_fact_vars, next_fact_vars, positive_effs_can_be_implied_by, negative_effs_can_be_implied_by);
        _stats.end(STAGE_ACTIONS);

        // Method implies precondition (Here method precondition is compiled into an action which is the first subtask of the method)
        // encodeMethods(node->getMethodAndVariables(), current_fact_vars, next_fact_vars);

        // action implies prim, method implies not prim
        _stats.begin(STAGE_PRIMITIVENESS);
        encodePrimitivenessOps(node->getActionAndVariables(), node->getMethodAndVariables(), node->getPrimVariable());
        _stats.end(STAGE_PRIMITIVENESS);

        // Encode frame axioms
        _stats.begin(STAGE_FRAME_AXIOMS);
        encodeFrameAxioms(current_fact_vars, next_fact_vars, node->getPrimVariable(), positive_effs_can_be_implied_by, negative_effs_can_be_implied_by);
        _stats.end(STAGE_FRAME_AXIOMS);

        // At most one action or method is selected
        _stats.begin(STAGE_AT_MOST_ONE_ACTION);
        encodeAtMostOneOp(node->getActionAndVariables());
        _stats.end(STAGE_AT_MOST_ONE_ACTION);
        _stats.begin(STAGE_AT_MOST_ONE_METHOD);
        encodeAtMostOneOp(node->getMethodAndVariables());
        _stats.end(STAGE_AT_MOST_ONE_METHOD);

        // Encode the hierarchy of each ops
        _stats.begin(STAGE_HIERARCHY);
        encodeHierarchy(node, parent_node);
        _stats.end(STAGE_HIERARCHY);
    }

    if (_encode_mutexes)
    {
        Log::i("  Encoding Mutexes\n");
        _stats.begin(STAGE_MUTEXES);
        // Encode mutexes for all time steps
        for (int i = 0; i < leaf_nodes.size(); ++i)
        {
            const FactVariables &current_fact_vars = leaf_nodes[i]->getFactVariables();
            // Encode all groups
            for (const std::vector<int> &group : _htn.getMutex().getMutexGroups())
            {

                std::vector<int> group_vars;
                for (int pred_idx : group)
                {
                    // Get the variable for this predicate at this time step
                    int var = current_fact_vars[pred_idx];
                    group_vars.push_back(var);
                }
                // Encode the at-most-one constraint for this group
                encodeAtMostOne(group_vars);
            }
        }
        _stats.end(STAGE_MUTEXES);
    }
}

void Encoding::initalEncode(PdtNode *root_node)
{
    _stats.begin(STAGE_INIT_GOAL_ROOT);
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
    _stats.end(STAGE_INIT_GOAL_ROOT);
}

void Encoding::encodeInitialState(const FactVariables &all_pred_vars, const std::unordered_set<int> &init_state)
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

void Encoding::encodeGoalState(const FactVariables &all_pred_vars, const std::unordered_set<int> &goal_state)
{
    for (int i = 0; i < _htn.getNumPredicates(); i++)
    {
        if (goal_state.find(i) != goal_state.end())
        {
            _sat.addClause(all_pred_vars[i]);
        }
    }
}

void Encoding::encodeActions(const std::unordered_map<int, int> &map_action_idx_to_var, const FactVariables &current_fact_vars, const FactVariables &next_fact_vars, std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by)
{
    for (const auto &[action_idx, action_var] : map_action_idx_to_var)
    {
        const Action &action = _htn.getActionById(action_idx);

        // Action implies preconditions
        for (int precondition_idx : action.getPreconditionsIdx())
        {
            _sat.addClause(-action_var, current_fact_vars[precondition_idx]);
        }

        // Action implies effects
        for (int pos_effect_idx : action.getPosEffsIdx())
        {
            _sat.addClause(-action_var, next_fact_vars[pos_effect_idx]);
            positive_effs_can_be_implied_by[pos_effect_idx].insert(action_var);
        }

        for (int neg_effect_idx : action.getNegEffsIdx())
        {
            _sat.addClause(-action_var, -next_fact_vars[neg_effect_idx]);
            negative_effs_can_be_implied_by[neg_effect_idx].insert(action_var);
        }
    }
}

// void Encoding::encodeMethods(const std::unordered_map<int, int> &map_method_idx_to_var, const FactVariables &current_fact_vars, const FactVariables &next_fact_vars) {
//     for (const auto &[method_idx, method_var] : map_method_idx_to_var)
//     {
//         const Method &method = _htn.getMethodById(method_idx);
//         const std::vector<int> &preconditions_idx = method.();

//         // Method implies preconditions
//         for (int precondition_idx : preconditions_idx)
//         {
//             _sat.addClause(-method_var, current_fact_vars[precondition_idx]);
//         }
//     }
// }

void Encoding::encodePrimitivenessOps(const std::unordered_map<int, int> &map_action_idx_to_var, const std::unordered_map<int, int> &map_method_idx_to_var, const int &prim_var)
{
    for (const auto &[action_idx, action_var] : map_action_idx_to_var)
    {
        _sat.addClause(-action_var, prim_var);
    }
    for (const auto &[method_idx, method_var] : map_method_idx_to_var)
    {
        _sat.addClause(-method_var, -prim_var);
    }
}

void Encoding::encodeFrameAxioms(const FactVariables &current_fact_vars, const FactVariables &next_fact_vars, const int &prim_var, const std::unordered_map<int, std::unordered_set<int>> &positive_effs_can_be_implied_by, const std::unordered_map<int, std::unordered_set<int>> &negative_effs_can_be_implied_by)
{
    for (int i = 0; i < _htn.getNumPredicates(); i++)
    {
        // If this predicate was true and become false, then either there is a method responsable for this change (so the position is non primtiive)
        // Or an action must be responsible for this change:
        // pred__t and not pred__t+1 => (non prim or negative effect of an action)
        // In CNF:
        // (not pred__t or pred__t+1 or non_prim or action_1_with_negative_effect or action_2_with_negative_effect)
        _sat.appendClause(-current_fact_vars[i], next_fact_vars[i]);
        _sat.appendClause(-prim_var);
        if (negative_effs_can_be_implied_by.find(i) != negative_effs_can_be_implied_by.end())
        {
            for (int action_var : negative_effs_can_be_implied_by.at(i))
            {
                _sat.appendClause(action_var);
            }
        }
        _sat.endClause();
        // Do the same things if a predicate was false and become true
        _sat.appendClause(current_fact_vars[i], -next_fact_vars[i]);
        _sat.appendClause(-prim_var);
        if (positive_effs_can_be_implied_by.find(i) != positive_effs_can_be_implied_by.end())
        {
            for (int action_var : positive_effs_can_be_implied_by.at(i))
            {
                _sat.appendClause(action_var);
            }
        }
        _sat.endClause();
    }
}

void Encoding::encodeAtMostOne(const std::vector<int> &vars)
{
    // Encode the at most one with the basic encoding
    for (int i = 0; i < vars.size(); i++)
    {
        for (int j = i + 1; j < vars.size(); j++)
        {
            _sat.addClause(-vars[i], -vars[j]);
        }
    }
}

void Encoding::encodeAtMostOneOp(const std::unordered_map<int, int> &map_op_idx_to_var)
{
    std::vector<int> op_vars;
    for (const auto &[op_idx, op_var] : map_op_idx_to_var)
    {
        op_vars.push_back(op_var);
    }

    // Encode the at most one with the basic encoding
    encodeAtMostOne(op_vars);
}

void Encoding::encodeHierarchy(const PdtNode *cur_node, const PdtNode *parent_node)
{
    // For each parent op, get the possible children
    std::unordered_map<int, std::unordered_set<int>> possible_children_var_per_op_var;

    for (const auto &[child_method, possible_parent] : cur_node->getParentsOfMethod())
    {
        int var_child = cur_node->getMethodAndVariables().at(child_method);
        for (int parent_method : possible_parent)
        {
            int var_parent = parent_node->getMethodAndVariables().at(parent_method);
            possible_children_var_per_op_var[var_parent].insert(var_child);
        }
    }
    // Iterate through the actions in the current node and their potential parents
    for (const auto &[child_action_idx, possible_parents_set] : cur_node->getParentsOfAction())
    {
        int var_child = cur_node->getActionAndVariables().at(child_action_idx);
        // Iterate through the set of parent pairs (ID, Type) for this child action
        for (const auto &parent_pair : possible_parents_set)
        {
            int parent_idx = parent_pair.first;
            OpType parent_type = parent_pair.second;

            int var_parent;
            // Get the parent variable based on its type
            if (parent_type == OpType::ACTION)
            {
                var_parent = parent_node->getActionAndVariables().at(parent_idx);
            }
            else // OpType::METHOD
            {
                var_parent = parent_node->getMethodAndVariables().at(parent_idx);
            }
            // Add the child variable to the set associated with the parent variable
            possible_children_var_per_op_var[var_parent].insert(var_child);
        }
    }

    // Each parent implies at least one child
    for (const auto &[parent_var, children_var] : possible_children_var_per_op_var)
    {
        _sat.appendClause(-parent_var);
        for (int child_var : children_var)
        {
            _sat.appendClause(child_var);
        }
        _sat.endClause();
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
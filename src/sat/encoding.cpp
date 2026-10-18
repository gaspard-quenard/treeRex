#include "sat/encoding.h"
#include "util/names.h"

void Encoding::encode(std::vector<PdtNode *> &leaf_nodes)
{
    for (int i = 0; i < leaf_nodes.size(); ++i)
    {
        // Log::i("Encode node %d/%d: %s\n", i, leaf_nodes.size(), TOSTR(*leaf_nodes[i]));
        PdtNode *node = leaf_nodes[i];

        const FactVariables &current_fact_vars = node->getFactVariables();
        const FactVariables &next_fact_vars = i + 1 < leaf_nodes.size() ? leaf_nodes[i + 1]->getFactVariables() : _htn.getFactVarsGoal();

        // For each predicate (index), the variables of the actions of this node which can add it (resp. delete it)
        std::vector<std::vector<int>> positive_effs_can_be_implied_by(_htn.getNumPredicates());
        std::vector<std::vector<int>> negative_effs_can_be_implied_by(_htn.getNumPredicates());

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

        // Encode the hierarchy between the parent of this node and all its children
        // (done once per parent, when we meet its first child)
        if (node->isFirstChild())
        {
            _stats.begin(STAGE_HIERARCHY);
            encodeHierarchy(node->getParent());
            _stats.end(STAGE_HIERARCHY);
        }
    }

    if (_encode_mutexes)
    {
        Log::i("  Encoding Mutexes\n");
        _stats.begin(STAGE_MUTEXES);
        // Encode mutexes for all time steps
        for (int i = 0; i < leaf_nodes.size(); ++i)
        {
            // A first child has the same fact variables as its parent: the mutexes on these variables
            // are already encoded (or, for the root, implied by the initial state). No need to add them again.
            if (leaf_nodes[i]->isFirstChild())
                continue;

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

void Encoding::encodeActions(const std::unordered_map<int, int> &map_action_idx_to_var, const FactVariables &current_fact_vars, const FactVariables &next_fact_vars, std::vector<std::vector<int>> &positive_effs_can_be_implied_by, std::vector<std::vector<int>> &negative_effs_can_be_implied_by)
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
            positive_effs_can_be_implied_by[pos_effect_idx].push_back(action_var);
        }

        for (int neg_effect_idx : action.getNegEffsIdx())
        {
            _sat.addClause(-action_var, -next_fact_vars[neg_effect_idx]);
            negative_effs_can_be_implied_by[neg_effect_idx].push_back(action_var);
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

void Encoding::encodeFrameAxioms(const FactVariables &current_fact_vars, const FactVariables &next_fact_vars, const int &prim_var, const std::vector<std::vector<int>> &positive_effs_can_be_implied_by, const std::vector<std::vector<int>> &negative_effs_can_be_implied_by)
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
        for (int action_var : negative_effs_can_be_implied_by[i])
        {
            _sat.appendClause(action_var);
        }
        _sat.endClause();
        // Do the same things if a predicate was false and become true
        _sat.appendClause(current_fact_vars[i], -next_fact_vars[i]);
        _sat.appendClause(-prim_var);
        for (int action_var : positive_effs_can_be_implied_by[i])
        {
            _sat.appendClause(action_var);
        }
        _sat.endClause();
    }
}

void Encoding::encodeAtMostOne(const std::vector<int> &vars)
{
    if (_amo_pairwise)
        encodeAtMostOnePairwise(vars);
    else
        encodeAtMostOneCommander(vars);
}

void Encoding::encodeAtMostOnePairwise(const std::vector<int> &vars)
{
    // Basic encoding: one binary clause (not x_i or not x_j) for each pair of variables -> n*(n-1)/2 clauses
    for (int i = 0; i < vars.size(); i++)
    {
        for (int j = i + 1; j < vars.size(); j++)
        {
            _sat.addClause(-vars[i], -vars[j]);
        }
    }
}

void Encoding::encodeAtMostOneCommander(const std::vector<int> &vars)
{
    // Commander encoding (Klieber & Kwon, 2007): about 3n clauses and n/2 new variables instead of n*(n-1)/2 clauses,
    // and unit propagation still sets all the other variables to false as soon as one variable is true.
    // The variables are split into groups of 3. In each group, at most one variable is true (pairwise encoding)
    // and each variable implies the "commander" variable of its group. Then, recursively, at most one commander is true.

    // For small sets, the pairwise encoding is as compact and needs no new variable
    if (vars.size() <= 6)
    {
        encodeAtMostOnePairwise(vars);
        return;
    }

    std::vector<int> commanders;
    for (size_t start = 0; start < vars.size(); start += 3)
    {
        std::vector<int> group(vars.begin() + start, vars.begin() + std::min(start + 3, vars.size()));
        if (group.size() == 1)
        {
            // A group with a single variable does not need a commander: the variable is its own commander
            commanders.push_back(group[0]);
            continue;
        }
        encodeAtMostOnePairwise(group);
        int commander = VariableProvider::nextVar();
        for (int var : group)
        {
            _sat.addClause(-var, commander);
        }
        commanders.push_back(commander);
    }
    encodeAtMostOneCommander(commanders);
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

void Encoding::encodeHierarchy(const PdtNode *parent_node)
{
    // Each op of the parent implies, in each child, at least one of the ops it can be decomposed into.
    // These are exactly the ops added to the children by PdtNode::expand.
    const std::vector<PdtNode *> &children = parent_node->getChildren();
    int blank_action_idx = _htn.getBlankAction().getId();

    // An action is repeated in the first child, and is followed by blank actions in the other children
    for (const auto &[action_idx, action_var] : parent_node->getActionAndVariables())
    {
        _sat.addClause(-action_var, children[0]->getActionAndVariables().at(action_idx));
        for (size_t j = 1; j < children.size(); ++j)
        {
            _sat.addClause(-action_var, children[j]->getActionAndVariables().at(blank_action_idx));
        }
    }

    // A method implies its j-th subtask in the j-th child (if the subtask is abstract: one of its methods),
    // and blank actions in the children after its last subtask
    for (const auto &[method_idx, method_var] : parent_node->getMethodAndVariables())
    {
        const std::vector<int> &subtasks_idx = _htn.getMethodById(method_idx).getSubtasksIdx();
        for (size_t j = 0; j < children.size(); ++j)
        {
            const PdtNode *child = children[j];
            _sat.appendClause(-method_var);
            if (j >= subtasks_idx.size())
            {
                _sat.appendClause(child->getActionAndVariables().at(blank_action_idx));
            }
            else if (_htn.isAbstractTask(subtasks_idx[j]))
            {
                for (int sub_method_idx : _htn.getAbstractTaskById(subtasks_idx[j]).getDecompositionMethodsIdx())
                {
                    _sat.appendClause(child->getMethodAndVariables().at(sub_method_idx));
                }
            }
            else
            {
                _sat.appendClause(child->getActionAndVariables().at(subtasks_idx[j]));
            }
            _sat.endClause();
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
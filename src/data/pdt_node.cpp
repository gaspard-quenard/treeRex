#include "data/pdt_node.h"
#include "util/names.h"
#include "sat/variable_provider.h"
#include "util/log.h"

void PdtNode::addMethodIdx(int method_idx)
{
    _methods_idx.insert(method_idx);
}

void PdtNode::addParentOfMethod(int method_idx, int parent_method_idx)
{
    _parents_of_method[method_idx].insert(parent_method_idx);
}

void PdtNode::addParentOfAction(int action_idx, int parent_idx, OpType parent_type)
{
    _parents_of_action[action_idx].insert({parent_idx, parent_type});
}

void PdtNode::addActionIdx(int action_idx)
{
    _actions_idx.insert(action_idx);
}

void PdtNode::addActionRepetitionIdx(int action_idx)
{
    _actions_repetition_idx.insert(action_idx);
}

// Renamed function
const std::unordered_map<int, std::unordered_set<int>> &PdtNode::getParentsOfMethodMap() const
{
    return _parents_of_method;
}

// Renamed function
const std::unordered_map<int, std::unordered_set<std::pair<int, OpType>, PairHash>> &PdtNode::getParentsOfActionMap() const
{
    return _parents_of_action;
}

// New helper function implementation
const std::unordered_set<std::pair<int, OpType>, PairHash> PdtNode::getParentsOfOp(int op_idx, OpType op_type) const
{
    std::unordered_set<std::pair<int, OpType>, PairHash> parents;
    if (op_type == OpType::METHOD)
    {
        auto it = _parents_of_method.find(op_idx);
        if (it != _parents_of_method.end())
        {
            for (int parent_method_idx : it->second)
            {
                parents.insert({parent_method_idx, OpType::METHOD});
            }
        }
    }
    else if (op_type == OpType::ACTION)
    {
        auto it = _parents_of_action.find(op_idx);
        if (it != _parents_of_action.end())
        {
            parents = it->second; // Direct copy
        }
    }
    return parents;
}

const std::unordered_set<int> &PdtNode::getMethodsIdx() const
{
    return _methods_idx;
}

const std::unordered_set<int> &PdtNode::getActionsIdx() const
{
    return _actions_idx;
}

const std::unordered_set<int> &PdtNode::getOpsIdx(const OpType &type) const
{
    if (type == OpType::METHOD)
    {
        return _methods_idx;
    }
    else if (type == OpType::ACTION)
    {
        return _actions_idx;
    }
    else
    {
        throw std::invalid_argument("Invalid operation type");
    }
}

const std::unordered_set<int> &PdtNode::getActionsRepetitionIdx() const
{
    return _actions_repetition_idx;
}

std::vector<PdtNode *> &PdtNode::getChildren()
{
    return _children;
}

const PdtNode *PdtNode::getParent() const
{
    return _parent;
}

const std::vector<int> &PdtNode::getFactVariables() const
{
    return _fact_variables;
}

const std::unordered_map<int, int> &PdtNode::getMethodAndVariables() const
{
    return _method_variables;
}

const std::unordered_map<int, int> &PdtNode::getActionAndVariables() const
{
    return _action_variables;
}

const int PdtNode::getPrimVariable() const
{
    return _prim_var;
}

const std::pair<int, OpType> &PdtNode::getOpSolution() const
{
    return _op_solution;
}

// Getter for expansions map
const std::unordered_map<int, std::unordered_set<std::pair<int, OpType>, PairHash>> &PdtNode::getExpansionsOfParentMethodMap() const
{
    return _expansions_of_parent_method;
}

void PdtNode::setOpSolution(int id, OpType type)
{
    _op_solution = {id, type};
}

const std::string PdtNode::getPositionString() const
{
    return std::to_string(_layer) + "_" + std::to_string(_pos);
}

void PdtNode::assignSatVariables(const HtnInstance &htn, const bool print_var_names)
{
    if (_sat_var_assigned)
    {
        // Log::w("SAT variables already assigned for node %s\n", getPositionString().c_str());
        return;
    }
    const int num_predicates = htn.getNumPredicates();

    // Assign a variable to each method
    for (int method_idx : _methods_idx)
    {
        _method_variables[method_idx] = VariableProvider::nextVar();
        if (print_var_names)
        {
            std::string method_name = htn.getMethodById(method_idx).getName() + "__" + getPositionString();
            Log::i("PVN: %d %s\n", _method_variables[method_idx], method_name.c_str());
        }
    }

    // Assign a variable to each action
    for (int action_idx : _actions_idx)
    {
        // If this action has only one parent and it is the same action, use the same variable
        if (_offset == 0 && _parents_of_action[action_idx].size() == 1 && _parents_of_action[action_idx].begin()->second == OpType::ACTION)
        {
            // Log::i("Reusing action variable %d for action %s\n", _parent->getActionAndVariables().at(action_idx), TOSTR(htn.getActionById(action_idx)));
            _action_variables[action_idx] = _parent->getActionAndVariables().at(action_idx);
        }
        else
        {

            _action_variables[action_idx] = VariableProvider::nextVar();
            if (print_var_names)
            {
                std::string action_name = htn.getActionById(action_idx).getName() + "__" + getPositionString();
                Log::i("PVN: %d %s\n", _action_variables[action_idx], action_name.c_str());
            }
        }
    }

    // For each action repetition, get the same variable as the parent action
    // for (int action_idx : _actions_repetition_idx)
    // {
    //     _action_variables[action_idx] = _parent->_action_variables[action_idx];
    // }

    // Assign a variable to each predicate
    // If we are the first child of the parent, we have the same fact variables as the parent
    // bool first_child = _parent != nullptr && _parent->_children[0] == this;
    bool first_child = _parent != nullptr && _offset == 0;
    if (first_child)
    {
        _fact_variables = _parent->_fact_variables;
    }
    else
    {
        for (int pred_id = 0; pred_id < num_predicates; ++pred_id)
        {
            _fact_variables.push_back(VariableProvider::nextVar());
            if (print_var_names)
            {
                std::string pred_name = htn.getPredicateById(pred_id).getName() + "__" + getPositionString();
                Log::i("PVN: %d %s\n", _fact_variables[pred_id], pred_name.c_str());
            }
        }
    }

    // Assign a variable to the prim
    _prim_var = VariableProvider::nextVar();
    if (print_var_names)
    {
        std::string prim_name = "prim__" + getPositionString();
        Log::i("PVN: %d %s\n", _prim_var, prim_name.c_str());
    }

    // Set the flag to indicate that the SAT variables have been assigned
    _sat_var_assigned = true;
}

size_t PdtNode::computeNumberOfChildren(HtnInstance &htn)
{
    size_t num_children = 1;
    for (int method_idx : _methods_idx)
    {
        num_children = std::max(num_children, htn.getMethodById(method_idx).getSubtasksIdx().size());
    }
    return num_children;
}

void PdtNode::expand(HtnInstance &htn)
{

    // Get the number of children for this node
    size_t num_children = computeNumberOfChildren(htn);

    // Create the children
    for (size_t i = 0; i < num_children; ++i)
    {
        PdtNode *child = new PdtNode(this);
        _children.push_back(child);
    }

    int id_blank_action = htn.getBlankAction().getId();

    // For each action, repeat it for the first children
    for (int action_idx : _actions_idx)
    {
        PdtNode *child = _children[0];
        // Log::i("Repeating action %s\n", TOSTR(htn.getActionById(action_idx)));
        // child->addActionRepetitionIdx(action_idx);
        child->addActionIdx(action_idx);
        child->addParentOfAction(action_idx, action_idx, OpType::ACTION);

        // The action add some blank actions in the remaining children
        for (size_t i = 1; i < num_children; ++i)
        {
            PdtNode *child = _children[i];
            child->addActionIdx(id_blank_action);
            child->addParentOfAction(id_blank_action, action_idx, OpType::ACTION);
        }
    }

    // For each method, if the j-th subtask is an action, add it to the j-th children
    // if the j-th subtask is an abstract task, add all the methods of the abstract task to the j-th children
    for (int method_idx : _methods_idx)
    {
        const Method &method = htn.getMethodById(method_idx);
        Log::d("Children of method %s\n", TOSTR(method));
        for (size_t j = 0; j < method.getSubtasksIdx().size(); ++j)
        {
            PdtNode *jth_child = _children[j];
            int subtask_idx = method.getSubtasksIdx()[j];
            if (htn.isAbstractTask(subtask_idx))
            {
                Log::d("Subtask is the abstract task %s\n", TOSTR(htn.getAbstractTaskById(subtask_idx)));
                const AbstractTask &task = htn.getAbstractTaskById(subtask_idx);
                // Add all the methods of the abstract task to the j-th children
                for (int sub_method_idx : task.getDecompositionMethodIdx())
                {
                    jth_child->addMethodIdx(sub_method_idx);
                    jth_child->addParentOfMethod(sub_method_idx, method_idx);
                    jth_child->_expansions_of_parent_method[method_idx].insert({sub_method_idx, OpType::METHOD});
                    Log::d("At subtask %d, adding method %s\n", j, TOSTR(htn.getMethodById(sub_method_idx)));
                }
            }
            else
            {
                // Add the action to the j-th child
                jth_child->addActionIdx(subtask_idx);
                jth_child->addParentOfAction(subtask_idx, method_idx, OpType::METHOD);
                jth_child->_expansions_of_parent_method[method_idx].insert({subtask_idx, OpType::ACTION});
                Log::d("At subtask %d, adding action %s\n", j, TOSTR(htn.getActionById(subtask_idx)));
            }
        }
        // The method add blank actions in the remaining children
        for (size_t j = method.getSubtasksIdx().size(); j < num_children; ++j)
        {
            PdtNode *child = _children[j];
            child->addActionIdx(id_blank_action);
            child->addParentOfAction(id_blank_action, method_idx, OpType::METHOD);
            child->_expansions_of_parent_method[method_idx].insert({id_blank_action, OpType::ACTION});
        }
    }
}

// New helper function implementation
bool PdtNode::removeOp(int op_idx, OpType op_type)
{
    bool removed = false;
    if (op_type == OpType::METHOD)
    {
        // 1. Check existence and remove from main set
        if (_methods_idx.erase(op_idx) > 0)
        {
            removed = true;

            // 2. Get parents before removing parent data
            std::unordered_set<int> parent_method_indices;
            auto parents_it = _parents_of_method.find(op_idx);
            if (parents_it != _parents_of_method.end())
            {
                parent_method_indices = parents_it->second;
                _parents_of_method.erase(parents_it); // Remove entry for this op
            }

            // 3. Remove variable and expansion data (if it was a parent)
            _method_variables.erase(op_idx);
            _expansions_of_parent_method.erase(op_idx);

            // 4. Remove from parent's expansion lists
            for (int parent_method_idx : parent_method_indices)
            {
                auto expansion_it = _expansions_of_parent_method.find(parent_method_idx);
                if (expansion_it != _expansions_of_parent_method.end())
                {
                    expansion_it->second.erase({op_idx, OpType::METHOD});
                }
            }
        }
    }
    else if (op_type == OpType::ACTION)
    {
        // 1. Check existence and remove from main set
        if (_actions_idx.erase(op_idx) > 0)
        {
            removed = true;

            // Also remove from repetitions if present
            _actions_repetition_idx.erase(op_idx);

            // 2. Get parents before removing parent data
            std::unordered_set<std::pair<int, OpType>, PairHash> parents;
            auto parents_it = _parents_of_action.find(op_idx);
            if (parents_it != _parents_of_action.end())
            {
                parents = parents_it->second;
                _parents_of_action.erase(parents_it); // Remove entry for this op
            }

            // 3. Remove variable
            _action_variables.erase(op_idx);

            // 4. Remove from parent method's expansion lists
            for (const auto &parent_pair : parents)
            {
                if (parent_pair.second == OpType::METHOD)
                {
                    int parent_method_idx = parent_pair.first;
                    auto expansion_it = _expansions_of_parent_method.find(parent_method_idx);
                    if (expansion_it != _expansions_of_parent_method.end())
                    {
                        expansion_it->second.erase({op_idx, OpType::ACTION});
                    }
                }
                // Note: Actions cannot be parents in the _expansions_of_parent_method map
            }
        }
    }

    return removed;
}

bool isActionExecutable(const HtnInstance &htn, int action_idx, const std::vector<bool> &poss_facts)
{
    const Action &action = htn.getActionById(action_idx);
    for (int precondition_idx : action.getPreconditionsIdx())
    {
        if (!poss_facts[precondition_idx])
        {
            return false;
        }
    }
    return true;
}
bool isMethodExecutable(const HtnInstance &htn, int method_idx, const std::vector<bool> &poss_facts)
{
    const Method &method = htn.getMethodById(method_idx);
    for (int precondition_idx : method.getPreconditionsIdx())
    {
        if (!poss_facts[precondition_idx])
        {
            return false;
        }
    }
    return true;
}

std::unordered_set<std::pair<int, OpType>, PairHash> PdtNode::getAllImpossibleOps(HtnInstance &htn, const std::vector<bool> &poss_facts)
{
    std::unordered_set<std::pair<int, OpType>, PairHash> impossible_ops;

    // Remove impossible actions
    // for (auto it = _actions_idx.begin(); it != _actions_idx.end();)
    // {
    //     const Action &action = htn.getActionById(*it);
    //     if (!isActionExecutable(htn, *it, poss_facts))
    //     {
    //         Log::i("Removing impossible action %s\n", TOSTR(action));
    //         // Remove the action from the parents
    //         _parents_of_action.erase(*it);
    //         it = _actions_idx.erase(it);
    //     }
    //     else
    //     {
    //         ++it;
    //     }
    // }

    // Remove impossible methods
    // for (auto it = _methods_idx.begin(); it != _methods_idx.end();)
    // {
    //     const Method &method = htn.getMethodById(*it);
    //     if (!isMethodExecutable(htn, *it, poss_facts))
    //     {
    //         // Log::i("Removing impossible method %s\n", TOSTR(method));
    //         // Remove the method from the parents
    //         _parents_of_method.erase(*it);
    //         it = _methods_idx.erase(it);
    //     }
    //     else
    //     {
    //         ++it;
    //     }
    // }

    // Find all impossible actions
    for (int action_idx : _actions_idx)
    {
        const Action &action = htn.getActionById(action_idx);
        if (!isActionExecutable(htn, action_idx, poss_facts))
        {
            impossible_ops.insert({action_idx, OpType::ACTION});
        }
    }

    // Find all impossible methods
    for (int method_idx : _methods_idx)
    {
        const Method &method = htn.getMethodById(method_idx);
        if (!isMethodExecutable(htn, method_idx, poss_facts))
        {
            impossible_ops.insert({method_idx, OpType::METHOD});
        }
    }

    return impossible_ops;
}

void PdtNode::updatePossFacts(HtnInstance &htn, std::vector<bool> &poss_facts)
{
    // Update the possible facts with the new actions/methods
    for (int action_idx : _actions_idx)
    {
        const Action &action = htn.getActionById(action_idx);
        // for (int effect_idx : action.getNegEffsIdx())
        // {
        //     poss_facts[effect_idx] = false;
        // }
        for (int effect_idx : action.getPosEffsIdx())
        {
            poss_facts[effect_idx] = true;
        }
    }

    for (int method_idx : _methods_idx)
    {
        const Method &method = htn.getMethodById(method_idx);
        for (int effect_idx : method.getPossPosEffsIdx())
        {
            poss_facts[effect_idx] = true;
        }
    }
}

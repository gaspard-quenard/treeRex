#include "data/pdt_node.h"
#include "util/names.h"
#include "sat/variable_provider.h"
#include "util/log.h"

void PdtNode::addMethodIdx(int method_idx)
{
    _methods_idx.insert(method_idx);
}

void PdtNode::addActionIdx(int action_idx)
{
    _actions_idx.insert(action_idx);
}

void PdtNode::addActionRepetitionIdx(int action_idx)
{
    _actions_repetition_idx.insert(action_idx);
}

const std::unordered_set<int> &PdtNode::getMethodsIdx() const
{
    return _methods_idx;
}

const std::unordered_set<int> &PdtNode::getActionsIdx() const
{
    return _actions_idx;
}

const std::unordered_set<int> &PdtNode::getActionsRepetitionIdx() const
{
    return _actions_repetition_idx;
}

std::vector<PdtNode *> &PdtNode::getChildren()
{
    return _children;
}

const std::vector<PdtNode *> &PdtNode::getChildren() const
{
    return _children;
}

const PdtNode *PdtNode::getParent() const
{
    return _parent;
}

const FactVariables &PdtNode::getFactVariables() const
{
    return _fact_variables;
}

bool PdtNode::isFirstChild() const
{
    return _parent != nullptr && _offset == 0;
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
        _action_variables[action_idx] = VariableProvider::nextVar();
        if (print_var_names)
        {
            std::string action_name = htn.getActionById(action_idx).getName() + "__" + getPositionString();
            Log::i("PVN: %d %s\n", _action_variables[action_idx], action_name.c_str());
        }
    }

    // For each action repetition, get the same variable as the parent action
    // for (int action_idx : _actions_repetition_idx)
    // {
    //     _action_variables[action_idx] = _parent->_action_variables[action_idx];
    // }

    // Assign a variable to each predicate
    // If we are the first child of the parent, we have the same fact variables as the parent
    if (isFirstChild())
    {
        _fact_variables = _parent->_fact_variables;
    }
    else
    {
        _fact_variables.first_var = VariableProvider::nextVars(num_predicates);
        if (print_var_names)
        {
            for (int pred_id = 0; pred_id < num_predicates; ++pred_id)
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

        // The action add some blank actions in the remaining children
        for (size_t i = 1; i < num_children; ++i)
        {
            PdtNode *child = _children[i];
            child->addActionIdx(id_blank_action);
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
                for (int sub_method_idx : task.getDecompositionMethodsIdx())
                {
                    jth_child->addMethodIdx(sub_method_idx);
                    Log::d("At subtask %d, adding method %s\n", j, TOSTR(htn.getMethodById(sub_method_idx)));
                }
            }
            else
            {
                // Add the action to the j-th child
                jth_child->addActionIdx(subtask_idx);
                Log::d("At subtask %d, adding action %s\n", j, TOSTR(htn.getActionById(subtask_idx)));
            }
        }
        // The method add blank actions in the remaining children
        for (size_t j = method.getSubtasksIdx().size(); j < num_children; ++j)
        {
            PdtNode *child = _children[j];
            child->addActionIdx(id_blank_action);
        }
    }
}
#include "algo/planner.h"

#include "util/names.h"

void printDebug(HtnInstance &htn)
{
    Log::i("To help starting\n");
    Log::i("You can get and print the predicates using '_htn.getPredicates()':\n");
    for (auto &pred : htn.getPredicates())
    {
        Log::i("Predicate: %s\n", TOSTR(pred));
    }

    Log::i("You can get the initial abstract task using '_htn.getRootTask()':\n");
    Log::i("Format of an abstract task printed is 'name { method1 method2 ... methodN}' where 'methodX' is a method which can be applied to the task.\n");
    const AbstractTask &_root_task = htn.getRootTask();
    Log::i("Root task: %s\n", TOSTR(_root_task));

    Log::i("\nTo get all the methods which are applicable to a given abstract task, you can use <abstract_task>.getDecompositionMethodsIdx():\n");
    Log::i("It will return a vector of integers, each integer being the index of a method. You can then use '_htn.getMethodById(<index>)' to get the method.\n");
    Log::i("Format of a method printed is 'name { subtask1 subtask2 ... subtaskN}' where 'subtaskI' is ith subtask of the method.\n");
    std::vector<int> methods_idx = _root_task.getDecompositionMethodsIdx();
    for (int method_idx : methods_idx)
    {
        const Method &method = htn.getMethodById(method_idx);
        Log::i("Method: %s\n", TOSTR(method));
    }

    const Method &first_method = htn.getMethodById(methods_idx[0]);
    Log::i("\nTo get the subtasks of a method, you can use <method>.getSubtasksIdx():\n");
    Log::i("It will return a vector of integers, each integer being the index of a subtask\n");
    Log::i("The subtask can be either an abstract task or an action. You can check the type of the subtask using '_htn.isAbstractTask(<index>)'.\n");
    Log::i("To get the subtask, you can use '_htn.getAbstractTaskById(<index>)' or '_htn.getActionById(<index>)'.\n");
    std::vector<int> subtasks_idx = first_method.getSubtasksIdx();
    Log::i("Subtask of method %s:\n", TOSTR(first_method));
    for (int subtask_idx : subtasks_idx)
    {
        if (htn.isAbstractTask(subtask_idx))
        {
            const AbstractTask &subtask = htn.getAbstractTaskById(subtask_idx);
            Log::i("  Abstract subtask: %s\n", TOSTR(subtask));
        }
        else
        {
            const Action &subtask = htn.getActionById(subtask_idx);
            Log::i("  Primitive subtask: %s\n", TOSTR(subtask));
        }
    }

    Log::i("To get the initial state, you can use '_htn.getInitState()':\n");
    Log::i("It will return a vector of integers, each integer being the index of a predicate.\n");
    Log::i("You can then use '_htn.getPredicateById(<index>)' to get the predicate.\n");
    std::vector<int> init_state = htn.getInitState();
    Log::i("Initial state:\n");
    for (int pred_id : init_state)
    {
        const Predicate &pred = htn.getPredicateById(pred_id);
        Log::i("  %s\n", TOSTR(pred));
    }

    Log::i("The goal state has been compiled into an action whose preconditions are the goal state predicates.\n");
    Log::i("You can get this action using '_htn.getGoalAction()':\n");
    Log::i("It will return an action with preconditions being the goal state predicates.\n");
    Log::i("Format of an action printed is 'precond1 precond2 ... precondN { name } eff1 eff2 ... effN' where 'precondI' is ith precondition and 'effI' is ith effect.\n");
    Action goal_action = htn.getGoalAction();
    Log::i("Goal action: %s\n", TOSTR(goal_action));

    const Action &first_action = htn.getActionById(0);
    Log::i("To get the preconditions of an action, you can use <action>.getPreconditionsIdx()");
    Log::i("It will return a vector of integers, each integer being the index of a predicate.\n");
    Log::i("First action: %s\n", TOSTR(first_action));
    std::vector<int> preconditions_idx = first_action.getPreconditionsIdx();
    Log::i("Preconditions of first action:\n");
    for (int pred_id : preconditions_idx)
    {
        const Predicate &pred = htn.getPredicateById(pred_id);
        Log::i("  %s\n", TOSTR(pred));
    }
    Log::i("To get the positive effects of an action, you can use <action>.getPosEffsIdx()");
    Log::i("It will return a vector of integers, each integer being the index of a predicate.\n");
    Log::i("First action: %s\n", TOSTR(first_action));
    std::vector<int> pos_effs_idx = first_action.getPosEffsIdx();
    Log::i("Positive effects of first action:\n");
    for (int pred_id : pos_effs_idx)
    {
        const Predicate &pred = htn.getPredicateById(pred_id);
        Log::i("  %s\n", TOSTR(pred));
    }
    Log::i("To get the negative effects of an action, you can use <action>.getNegEffsIdx()");
    Log::i("It will return a vector of integers, each integer being the index of a predicate.\n");
    Log::i("First action: %s\n", TOSTR(first_action));
    std::vector<int> neg_effs_idx = first_action.getNegEffsIdx();
    Log::i("Negative effects of first action:\n");
    for (int pred_id : neg_effs_idx)
    {
        const Predicate &pred = htn.getPredicateById(pred_id);
        Log::i("  %s\n", TOSTR(pred));
    }
}

int Planner::findPlan()
{

    Log::i("Starting the planner\n");
    printDebug(_htn);

    // Initialize Tree

    // Set SAT variables for the initial layer

    // Encode the inital layer

    // Run main loop
    bool solved = false;
    int max_depth = 100;
    int current_depth = 0;
    while (!solved && current_depth < max_depth)
    {
        current_depth++;
        // Expand the Tree with a new layer

        // Set SAT variables for the new layer

        // Encode the new layer

        // Launch the SAT solver
    }

    // If solved, extract the plan and verify it

    return 0;
}
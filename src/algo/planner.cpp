#include "algo/planner.h"
#include "util/names.h"

#include "util/names.h"


int Planner::findPlan()
{

    std::vector<PdtNode*> leaf_nodes;

    // Initialize Tree
    _root_node = new PdtNode(/*parent=*/nullptr);
    int root_method_idx = _htn.getRootTask().getDecompositionMethodsIdx()[0];
    _root_node->addMethodIdx(root_method_idx);

    leaf_nodes.push_back(_root_node);

    // Assign the SAT variables for the new layer
    for (PdtNode* node : leaf_nodes)
    {
        node->assignSatVariables(_htn, _print_var_names);
    }

    // Set SAT variables for the initial layer

    // Encode the inital layer
    _enc.initalEncode(_root_node);

    // Run main loop
    bool solved = false;
    int max_depth = 100;
    int current_depth = 0;
    std::vector<PdtNode*> new_leaf_nodes;
    while (!solved && current_depth < max_depth)
    {
        current_depth++;
        Log::i("For depth %d\n", current_depth);


        Log::i("  Expanding layer...\n");
        // Expand all the leaf nodes
        for (PdtNode* node : leaf_nodes)
        {
            node->expand(_htn);
            for (PdtNode* child : node->getChildren())
            {
                new_leaf_nodes.push_back(child);
            }
        }

        Log::i("  Assigning SAT variables...\n");
        // Assign the SAT variables for the new layer
        for (PdtNode* node : new_leaf_nodes)
        {
            // Log::i("%s\n", TOSTR(*node));
            node->assignSatVariables(_htn, _print_var_names);
        }

        Log::i("  Encoding...\n");
        // Encode the new leaf nodes
        _enc.encode(new_leaf_nodes);

        // Add assumptions that each leaf node must be primitive
        std::vector<int> prim_vars;
        for (PdtNode* node : new_leaf_nodes)
        {
            prim_vars.push_back(node->getPrimVariable());
        }
        _enc.addAssumptions(prim_vars);

        Log::i("  Solving...\n");
        // Launch the SAT solver
        int result = _enc.solve();
        // _enc.writeFormula("formula_" + std::to_string(current_depth) + ".cnf");
        Log::i("    Result: %d\n", result);
        solved = (result == 10);

        leaf_nodes = new_leaf_nodes;
        new_leaf_nodes.clear();
    }
    // If solved, extract the plan and verify it
    if (!solved) {
        Log::w("No success. Exiting.\n");
        return 1;
    }

    Log::i("Found a solution at layer %i.\n", current_depth);

    // For each node in the PDT, indicate which action or method is true
    _enc.setOpsTrueInTree(/*node=*/_root_node);

    // Generate the plan
    if (!_plan_manager.generatePlan(_root_node))
    {
        Log::e("Error: Failed to generate the final plan.\n");
        return 1;
    }
    if (_verify_plan)
    {
        // Verify the plan
        if (!_plan_manager.verifyPlan())
        {
            Log::e("Error: Plan verification failed.\n");
            return 1;
        }
        Log::i("Plan verified successfully.\n");
    }
    // Output the plan
    Log::log_notime(Log::V0_ESSENTIAL, _plan_manager.getPlanString().c_str());
    Log::i("End of solution plan. (counted length of %i)\n", _plan_manager.getPlanSize());

    if (_write_plan) {
        if (!_plan_manager.outputPlan("plan.txt")) {
            Log::e("Error: Failed to write the plan to file.\n");
            return 1;
        }
    }

    return 0;
}
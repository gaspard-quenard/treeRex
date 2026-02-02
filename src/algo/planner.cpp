#include "algo/planner.h"
#include "util/names.h"
#include "data/dag_node.h"

int Planner::findPlan()
{

    std::vector<PdtNode *> leaf_nodes;

    // Only used if we have the parameter computePossFactsAtEachPos
    std::vector<bool> poss_facts;
    if (_compute_poss_facts_at_each_pos)
        poss_facts.resize(_htn.getNumPredicates(), false);

    // Initialize Tree
    _root_node = new PdtNode(/*parent=*/nullptr);
    int root_method_idx = _htn.getRootTask().getDecompositionMethodIdx()[0];
    _root_node->addMethodIdx(root_method_idx);

    leaf_nodes.push_back(_root_node);

    // Assign the SAT variables for the new layer
    for (PdtNode *node : leaf_nodes)
    {
        node->assignSatVariables(_htn, _print_var_names);
    }

    // Encode the inital layer
    _enc.initalEncode(_root_node);

    // Run main loop
    bool solved = false;
    int max_depth = 500;
    int current_depth = 0;

    // Some stats
    int num_nodes = 0;
    int num_methods = 1;
    int num_actions = 0;

    // Used for sibylsat
    std::unordered_set<PdtNode *> nodes_to_expand;
    std::vector<PdtNode *> nodes_to_encode;
    if (_sibylsat_expansion)
        nodes_to_expand.insert(_root_node);

    std::vector<PdtNode *> new_leaf_nodes;
    while (!solved && current_depth < max_depth)
    {
        current_depth++;
        new_leaf_nodes.clear();

        if (_compute_poss_facts_at_each_pos)
        {
            // Reinitialize the possible facts with the init state
            for (int i = 0; i < poss_facts.size(); i++)
                poss_facts[i] = false;
            for (int pred_id : _htn.getInitState())
                poss_facts[pred_id] = true;
        }

        Log::i("For depth %d\n", current_depth);

        Log::i("  Expanding layer...\n");

        _stats.beginTiming(TimingStage::EXPANSION);
        if (_sibylsat_expansion)
        {
            Log::i("  ... Using SibylSAT expansion\n");
            for (PdtNode *node : leaf_nodes)
            {
                // Should we expand this node?
                if (nodes_to_expand.find(node) != nodes_to_expand.end())
                {
                    // Expand the node
                    node->expand(_htn);
                    for (PdtNode *child : node->getChildren())
                    {
                        new_leaf_nodes.push_back(child);
                        nodes_to_encode.push_back(child);
                        // Update stats
                        num_nodes++;
                        num_methods += child->getNumMethods();
                        num_actions += child->getNumActions();

                        if (_compute_poss_facts_at_each_pos)
                        {
                            // First, get all actions/methods whose preconditions cannot be satisfied
                            std::unordered_set<std::pair<int, OpType>, PairHash> impossible_ops = child->getAllImpossibleOps(_htn, poss_facts);
                            // Prune the impossible ops
                            for (std::pair<int, OpType> op : impossible_ops)
                            {
                                // Log::i("Prune op %s\n", op.second == OpType::ACTION ? TOSTR(_htn.getActionById(op.first)));
                                _retroactive_pruning.prune(child, op.first, op.second);
                            }

                            // Update the possible facts with the new actions/methods
                            child->updatePossFacts(_htn, poss_facts);
                        }
                    }
                }
                else
                {
                    // Add the node to the new layer
                    new_leaf_nodes.push_back(node);

                    if (_compute_poss_facts_at_each_pos)
                    {
                        // Update the possible facts with the new actions/methods
                        node->updatePossFacts(_htn, poss_facts);
                    }
                }
            }
        }
        else
        {
            Log::i("  ... Using Lilotane expansion\n");
            // Expand all the leaf nodes
            for (PdtNode *node : leaf_nodes)
            {
                node->expand(_htn);
                for (PdtNode *child : node->getChildren())
                {
                    new_leaf_nodes.push_back(child);
                    // Update stats
                    num_nodes++;
                    num_methods += child->getNumMethods();
                    num_actions += child->getNumActions();
                }
            }
        }
        _stats.endTiming(TimingStage::EXPANSION);

        Log::i("    * Size of new layer: %d\n", new_leaf_nodes.size());
        Log::i("    * Number of nodes: %d\n", num_nodes);
        Log::i("    * Number of methods: %d\n", num_methods);
        Log::i("    * Number of actions: %d\n", num_actions);

        Log::i("  Assigning SAT variables...\n");
        // Assign the SAT variables for the new layer
        for (PdtNode *node : new_leaf_nodes)
        {
            // Log::i("%s\n", TOSTR(*node));
            node->assignSatVariables(_htn, _print_var_names);
        }

        Log::i("  Encoding...\n");
        // Encode the new leaf nodes
        _stats.beginTiming(TimingStage::ENCODING);
        _enc.encode(new_leaf_nodes);
        _stats.endTiming(TimingStage::ENCODING);

        // Add assumptions that each leaf node must be primitive
        std::vector<int> prim_vars;
        for (PdtNode *node : new_leaf_nodes)
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

        if (!solved && _sibylsat_expansion)
        {
            Log::i("  Failed to find a solution. Try to find a relaxed solution...\n");
            // Find a relaxed solution to determinate which nodes to expand
            // Launch the SAT solver wihout assumptions
            int result = _enc.solve();
            bool relaxed_solved = (result == 10);
            if (!relaxed_solved)
            {
                Log::e("No solution possible for this problem !\n");
                return 1;
            }
            Log::i("Found a relaxed solution !\n");

            _enc.setOpsTrueInTree(/*node=*/_root_node, /*accept_relaxed_solution=*/true);

            // Indicate all the nodes to expand
            for (PdtNode *node : new_leaf_nodes)
            {
                if (node->getOpSolution().second == OpType::METHOD)
                {
                    nodes_to_expand.insert(node);
                }
            }
        }

        leaf_nodes = new_leaf_nodes;
    }

    // If solved, extract the plan and verify it
    if (!solved)
    {
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

    if (_write_plan)
    {
        if (!_plan_manager.outputPlan("plan.txt"))
        {
            Log::e("Error: Failed to write the plan to file.\n");
            return 1;
        }
    }

    // Print final stats
    Log::i("Final stats:\n");
    Log::i("  * Number of nodes: %d\n", num_nodes);
    Log::i("  * Number of methods: %d\n", num_methods);
    Log::i("  * Number of actions: %d\n", num_actions);
    Log::i("  * Number of layers: %d\n", current_depth);
    // Add pruning stats
    Log::i("  * Number of pruned actions: %d\n", _retroactive_pruning.getNumPrunedActions());
    Log::i("  * Number of pruned methods: %d\n", _retroactive_pruning.getNumPrunedMethods());

    return 0;
}

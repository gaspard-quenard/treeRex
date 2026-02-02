#include "algo/retroactive_pruning.h"
#include "util/log.h"
#include "util/names.h" // For TOSTR
#include "sat/encoding.h" // For _enc.addClause
#include <vector>         // For storing children to prune
#include <set>            // For collecting ops to prune in children

void RetroactivePruning::prune(PdtNode *node, int op_idx, OpType op_type)
{
    // 1. Check if the node contains the operation. Use count() for unordered_set check.
    if (node->getOpsIdx(op_type).count(op_idx) == 0)
    {
        // This might happen if prune is called recursively on an op already removed by another path.
        // Log::d("Prune: Node %s already lacks op %d (%s)\n", node->getPositionString().c_str(), op_idx, TOSTR(op_type));
        return;
    }

    Log::d("Pruning op %d (%s) from node %s\n", op_idx, TOSTR(op_type), node->getPositionString().c_str());

    // 2. Add clause to SAT encoding to disable this operation, if variable exists
    int sat_var = 0;
    if (op_type == OpType::METHOD)
    {
        num_pruned_methods++; // Count pruned method
        auto it = node->getMethodAndVariables().find(op_idx);
        if (it != node->getMethodAndVariables().end())
        {
            sat_var = it->second;
            
        }
    }
    else // OpType::ACTION
    {
        num_pruned_actions++; // Count pruned action
        auto it = node->getActionAndVariables().find(op_idx);
        if (it != node->getActionAndVariables().end())
        {
            sat_var = it->second;
        }
    }

    if (sat_var != 0)
    {
        // Log::i("Adding clause to disable var %d for op %d (%s) in node %s\n", sat_var, op_idx, TOSTR(op_type), node->getPositionString().c_str());
        _enc.addUnitConstraint(-sat_var);
    } else {
        // Log::w("No SAT variable found for op %d (%s) in node %s during pruning.\n", op_idx, TOSTR(op_type), node->getPositionString().c_str());
        // This might be okay if pruning happens before full encoding, or if it's an action repetition without its own var.
    }


    // 3. Get the parents of the operation *before* removing it.
    std::unordered_set<std::pair<int, OpType>, PairHash> parents = node->getParentsOfOp(op_idx, op_type);

    // 4. Collect children ops that depend *solely* on this op *before* removing it.
    // We need to do this before modifying the node's state.
    std::vector<std::pair<PdtNode*, std::pair<int, OpType>>> children_ops_to_prune;
    std::pair<int, OpType> current_op_pair = {op_idx, op_type};

    for (PdtNode* child_node : node->getChildren()) {
        // Check child methods
        for (int child_method_idx : child_node->getMethodsIdx()) {
            auto child_parents = child_node->getParentsOfOp(child_method_idx, OpType::METHOD);
            if (child_parents.size() == 1 && child_parents.count(current_op_pair)) {
                children_ops_to_prune.push_back({child_node, {child_method_idx, OpType::METHOD}});
            }
        }
        // Check child actions
        for (int child_action_idx : child_node->getActionsIdx()) {
             // Skip blank action derived from this parent if this parent is a method
             if (op_type == OpType::METHOD && child_action_idx == _htn.getBlankAction().getId()) {
                 auto child_parents = child_node->getParentsOfOp(child_action_idx, OpType::ACTION);
                 bool parented_by_pruned_op = false;
                 for(const auto& p : child_parents) {
                     if (p == current_op_pair) {
                         parented_by_pruned_op = true;
                         break;
                     }
                 }
                 // If the blank action has other parents OR is not parented by the op being pruned, don't prune it.
                 // We only prune it if its *sole* parent is the method being pruned.
                 if (!parented_by_pruned_op || child_parents.size() > 1) {
                     continue;
                 }
             }

            auto child_parents = child_node->getParentsOfOp(child_action_idx, OpType::ACTION);
            if (child_parents.size() == 1 && child_parents.count(current_op_pair)) {
                 children_ops_to_prune.push_back({child_node, {child_action_idx, OpType::ACTION}});
            }
        }
    }


    // 5. Remove the operation from the current node.
    bool removed = node->removeOp(op_idx, op_type);
    if (!removed) {
         Log::e("Failed to remove op %d (%s) from node %s, inconsistency detected.\n", op_idx, TOSTR(op_type), node->getPositionString().c_str());
         return; // Avoid further processing if removal failed
    }


    // 6. Check if any parent method now has no children (expansions) left in this node.
    //    If so, recursively prune that parent method from this node.
    std::vector<std::pair<int, OpType>> parents_to_prune; // Collect parents to prune after iteration
    for (const auto& parent_pair : parents) {
        if (parent_pair.second == OpType::METHOD) {
            int parent_method_idx = parent_pair.first;
            // Check if the parent method still exists in the node (it might have been pruned already)
            if (node->getMethodsIdx().count(parent_method_idx)) {
                 // Check if its expansion map entry exists and is empty
                 auto expansion_map_it = node->getExpansionsOfParentMethodMap().find(parent_method_idx);
                 if (expansion_map_it != node->getExpansionsOfParentMethodMap().end() && expansion_map_it->second.empty()) {
                     Log::d("Parent method %d of pruned op %d (%s) in node %s now has no expansions. Adding to prune list.\n", parent_method_idx, op_idx, TOSTR(op_type), node->getPositionString().c_str());
                     parents_to_prune.push_back({parent_method_idx, OpType::METHOD});
                 }
            }
        }
        // Actions cannot be parents in the expansion map, so no need to check OpType::ACTION here.
    }
    // Prune the collected parents
    for (const auto& parent_to_prune : parents_to_prune) {
        prune(node, parent_to_prune.first, parent_to_prune.second);
    }


    // 7. Recursively prune the collected children ops.
    for (const auto& child_op_to_prune : children_ops_to_prune) {
        prune(child_op_to_prune.first, child_op_to_prune.second.first, child_op_to_prune.second.second);
    }

}

#include "sat/dag_encoding.h"
#include "util/names.h"

#include <cstdint>

void DAG_Encoding::encode(std::vector<DagNode *> &prim_nodes)
{

    // Reset the SAT solver
    _sat.reset();

    int num_timestep = 0;
    for (DagNode *n : prim_nodes)
        num_timestep = std::max(num_timestep, n->getTs());
    Log::i("Encoding DAG with %d primitive nodes and %d timesteps...\n", prim_nodes.size(), num_timestep);

    // Assign a sat variable to each primitive node
    for (int i = 0; i < (int)prim_nodes.size(); i++)
        prim_nodes[i]->setSatVar(i + 1); // Start from 1

    // Connectivity anchor (HTN-style): some instances have no fact-based goal (i.e., the
    // goal state is empty), and the goal pseudo-action can be empty. In that case we
    // require a path init -> ... -> goal by selecting the init node and forbidding any
    // other selected root.
    //
    // IMPORTANT: getFactVarsGoal() is a variable pool and is typically non-empty even
    // when there is no goal state. Use getGoalState().empty() to detect the HTN-style case.
    const bool has_fact_goal = !_htn.getGoalState().empty();
    const bool enforce_connectivity = !has_fact_goal;

    Log::i("DAG_Encoding: goal_state=%zu, fact_goal_mode=%d, enforce_connectivity=%d\n",
           _htn.getGoalState().size(), has_fact_goal ? 1 : 0, enforce_connectivity ? 1 : 0);

    // Force the goal pseudo-action node to be selected only in the HTN-style case.
    if (enforce_connectivity)
    {
        std::vector<int> goal_vars;
        for (DagNode *n : prim_nodes)
            if (n->getOpId() == _htn.getIdGoalAction())
                goal_vars.push_back(n->getSatVar());
        if (goal_vars.size() == 1)
            _sat.addClause(goal_vars[0]);
        else if (!goal_vars.empty())
            _sat.addClause(goal_vars);
    }

    // Keep the original sequential semantics: at most one selected node per timestep,
    // but do not force any timestep to have a node.
    // NOTE: We encode this after we establish next_free_var (shared with edge vars),
    // because the linear at-most-one encoding uses auxiliary variables.

    // If there is no fact-based goal (HTN-style), selecting only init/goal pseudo-actions
    // is a trivial "solution" that does not execute the decomposition's real actions.
    // Require at least one real action node to be selected.
    if (enforce_connectivity)
    {
        std::vector<int> real_action_vars;
        real_action_vars.reserve(prim_nodes.size());
        const int blank_id = _htn.getBlankAction().getId();
        const int init_id = _htn.getIdInitAction();
        const int goal_id = _htn.getIdGoalAction();
        for (DagNode *n : prim_nodes)
        {
            if (!n)
                continue;
            const int op_id = n->getOpId();
            if (op_id == blank_id || op_id == init_id || op_id == goal_id)
                continue;
            real_action_vars.push_back(n->getSatVar());
        }
        if (!real_action_vars.empty())
            _sat.addClause(real_action_vars);
    }

    int current_prim_node = 0;
    int current_timestep = 0;

    std::unordered_map<int, std::unordered_set<int>> fa_pos;
    std::unordered_map<int, std::unordered_set<int>> fa_neg;

    int init_state_pred_var = prim_nodes.size() + 1;
    // TODO encode initial state there
    for (int i = 0; i < _htn.getNumPredicates(); i++)
    {
        // Add a variable for each predicate
        int pred_var = init_state_pred_var + i;
        // Add the initial state predicate to the frame axiom
        if (_htn.getInitState().count(i) > 0)
        {
            _sat.addClause(pred_var);
        }
        else
        {
            _sat.addClause(-pred_var);
        }
    }

    // NOTE: We do NOT force "one action per timestep" nor "at least one at ts=0".
    // Some problems/plans may allow stuttering steps.

    int last_ts = 0;

    // Edge variables E(u,v) (u->v is chosen in the path)
    // We index edges after predicate variables to avoid collisions.
    const int first_pred_var = init_state_pred_var;
    const int first_edge_var = first_pred_var + _htn.getNumPredicates() * (num_timestep + 2);
    int next_free_var = first_edge_var;

    auto encodeAtMostOneLinear = [&](const std::vector<int> &vars) {
        // Sequential (Sinz) encoding: O(n) clauses and O(n) auxiliary variables.
        // For tiny sets, pairwise is fine and avoids extra vars.
        const int n = static_cast<int>(vars.size());
        if (n <= 1)
            return;
        if (n <= 5)
        {
            encodeAtMostOne(vars);
            return;
        }

        // s[0..n-2]
        std::vector<int> s;
        s.resize(static_cast<std::size_t>(n - 1));
        for (int i = 0; i < n - 1; ++i)
            s[static_cast<std::size_t>(i)] = next_free_var++;

        // (-x1 or s1)
        _sat.addClause(-vars[0], s[0]);
        // For 2..n-1
        for (int i = 1; i < n - 1; ++i)
        {
            // (-xi or si)
            _sat.addClause(-vars[i], s[i]);
            // (-s_{i-1} or si)
            _sat.addClause(-s[i - 1], s[i]);
            // (-xi or -s_{i-1})
            _sat.addClause(-vars[i], -s[i - 1]);
        }
        // (-xn or -s_{n-1})
        _sat.addClause(-vars[n - 1], -s[n - 2]);
    };

    // Keep the original sequential semantics: at most one selected node per timestep,
    // but do not force any timestep to have a node.
    {
        std::vector<std::vector<int>> vars_by_ts;
        vars_by_ts.resize(static_cast<std::size_t>(num_timestep + 1));
        for (DagNode *n : prim_nodes)
        {
            const int ts = n->getTs();
            if (ts >= 0 && ts <= num_timestep)
                vars_by_ts[static_cast<std::size_t>(ts)].push_back(n->getSatVar());
        }
        for (auto &bucket : vars_by_ts)
            if (bucket.size() > 1)
                encodeAtMostOneLinear(bucket);
    }

    std::unordered_set<DagNode *> prim_set;
    prim_set.reserve(prim_nodes.size());
    for (DagNode *n : prim_nodes)
        prim_set.insert(n);

    auto edgeKey = [](int u, int v) -> std::uint64_t {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(u)) << 32) |
               static_cast<std::uint64_t>(static_cast<std::uint32_t>(v));
    };

    std::unordered_map<std::uint64_t, int> edge_var;
    edge_var.reserve(prim_nodes.size() * 4);

    auto getEdgeVar = [&](DagNode *u, DagNode *v) -> int {
        std::uint64_t k = edgeKey(u->getSatVar(), v->getSatVar());
        auto it = edge_var.find(k);
        if (it != edge_var.end())
            return it->second;
        int var = next_free_var++;
        edge_var.emplace(k, var);
        // E(u,v) => u and v are selected
        _sat.addClause(-var, u->getSatVar());
        _sat.addClause(-var, v->getSatVar());
        return var;
    };
    for (DagNode *current_node : prim_nodes)
    {
        int ts = current_node->getTs();

        int current_pred_var = init_state_pred_var + _htn.getNumPredicates() * ts;
        int next_pred_var = current_pred_var + _htn.getNumPredicates();

        if (ts < last_ts)
        {
            Log::e("Error: Primitive nodes are not sorted by timestep. Current timestep: %d, Last timestep: %d\n", ts, last_ts);
            exit(1);
        }
        // Advance time layers. If there is a gap (no nodes for some timesteps), we must
        // still emit frame axioms for each intermediate timestep; otherwise we incorrectly
        // connect last_ts directly to ts, skipping layers.
        while (last_ts < ts)
        {
            const int prev_pred_var = init_state_pred_var + _htn.getNumPredicates() * last_ts;
            const int curr_pred_var = init_state_pred_var + _htn.getNumPredicates() * (last_ts + 1);

            for (int i = 0; i < _htn.getNumPredicates(); ++i)
            {
                const int pred_var = prev_pred_var + i; // layer last_ts
                const int next_var = curr_pred_var + i; // layer last_ts+1

                _sat.appendClause(-pred_var, next_var); // keep-true unless deleted
                for (int neg_var : fa_neg[i])
                    _sat.appendClause(neg_var);
                _sat.endClause();

                _sat.appendClause(pred_var, -next_var); // keep-false unless added
                for (int pos_var : fa_pos[i])
                    _sat.appendClause(pos_var);
                _sat.endClause();
            }

            fa_pos.clear();
            fa_neg.clear();
            last_ts++;

            // If mutex param is true, encode mutexes between predicates for this timestep
            // (the new layer at last_ts).
            if (_encode_mutexes)
            {
                for (const std::vector<int> &group : _htn.getMutex().getMutexGroups())
                {
                    std::vector<int> group_vars;
                    group_vars.reserve(group.size());
                    for (int pred_idx : group)
                        group_vars.push_back(curr_pred_var + pred_idx);
                    if (group_vars.size() > 1)
                        encodeAtMostOneLinear(group_vars);
                }
            }
        }

        // Build incoming/outgoing edge sets among primitive nodes
        std::vector<int> in_edges;
        std::vector<int> out_edges;
        in_edges.reserve(current_node->getPrev().size());

        for (DagNode *prev_node : current_node->getPrev())
        {
            if (!prev_node->isActive())
                continue;
            if (!prim_set.count(prev_node))
                continue;
            in_edges.push_back(getEdgeVar(prev_node, current_node));
        }

        if (enforce_connectivity)
        {
            const int op_id = current_node->getOpId();
            const bool is_init = (op_id == _htn.getIdInitAction());
            // If this primitive node has no primitive predecessors, it is a root.
            // Only the init node is allowed to be selected as a root.
            if (in_edges.empty() && !is_init)
            {
                _sat.addClause(-current_node->getSatVar());
            }
            if (is_init)
            {
                _sat.addClause(current_node->getSatVar());
            }
        }

        // Successors: conditional union if available, else unconditional
        std::unordered_set<DagNode *> succs;
        if (current_node->hasConditionalNext())
        {
            
            for (const auto &kv : current_node->getConditionalNext())
                for (DagNode *s : kv.second)
                    succs.insert(s);
        }
        else
        {
            for (DagNode *s : current_node->getNext())
                succs.insert(s);
        }

        for (DagNode *next_node : succs)
        {
            if (!next_node->isActive())
                continue;
            if (!prim_set.count(next_node))
                continue;
            out_edges.push_back(getEdgeVar(current_node, next_node));
        }

        // If node is selected, it must have one incoming (unless it has no predecessors)
        if (!in_edges.empty())
        {
            std::vector<int> clause;
            clause.reserve(in_edges.size() + 1);
            clause.push_back(-current_node->getSatVar());
            for (int e : in_edges)
                clause.push_back(e);
            _sat.addClause(clause);

            if (in_edges.size() > 1)
                encodeAtMostOneLinear(in_edges);
        }

        // If node is selected, it must have one outgoing (unless it has no successors)
        if (!out_edges.empty())
        {
            std::vector<int> clause;
            clause.reserve(out_edges.size() + 1);
            clause.push_back(-current_node->getSatVar());
            for (int e : out_edges)
                clause.push_back(e);
            _sat.addClause(clause);

            if (out_edges.size() > 1)
                encodeAtMostOneLinear(out_edges);
        }

        // Conditional successor gating (singleton predecessor-context only)
        if (current_node->hasConditionalNext() && !in_edges.empty() && !out_edges.empty())
        {
            // allowed[pred] = set of successors allowed when reached from pred
            std::unordered_map<DagNode *, std::unordered_set<DagNode *>> allowed;
            for (const auto &kv : current_node->getConditionalNext())
            {
                DagNode *pred = kv.first;
                if (!pred || !prim_set.count(pred))
                    continue;
                for (DagNode *s : kv.second)
                    if (s && prim_set.count(s))
                        allowed[pred].insert(s);
            }

            if (!allowed.empty())
            {
                // Precompute outgoing edge vars by successor
                std::unordered_map<DagNode *, int> out_var_by_succ;
                out_var_by_succ.reserve(out_edges.size());
                for (DagNode *s : succs)
                {
                    if (!s || !prim_set.count(s) || !s->isActive())
                        continue;
                    out_var_by_succ.emplace(s, getEdgeVar(current_node, s));
                }

                for (const auto &kv : allowed)
                {
                    DagNode *pred = kv.first;
                    const auto &succ_allowed = kv.second;

                    // incoming edge var pred -> current
                    if (!pred->isActive())
                        continue;
                    int in_var = getEdgeVar(pred, current_node);

                    // pred->current implies choose one allowed outgoing edge
                    std::vector<int> clause;
                    clause.reserve(succ_allowed.size() + 1);
                    clause.push_back(-in_var);
                    for (DagNode *s : succ_allowed)
                    {
                        auto it = out_var_by_succ.find(s);
                        if (it != out_var_by_succ.end())
                            clause.push_back(it->second);
                    }
                    if (clause.size() > 1)
                        _sat.addClause(clause);

                    // forbid disallowed outgoing edges when reached from pred
                    for (const auto &ov : out_var_by_succ)
                    {
                        DagNode *s = ov.first;
                        int e_us = ov.second;
                        if (succ_allowed.count(s))
                            continue;
                        _sat.addClause(-in_var, -e_us);
                    }
                }
            }
        }

        // Encode preconditions and effects of the current node.
        // Note: the init pseudo-action (-2) is not part of _actions, so treat it as an empty no-op.
        if (current_node->getOpId() != _htn.getIdInitAction())
        {
            const Action &action = _htn.getActionById(current_node->getOpId());
            for (const int &precondition_idx : action.getPreconditionsIdx())
            {
                // Add the precondition to the frame axiom
                _sat.addClause(-current_node->getSatVar(), current_pred_var + precondition_idx);
            }
            for (const int &pos_effect_idx : action.getPosEffsIdx())
            {
                // Add the positive effect to the frame axiom
                int pred_id = next_pred_var + pos_effect_idx;
                _sat.addClause(-current_node->getSatVar(), pred_id);
                fa_pos[pos_effect_idx].insert(current_node->getSatVar());
            }
            for (const int &neg_effect_idx : action.getNegEffsIdx())
            {
                // Add the negative effect to the frame axiom
                int pred_id = next_pred_var + neg_effect_idx;
                _sat.addClause(-current_node->getSatVar(), -pred_id);
                fa_neg[neg_effect_idx].insert(current_node->getSatVar());
            }
        }
    }

    if (!_htn.getGoalState().empty())
    {

        // Need to do the frame axioms for the last timestep
        int prev_pred_var = init_state_pred_var + _htn.getNumPredicates() * (num_timestep);
        int goal_state_pred_var = init_state_pred_var + _htn.getNumPredicates() * (num_timestep + 1);
        for (int i = 0; i < _htn.getNumPredicates(); ++i)
        {
            int pred_var = prev_pred_var + i;       // layer  last_ts
            int next_var = goal_state_pred_var + i; // layer  ts
            _sat.appendClause(-pred_var, next_var); // keep‐true unless deleted
            for (int neg_var : fa_neg[i])
                _sat.appendClause(neg_var);
            _sat.endClause();
            _sat.appendClause(pred_var, -next_var); // keep‐false unless added
            for (int pos_var : fa_pos[i])
                _sat.appendClause(pos_var);
            _sat.endClause();
        }

        for (int pred_idx : _htn.getGoalState())
        {
            int pred_var = goal_state_pred_var + pred_idx;
            _sat.addClause(pred_var); // The goal predicate must be true
        }
    }
}

void DAG_Encoding::encodeAtMostOne(const std::vector<int> &vars)
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

int DAG_Encoding::solve()
{
    return _sat.solve();
}

void DAG_Encoding::printPlanFromDagNodes(const std::vector<DagNode *> &dag_nodes)
{
    Log::i("Plan actions:\n");

    int printed = 0;
    const int blank_id = _htn.getBlankAction().getId();
    const int init_id = _htn.getIdInitAction();
    const int goal_id = _htn.getIdGoalAction();
    for (DagNode *n : dag_nodes)
    {
        if (!n)
            continue;
        const int v = n->getSatVar();
        if (v <= 0)
            continue;

        if (_sat.holds(v))
        {
            const int op_id = n->getOpId();
            if (op_id == blank_id || op_id == init_id || op_id == goal_id)
                continue;
            const Action &a = _htn.getActionById(n->getOpId());
            const std::string name = a.getName();
            if (!name.empty())
                Log::i("%d: %s\n", n->getTs(), name.c_str());
            else
                Log::i("%d: <action id=%d>\n", n->getTs(), n->getOpId());
            printed++;
        }
    }

    if (printed == 0)
        Log::i("(no actions selected)\n");
    else
        Log::i("(selected %d action nodes)\n", printed);

    // Validate the printed plan by simulating it from the initial state.
    // This checks that each selected action's preconditions are satisfied by the
    // state produced by previous selected actions, and that effects update state.
    {
        struct Step
        {
            int ts;
            DagNode *node;
        };

        std::vector<Step> steps;
        steps.reserve(static_cast<std::size_t>(printed));
        for (DagNode *n : dag_nodes)
        {
            if (!n)
                continue;
            const int v = n->getSatVar();
            if (v <= 0)
                continue;
            if (!_sat.holds(v))
                continue;

            const int op_id = n->getOpId();
            if (op_id == blank_id || op_id == init_id || op_id == goal_id)
                continue;
            steps.push_back(Step{n->getTs(), n});
        }

        std::sort(steps.begin(), steps.end(), [](const Step &a, const Step &b) {
            return a.ts < b.ts;
        });

        // Track state as a dense bit-vector for speed.
        const int num_preds = _htn.getNumPredicates();
        std::vector<uint8_t> state(static_cast<std::size_t>(num_preds), 0);
        for (int p : _htn.getInitState())
        {
            if (p >= 0 && p < num_preds)
                state[static_cast<std::size_t>(p)] = 1;
        }

        bool ok = true;
        for (const Step &st : steps)
        {
            DagNode *n = st.node;
            const int op_id = n->getOpId();

            // init pseudo-action is excluded above; other actions should exist in _htn.
            const Action &action = _htn.getActionById(op_id);

            std::vector<int> missing;
            for (const int &pre_idx : action.getPreconditionsIdx())
            {
                if (pre_idx < 0 || pre_idx >= num_preds)
                    continue;
                if (!state[static_cast<std::size_t>(pre_idx)])
                    missing.push_back(pre_idx);
            }

            if (!missing.empty())
            {
                ok = false;
                Log::e("Plan validation failed at ts=%d: preconditions not satisfied for action %s\n",
                       st.ts, Names::to_string(action).c_str());
                for (int pred_idx : missing)
                    Log::e("  missing: %s\n", Names::to_string(_htn.getPredicateById(pred_idx)).c_str());
            }

            // Apply effects to state.
            for (const int &neg_idx : action.getNegEffsIdx())
            {
                if (neg_idx >= 0 && neg_idx < num_preds)
                    state[static_cast<std::size_t>(neg_idx)] = 0;
            }
            for (const int &pos_idx : action.getPosEffsIdx())
            {
                if (pos_idx >= 0 && pos_idx < num_preds)
                    state[static_cast<std::size_t>(pos_idx)] = 1;
            }
        }

        if (!_htn.getGoalState().empty())
        {
            std::vector<int> missing_goal;
            for (int pred_idx : _htn.getGoalState())
            {
                if (pred_idx < 0 || pred_idx >= num_preds)
                    continue;
                if (!state[static_cast<std::size_t>(pred_idx)])
                    missing_goal.push_back(pred_idx);
            }
            if (!missing_goal.empty())
            {
                ok = false;
                Log::e("Plan validation failed: final state does not satisfy goal. Missing %zu goal predicates:\n",
                       missing_goal.size());
                for (int pred_idx : missing_goal)
                    Log::e("  missing goal: %s\n", Names::to_string(_htn.getPredicateById(pred_idx)).c_str());
            }
        }

        if (ok)
            Log::i("Plan validation: OK (preconditions/effects/goal consistent with simulated state).\n");
        else
            Log::e("Plan validation: FAILED (see messages above).\n");
    }
}
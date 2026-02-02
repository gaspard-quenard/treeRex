#include "algo/planner.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>




int Planner::findPlanLtp()
{
    // Implement the LTP-specific planning logic here

    const bool compress_dag = _htn.getParams().isNonzero("compress_dag");
    const bool no_sat_encoding = _htn.getParams().isNonzero("no_sat_encoding");
    const bool optimized_expansion = _htn.getParams().isNonzero("optimized_expansion");
    const bool htn_connectivity_mode = _htn.getGoalState().empty();

    std::vector<DagNode *> nodes;

    // Always start from the top method (root decomposition).
    DagNode *top_method_node = new DagNode(/*op_id=*/_htn.getRootTask().getDecompositionMethodIdx()[0], /*last_child=*/true, /*type=*/NodeType::METHOD);
    // Seed a consistent time label for incremental timestamping.
    top_method_node->setTs(htn_connectivity_mode ? 1 : 0);
    nodes.push_back(top_method_node);

    // In HTN-connectivity mode (no fact goal), we add init/goal pseudo-actions to
    // anchor an explicit init->...->goal path.
    if (htn_connectivity_mode)
    {
        DagNode *init_node = new DagNode(/*op_id=*/_htn.getIdInitAction(), /*last_child=*/true, /*type=*/NodeType::ACTION);
        DagNode *goal_node = new DagNode(/*op_id=*/_htn.getIdGoalAction(), /*last_child=*/true, /*type=*/NodeType::ACTION);

        init_node->setTs(0);
        goal_node->setTs(2);

        init_node->addNext(top_method_node);
        top_method_node->addNext(goal_node);

        nodes.push_back(init_node);
        nodes.push_back(goal_node);
    }

    // Run main loop
    bool solved = false;
    int max_depth = _htn.getParams().getIntParam("D", 20);
    if (max_depth <= 0)
    {
        Log::w("LTP: -D is %d; using default max depth 20.\n", max_depth);
        max_depth = 20;
    }
    int current_depth = 0;
    bool saw_any_real_primitive = false;

    // With compression, the frontier can quickly reach a fixed point.
    // When enabled (optimized_expansion=1), stop deeper iterations once the
    // compressed frontier stabilizes.
    std::uint64_t prev_frontier_sig = 0;
    std::size_t prev_frontier_size = 0;
    int stable_frontier_iters = 0;

    while (!solved && current_depth < max_depth)
    {
        current_depth++;
        Log::i("For depth %d\n", current_depth);

        Log::i("  Expanding nodes...\n");

        // Expand exactly like before (no compression during expansion)
        std::vector<DagNode *> new_dag_nodes;
        new_dag_nodes.reserve(nodes.size() * 2);
        for (DagNode *node : nodes)
        {
            std::vector<DagNode*> children = node->expand(_htn);
            if (node->isMethod()) {
                for (DagNode *child : children)
                    new_dag_nodes.push_back(child);
            } else {
                new_dag_nodes.push_back(node);
            }
        }
        nodes = new_dag_nodes;

        Log::i("Number of nodes after expansion: %zu\n", nodes.size());


        // Now, get a topological sort of the nodes
        Log::i("Topological sort of the nodes...\n");
        std::vector<DagNode*> ordered;
        ordered.reserve(nodes.size());

        const bool skip_topo_and_kahn = optimized_expansion && no_sat_encoding && compress_dag;

        // If compress_dag is enabled, we will assign timestamps while performing a
        // topological sort. This avoids a second full pass over predecessor sets.
        if (compress_dag && !skip_topo_and_kahn)
        {
            const std::size_t n_nodes = nodes.size();

            std::unordered_map<DagNode *, std::size_t> idx;
            idx.reserve(n_nodes);
            for (std::size_t i = 0; i < n_nodes; ++i)
                idx.emplace(nodes[i], i);

            if (idx.size() != n_nodes)
                throw std::logic_error("Graph contains duplicate node pointers; topological sort impossible");

            std::vector<std::size_t> indeg(n_nodes, 0);
            for (std::size_t i = 0; i < n_nodes; ++i)
            {
                DagNode *u = nodes[i];
                u->setTs(0);
                for (DagNode *v : u->getNext())
                {
                    auto it = idx.find(v);
                    if (it == idx.end())
                        continue;
                    ++indeg[it->second];
                }
            }

            std::queue<std::size_t> q;
            for (std::size_t i = 0; i < n_nodes; ++i)
                if (indeg[i] == 0)
                    q.push(i);

            while (!q.empty())
            {
                const std::size_t ui = q.front();
                q.pop();
                DagNode *u = nodes[ui];
                ordered.push_back(u);

                const int next_ts = u->getTs() + 1;
                for (DagNode *v : u->getNext())
                {
                    auto it = idx.find(v);
                    if (it == idx.end())
                        continue;
                    const std::size_t vi = it->second;
                    if (v->getTs() < next_ts)
                        v->setTs(next_ts);
                    if (--indeg[vi] == 0)
                        q.push(vi);
                }
            }

            if (ordered.size() != n_nodes)
                throw std::logic_error("Graph contains a cycle; topological sort impossible");
        }
        else if (skip_topo_and_kahn)
        {
            // In expansion-only mode, we can rely on timestamps assigned incrementally
            // during DagNode::expand(), so we avoid a full induced-subgraph topo pass.
            ordered = nodes;
            std::sort(ordered.begin(), ordered.end(),
                      [](DagNode *a, DagNode *b) { return a->getTs() < b->getTs(); });
        }
        else
        {
            ordered = DagNode::topologicalSort(nodes);
        }

        // Keep the working set in topological order even without compression.
        // DagNode::expand() assumes METHOD predecessors are expanded before dependent ACTIONs.
        nodes = ordered;

        // If compress_dag is enabled, compress the full expanded DAG and keep working on it.
        // This drastically reduces the number of nodes carried to the next iteration.
        if (compress_dag)
        {

            auto compressFullDag = [&](const std::vector<DagNode*> &input_nodes) {
                struct NodeKey {
                    NodeType type;
                    int op_id;
                    int ts;
                    bool operator==(const NodeKey &o) const { return type == o.type && op_id == o.op_id && ts == o.ts; }
                };
                struct NodeKeyHash {
                    std::size_t operator()(const NodeKey &k) const {
                        std::size_t h = (static_cast<std::size_t>(k.op_id) << 1) ^ static_cast<std::size_t>(k.type == NodeType::METHOD ? 1 : 0);
                        h ^= (static_cast<std::size_t>(k.ts) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
                        return h;
                    }
                };

                std::unordered_map<NodeKey, DagNode*, NodeKeyHash> rep;
                rep.reserve(input_nodes.size());
                std::unordered_map<DagNode*, DagNode*> map_old_to_new;
                map_old_to_new.reserve(input_nodes.size());

                // Create representatives.
                for (DagNode *n : input_nodes)
                {
                    NodeKey k{n->getType(), n->getOpId(), n->getTs()};
                    auto it = rep.find(k);
                    if (it == rep.end())
                    {
                        DagNode *nn = new DagNode(n->getOpId(), n->isLastChild(), n->getType());
                        nn->setTs(n->getTs());
                        nn->setActive(n->isActive());
                        rep.emplace(k, nn);
                        map_old_to_new.emplace(n, nn);
                    }
                    else
                    {
                        it->second->setLastChild(n->isLastChild());
                        it->second->setActive(it->second->isActive() || n->isActive());
                        map_old_to_new.emplace(n, it->second);
                    }
                }

                // Build union edges for traversal/expansion.
                // NOTE: do NOT build conditional successors here; they are only needed
                // for the SAT encoding on the primitive compressed DAG, and building
                // them on the full DAG is very expensive.
                for (DagNode *n : input_nodes)
                {
                    auto itn = map_old_to_new.find(n);
                    if (itn == map_old_to_new.end())
                        continue;
                    DagNode *nn = itn->second;

                    for (DagNode *s : n->getNext())
                    {
                        auto its = map_old_to_new.find(s);
                        if (its == map_old_to_new.end())
                            continue;
                        DagNode *ss = its->second;
                        if (ss == nn)
                            continue;
                        if (nn->getTs() >= ss->getTs())
                            continue;
                        nn->addNext(ss);
                    }
                }

                std::vector<DagNode*> out;
                out.reserve(rep.size());
                for (auto &kv : rep)
                    out.push_back(kv.second);

                struct Result { std::vector<DagNode*> nodes; std::size_t removed; };
                return Result{std::move(out), map_old_to_new.size() - rep.size()};
            };

            auto compressed_full = compressFullDag(ordered);
            Log::i("Full-DAG compression removed %zu nodes\n", compressed_full.removed);
            nodes = std::move(compressed_full.nodes);

            // IMPORTANT: keep the working set in topological order.
            // After timestamping, all edges are forward in time, so sorting by ts is
            // a valid topological order and is much faster than a full topo sort.
            std::sort(nodes.begin(), nodes.end(),
                      [](DagNode *a, DagNode *b) { return a->getTs() < b->getTs(); });
        }

        bool frontier_stable_now = false;
        // Frontier stabilization tracking (independent of SAT/encoding).
        if (optimized_expansion)
        {
            // Compute a cheap but robust-ish signature of the current frontier.
            // Incorporate node identity (type/op/ts) and local degrees.
            std::uint64_t sig_xor = 0;
            std::uint64_t sig_sum = 0;
            for (DagNode *n : nodes)
            {
                const std::uint64_t type_bit = (n->getType() == NodeType::METHOD) ? 1ULL : 0ULL;
                std::uint64_t h = (static_cast<std::uint64_t>(n->getOpId()) << 1) ^ type_bit;
                h ^= (static_cast<std::uint64_t>(n->getTs()) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
                h ^= (static_cast<std::uint64_t>(n->getPrev().size()) << 32) ^ static_cast<std::uint64_t>(n->getNext().size());
                sig_xor ^= h;
                sig_sum += h;
            }
            const std::uint64_t frontier_sig = sig_xor ^ (sig_sum + 0x9e3779b97f4a7c15ULL + (sig_xor << 6) + (sig_xor >> 2));

            if (nodes.size() == prev_frontier_size && frontier_sig == prev_frontier_sig)
            {
                ++stable_frontier_iters;
            }
            else
            {
                stable_frontier_iters = 0;
                prev_frontier_size = nodes.size();
                prev_frontier_sig = frontier_sig;
            }

            // Require stability twice to reduce risk of accidental collisions.
            frontier_stable_now = (stable_frontier_iters >= 2);

            // In expansion-only mode, we can stop immediately.
            if (frontier_stable_now && no_sat_encoding)
            {
                Log::i("Frontier stabilized at depth %d; stopping early in -no_sat_encoding mode.\n", current_depth);
                break;
            }
        }

        // In expansion-only mode, skip all primitive extraction/timestamping/encoding prep.
        // This is used by scripts/check_speed_expansion.py.
        if (no_sat_encoding)
            continue;

        // Optional post-pass DAG compression (ACTION nodes only).
        // Builds conditional successors; keeps only edges between provided nodes.
        auto compressDag = [&](const std::vector<DagNode*> &input_nodes) {
            struct NodeKey {
                NodeType type;
                int op_id;
                int ts;
                bool operator==(const NodeKey &o) const { return type == o.type && op_id == o.op_id && ts == o.ts; }
            };
            struct NodeKeyHash {
                std::size_t operator()(const NodeKey &k) const {
                    std::size_t h = (static_cast<std::size_t>(k.op_id) << 1) ^ static_cast<std::size_t>(k.type == NodeType::METHOD ? 1 : 0);
                    h ^= (static_cast<std::size_t>(k.ts) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
                    return h;
                }
            };
            // Work on the induced subgraph over input_nodes only.
            std::unordered_set<DagNode*> all;
            all.reserve(input_nodes.size());
            for (DagNode *n : input_nodes)
                all.insert(n);

            std::unordered_map<NodeKey, DagNode*, NodeKeyHash> rep;
            rep.reserve(all.size());
            std::unordered_map<DagNode*, DagNode*> map_old_to_new;
            map_old_to_new.reserve(all.size());

            // Create representatives.
            for (DagNode *n : all)
            {
                NodeKey k{n->getType(), n->getOpId(), n->getTs()};
                auto it = rep.find(k);
                if (it == rep.end())
                {
                    DagNode *nn = new DagNode(n->getOpId(), n->isLastChild(), n->getType());
                    nn->setActive(n->isActive());
                    nn->setTs(n->getTs());
                    rep.emplace(k, nn);
                    map_old_to_new.emplace(n, nn);
                }
                else
                {
                    it->second->setLastChild(n->isLastChild());
                    it->second->setActive(it->second->isActive() || n->isActive());
                    // same key => same timestamp
                    map_old_to_new.emplace(n, it->second);
                }
            }

            // Cache contexts to avoid rebuilding vectors repeatedly.
            std::unordered_map<DagNode*, std::vector<DagNode*>> ctx_cache;
            ctx_cache.reserve(all.size());

            // Build edges and conditional successors.
            for (DagNode *n : all)
            {
                DagNode *nn = map_old_to_new[n];

                // Incoming edges are unconditional (but filtered to stay forward in time).
                for (DagNode *p : n->getPrev())
                {
                    if (!all.count(p))
                        continue;
                    DagNode *pp = map_old_to_new[p];
                    if (pp == nn)
                        continue; // don't create self-loops when merging identical nodes
                    if (pp->getTs() >= nn->getTs())
                        continue; // keep compressed graph acyclic in timestamp order
                    pp->addNext(nn);
                }

                // Outgoing edges are conditional on the predecessor-context.
                auto it_ctx = ctx_cache.find(n);
                if (it_ctx == ctx_cache.end())
                {
                    std::unordered_set<DagNode*> prev_new;
                    for (DagNode *p : n->getPrev())
                    {
                        if (!all.count(p))
                            continue;
                        prev_new.insert(map_old_to_new.at(p));
                    }
                    ctx_cache.emplace(n, DagNode::canonicalizePrevContext(prev_new));
                    it_ctx = ctx_cache.find(n);
                }

                // Store conditional successors keyed by SINGLE predecessor.
                // (The encoding chooses exactly one incoming edge for a selected node.)
                for (DagNode *p : n->getPrev())
                {
                    if (!all.count(p))
                        continue;
                    DagNode *pp = map_old_to_new.at(p);
                    if (pp == nn)
                        continue;
                    std::vector<DagNode*> ctx{pp};
                    for (DagNode *s : n->getNext())
                    {
                        if (!all.count(s))
                            continue;
                        DagNode *ss = map_old_to_new.at(s);
                        if (ss == nn)
                            continue;
                        if (nn->getTs() >= ss->getTs())
                            continue;
                        nn->addConditionalNext(ctx, ss);
                    }
                }
            }

            std::vector<DagNode*> out;
            out.reserve(rep.size());
            for (auto &kv : rep)
                out.push_back(kv.second);

            struct Result { std::vector<DagNode*> nodes; std::size_t removed; };
            return Result{std::move(out), all.size() - rep.size()};
        };

        if (compress_dag) {
            Log::i("Number of nodes before compression: %zu\n", ordered.size());
        }

        // First extract primitive ACTION nodes as before.
        Log::i("Pruning methods from the nodes...\n");
        std::vector<DagNode*> primitive_nodes = DagNode::pruneMethods(ordered);
        Log::i("Number of primitive nodes: %d/%d\n", primitive_nodes.size(), ordered.size());

        if (!compress_dag)
            debug_write_all_paths_in_file(nodes, "nodes_" + std::to_string(current_depth) + ".txt");

        DagNode *prim_init = nullptr;
        DagNode *prim_goal = nullptr;
        if (htn_connectivity_mode)
        {
            // Ensure init/goal pseudo-actions exist in the primitive graph so we can
            // anchor a path for HTN-style instances.
            for (DagNode *n : primitive_nodes)
            {
                if (!n)
                    continue;
                if (n->getOpId() == _htn.getIdInitAction())
                    prim_init = n;
                else if (n->getOpId() == _htn.getIdGoalAction())
                    prim_goal = n;
            }
            if (!prim_init)
            {
                prim_init = new DagNode(/*op_id=*/_htn.getIdInitAction(), /*last_child=*/true, /*type=*/NodeType::ACTION);
                prim_init->setActive(true);
                primitive_nodes.push_back(prim_init);
            }
            if (!prim_goal)
            {
                prim_goal = new DagNode(/*op_id=*/_htn.getIdGoalAction(), /*last_child=*/true, /*type=*/NodeType::ACTION);
                prim_goal->setActive(true);
                primitive_nodes.push_back(prim_goal);
            }
        }

        // Treat pseudo actions as scaffolding: if we do not have any REAL primitive ACTION
        // yet (excluding init/goal/blank), there is nothing meaningful to encode.
        {
            int real_actions = 0;
            const int blank_id = _htn.getBlankAction().getId();
            const int init_id = _htn.getIdInitAction();
            const int goal_id = _htn.getIdGoalAction();
            for (DagNode *n : primitive_nodes)
            {
                if (!n || n->getType() != NodeType::ACTION)
                    continue;
                const int op = n->getOpId();
                if (op == blank_id || op == init_id || op == goal_id)
                    continue;
                real_actions++;
            }
            if (real_actions == 0)
            {
                Log::i("No real primitive actions found yet. Skipping SAT encoding.\n");
                continue;
            }
            saw_any_real_primitive = true;
        }

        // Wire init/root and leaf/goal on the PRIMITIVE graph only in HTN-connectivity mode.
        // For fact-goal instances, forcing a pseudo goal action into the sequence can
        // over-constrain the problem (goal facts must hold at that action's timestep).
        if (htn_connectivity_mode)
        {
            std::unordered_set<DagNode *> prim_set;
            prim_set.reserve(primitive_nodes.size());
            for (DagNode *n : primitive_nodes)
                prim_set.insert(n);

            // Enforce: init -> (primitive roots) and (primitive leaves) -> goal.
            // Connect init to any primitive node that currently has no primitive predecessors.
            for (DagNode *n : primitive_nodes)
            {
                if (!n)
                    continue;
                const int op_id = n->getOpId();
                if (op_id == _htn.getIdInitAction() || op_id == _htn.getIdGoalAction())
                    continue;

                bool has_prim_pred = false;
                for (DagNode *p : n->getPrev())
                {
                    if (p && prim_set.count(p))
                    {
                        has_prim_pred = true;
                        break;
                    }
                }
                if (!has_prim_pred)
                    prim_init->addNext(n);
            }

            // Connect any primitive node that has no primitive successors to goal.
            for (DagNode *n : primitive_nodes)
            {
                if (!n)
                    continue;
                const int op_id = n->getOpId();
                if (op_id == _htn.getIdInitAction() || op_id == _htn.getIdGoalAction())
                    continue;

                bool has_prim_succ = false;
                for (DagNode *s : n->getNext())
                {
                    if (s && prim_set.count(s) && s != prim_goal)
                    {
                        has_prim_succ = true;
                        break;
                    }
                }
                if (!has_prim_succ)
                    n->addNext(prim_goal);
            }
        }

        // Assign timestamps as topological "levels" (longest-path distance from sources).
        // This dramatically reduces the number of SAT time layers: time layers should
        // reflect required ordering depth of the primitive DAG, not the number of nodes.
        Log::i("Assigning timestamps to the primitive nodes...\n");
        {
            std::vector<DagNode *> prim_order = DagNode::topologicalSort(primitive_nodes);
            std::unordered_set<DagNode *> prim_set;
            prim_set.reserve(prim_order.size() * 2);
            for (DagNode *n : prim_order)
                prim_set.insert(n);

            for (DagNode *n : prim_order)
            {
                int max_prev_ts = -1;
                for (DagNode *p : n->getPrev())
                {
                    if (p && prim_set.count(p))
                        max_prev_ts = std::max(max_prev_ts, p->getTs());
                }
                n->setTs(max_prev_ts + 1);
            }

            // Keep primitive_nodes in a deterministic nondecreasing-ts order.
            std::stable_sort(prim_order.begin(), prim_order.end(),
                             [](DagNode *a, DagNode *b) { return a->getTs() < b->getTs(); });
            primitive_nodes = std::move(prim_order);
        }

        std::vector<DagNode*> nodes_for_encoding;
        if (compress_dag)
        {
            auto compressed = compressDag(primitive_nodes);
            Log::i("Compression removed %zu nodes\n", compressed.removed);
            nodes_for_encoding = std::move(compressed.nodes);
            debug_write_all_paths_in_file(nodes_for_encoding, "compressed_primitive_nodes_" + std::to_string(current_depth) + ".txt");
        }
        else
        {
            nodes_for_encoding = primitive_nodes;
        }

        // Then we encode
        if (nodes_for_encoding.empty())
        {
            Log::i("No primitive nodes found. No need to encode.\n");
            if (frontier_stable_now)
            {
                Log::i("Frontier stabilized and still no primitive nodes at depth %d; stopping early.\n", current_depth);
                break;
            }
            continue;
        }

        Log::i("Encoding the primitive nodes...\n");

        // The encoder expects nodes in nondecreasing timestamp order.
        std::sort(nodes_for_encoding.begin(), nodes_for_encoding.end(),
              [](DagNode *a, DagNode *b) { return a->getTs() < b->getTs(); });


        // (no_sat_encoding handled earlier as a fast-path)

        // Encode the primitive nodes
        _dag_enc.encode(nodes_for_encoding);

        // Then we call the SAT solver
        Log::i("Solving the SAT problem...\n");
        int result = _dag_enc.solve();

        // Is there a solution?
        bool solved = (result == 10);
        Log::i("Result of the SAT solver: %d\n", result);

        if (solved) {
            Log::i("Found a solution at depth %d.\n", current_depth);
            // Print the sequence of action of the plan
            _dag_enc.printPlanFromDagNodes(nodes_for_encoding);


            return 0; // Success
        }

        if (frontier_stable_now)
        {
            Log::i("Frontier stabilized and SAT is UNSAT at depth %d; stopping early.\n", current_depth);
            break;
        }
    }

    if (!saw_any_real_primitive)
        Log::e("LTP failed: reached depth %d with no primitive ACTION nodes; decomposition may require deeper expansion.\n", max_depth);
    else
        Log::e("LTP failed: no satisfying plan found up to depth %d.\n", max_depth);

    return 1;
}




void Planner::debug_write_all_paths_in_file(std::vector<DagNode *> &dag, std::string nameFile)
{
    std::ofstream file;
    file.open(nameFile);

    auto nodeLabel = [](DagNode *node) -> std::string {
        const std::string type_str = (node->getType() == NodeType::METHOD) ? "M" : "A";
        return type_str + "_" + std::to_string(node->getOpId()) + "%" + std::to_string(node->getId());
    };

    // Iterate over nodes in the dag
    // for (int posIdx = 0; posIdx < layer.size(); posIdx++) {
    //     Position& newPos = layer[posIdx];
    //     std::string posString = std::to_string(posIdx);
    //     for (USignature& rSig : newPos.getReductionsWithUniqueID()) {
    //         file << Names::to_SMT_string(rSig, false) << "__" << posString << " 0 0" << std::endl;
    //     }
    //     for (USignature& aSig : newPos.getActionsWithUniqueID()) {
    //         if (newPos.getActionsInPrimitiveTree().contains(aSig)) {
    //             file << Names::to_SMT_string(aSig, true) << "__" << posString << " 1 1" << std::endl;
    //         } else {
    //             file << Names::to_SMT_string(aSig, true) << "__" << posString << " 1 0" << std::endl;
    //         }
    //     }
    // }
    for (DagNode *node : dag)
    {
        bool isAction = node->getType() == NodeType::ACTION;
        bool isInPrimitiveTree = node->isActive();
        std::string nodeString = nodeLabel(node);
        // file << nodeString << " 0 0" << std::endl;
        file << nodeString << " " << (isAction ? "1" : "0") << " " << (isInPrimitiveTree ? "1" : "0") << std::endl;
    }

    file << std::endl;

    // Now, make a second pass to indicate all the nexts
    for (DagNode *node : dag)
    {
        std::string nodeString = nodeLabel(node);
        file << nodeString << " ";
        if (node->hasConditionalNext())
        {
            // Union of all conditional successors (keeps old format parsable)
            std::unordered_set<DagNode*> union_succ;
            for (const auto &kv : node->getConditionalNext())
                for (DagNode *s : kv.second)
                    union_succ.insert(s);

            for (DagNode *s : union_succ)
                file << nodeLabel(s) << " ";
        }
        else
        {
            for (DagNode *next : node->getNext())
                file << nodeLabel(next) << " ";
        }
        file << std::endl;
    }

    // Optional: detailed conditional successors
    bool any_conditional = false;
    for (DagNode *node : dag)
        any_conditional = any_conditional || node->hasConditionalNext();

    if (any_conditional)
    {
        file << std::endl;
        file << "# CONDITIONAL_SUCCESSORS\n";
        for (DagNode *node : dag)
        {
            if (!node->hasConditionalNext())
                continue;
            const std::string nodeString = nodeLabel(node);
            for (const auto &kv : node->getConditionalNext())
            {
                file << nodeString << " @(";
                const auto &ctx = kv.first;
                for (std::size_t i = 0; i < ctx.size(); ++i)
                {
                    file << nodeLabel(ctx[i]);
                    if (i + 1 < ctx.size())
                        file << ",";
                }
                file << ") ";
                for (DagNode *s : kv.second)
                    file << nodeLabel(s) << " ";
                file << "\n";
            }
        }
    }

    file.flush();
    file.close();
}
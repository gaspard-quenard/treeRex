#include "algo/planner.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <chrono>
#include <cstdio>




int Planner::findPlanLtp()
{
    // Implement the LTP-specific planning logic here

    const bool compress_dag = _htn.getParams().isNonzero("compress_dag");
    const bool no_sat_encoding = _htn.getParams().isNonzero("no_sat_encoding");
    const bool optimized_expansion = _htn.getParams().isNonzero("optimized_expansion");
    const bool profile_expansion = _htn.getParams().isNonzero("profile_expansion");
    const bool htn_connectivity_mode = _htn.getGoalState().empty();
    const int verbosity = _htn.getParams().getIntParam("v", 2);
    const bool dump_paths = (verbosity >= 5);

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

    // Buffers for full-DAG compression (reused across depths to reduce allocations).
    std::unordered_map<std::uint64_t, std::size_t> rep_index;
    std::vector<DagNode*> reps;
    std::vector<std::size_t> rep_of;
    std::vector<std::size_t> rep_indeg_full;
    std::vector<std::size_t> rep_outdeg_dup_full;
    std::vector<std::size_t> rep_offsets_dup_full;
    std::vector<std::size_t> rep_cursor_dup_full;
    std::vector<std::size_t> rep_offsets_full;
    std::vector<std::uint32_t> rep_succ_full;
    // Scratch for fast successor dedup in full compression: seen[rep] == stamp.
    std::vector<std::uint32_t> rep_seen_stamp;
    std::uint32_t rep_seen_epoch = 1;
    // Avoid repeated rehash/reserve churn across depths.
    std::size_t rep_index_reserve_hint = 0;
    std::size_t reps_reserve_hint = 0;
    rep_index.reserve(1024);
    reps.reserve(1024);
    rep_of.reserve(1024);

    // Reused buffers for building predecessor lists on-demand from forward edges.
    // (Used in compress_dag mode to avoid maintaining DagNode::_prev.)
    std::vector<std::size_t> pred_offsets;
    std::vector<std::size_t> pred_cursor;
    std::vector<DagNode*> pred_flat;
    std::vector<std::size_t> pred_indeg;

    // Reused FIFO queues for Kahn topological order (vector-backed).
    std::vector<std::size_t> kahn_q;
    std::vector<std::size_t> rep_kahn_q;

    while (!solved && current_depth < max_depth)
    {
        current_depth++;
        using Clock = std::chrono::steady_clock;
        const auto t_depth_start = Clock::now();
        auto ms_since = [&](const Clock::time_point &t0, const Clock::time_point &t1) {
            return std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
        };
        Log::i("For depth %d\n", current_depth);

        Log::i("  Expanding nodes...\n");

        const auto t_expand_start = Clock::now();

        // In compress_dag mode, METHOD expansion needs predecessor information,
        // but we intentionally do not maintain DagNode::_prev for performance.
        // Build a reverse CSR (successor -> predecessors) once per depth from
        // forward edges.
        if (compress_dag)
        {
            const std::size_t n = nodes.size();
            for (std::size_t i = 0; i < n; ++i)
                nodes[i]->setTmpIndex(static_cast<int>(i));

            // Ensure outgoing edges only point inside this induced set.
            for (std::size_t i = 0; i < n; ++i)
                nodes[i]->compactNextToInduced(nodes);

            pred_indeg.assign(n, 0);
            for (std::size_t ui = 0; ui < n; ++ui)
            {
                DagNode *u = nodes[ui];
                for (DagNode *v : u->getNext())
                {
                    const int vi_i = v ? v->getTmpIndex() : -1;
                    if (vi_i < 0 || static_cast<std::size_t>(vi_i) >= n)
                        continue;
                    if (nodes[static_cast<std::size_t>(vi_i)] != v)
                        continue;
                    ++pred_indeg[static_cast<std::size_t>(vi_i)];
                }
            }

            pred_offsets.assign(n + 1, 0);
            for (std::size_t i = 0; i < n; ++i)
                pred_offsets[i + 1] = pred_offsets[i] + pred_indeg[i];

            pred_flat.resize(pred_offsets[n]);
            pred_cursor = pred_offsets;

            for (std::size_t ui = 0; ui < n; ++ui)
            {
                DagNode *u = nodes[ui];
                for (DagNode *v : u->getNext())
                {
                    const int vi_i = v ? v->getTmpIndex() : -1;
                    if (vi_i < 0 || static_cast<std::size_t>(vi_i) >= n)
                        continue;
                    const std::size_t vi = static_cast<std::size_t>(vi_i);
                    if (nodes[vi] != v)
                        continue;
                    const std::size_t pos = pred_cursor[vi]++;
                    if (pos < pred_flat.size())
                        pred_flat[pos] = u;
                }
            }
        }

        // Expand exactly like before (no compression during expansion)
        std::vector<DagNode *> new_dag_nodes;
        new_dag_nodes.reserve(nodes.size() * 2);
        std::vector<DagNode *> expanded_methods_to_free;
        expanded_methods_to_free.reserve(nodes.size() / 2);
        for (DagNode *node : nodes)
        {
            if (node->isMethod())
            {
                // In compress_dag mode, expanding with detach/erase is expensive.
                // We keep old edges around and rebuild a clean graph after the
                // full-DAG compression.
                const bool detach_and_rewire = !compress_dag;
                std::vector<DagNode*> children;
                if (compress_dag)
                {
                    const std::size_t mi = static_cast<std::size_t>(node->getTmpIndex());
                    DagNode *const *preds = pred_flat.empty() ? nullptr : (pred_flat.data() + pred_offsets[mi]);
                    const std::size_t n_preds = pred_offsets[mi + 1] - pred_offsets[mi];
                    children = node->expandWithPreds(_htn, preds, n_preds, /*detach_and_rewire=*/false);
                }
                else
                {
                    children = node->expand(_htn, detach_and_rewire);
                }
                for (DagNode *child : children)
                    new_dag_nodes.push_back(child);

                // The METHOD node is replaced by its children in the frontier.
                // Keep it alive until the end of this pass because later METHOD
                // expansions may need its last-children frontier.
                expanded_methods_to_free.push_back(node);
            }
            else
            {
                new_dag_nodes.push_back(node);
            }
        }

        // NOTE: When compress_dag=1 we must NOT delete expanded METHOD nodes yet.
        // Other nodes may still reference them in predecessor/successor vectors.
        // We will delete them only after we rebuild a clean compressed graph.
        if (!compress_dag)
        {
            for (DagNode *m : expanded_methods_to_free)
                delete m;
            expanded_methods_to_free.clear();
        }
        const std::size_t expanded_nodes_count = new_dag_nodes.size();
        nodes = std::move(new_dag_nodes);

        const auto t_expand_end = Clock::now();

        Log::i("Number of nodes after expansion: %zu\n", nodes.size());


        // Now, get a topological sort of the nodes
        Log::i("Topological sort of the nodes...\n");
        std::vector<DagNode*> ordered;
        ordered.reserve(nodes.size());

        const auto t_order_start = Clock::now();

        // If compress_dag is enabled, we will assign timestamps while performing a
        // topological sort. This avoids a second full pass over predecessor sets.
        if (compress_dag)
        {
            const std::size_t n_nodes = nodes.size();

            for (std::size_t i = 0; i < n_nodes; ++i)
                nodes[i]->setTmpIndex(static_cast<int>(i));

            for (std::size_t i = 0; i < n_nodes; ++i)
            {
                DagNode *n = nodes[i];
                const int ti = n->getTmpIndex();
                if (ti < 0 || static_cast<std::size_t>(ti) != i || nodes[static_cast<std::size_t>(ti)] != n)
                    throw std::logic_error("Graph contains duplicate node pointers; topological sort impossible");
            }

            // One-time edge compaction: remove outgoing edges that point outside the
            // induced set (e.g., towards expanded METHOD nodes that are no longer in
            // the frontier). This dramatically reduces the cost of subsequent passes.
            for (std::size_t i = 0; i < n_nodes; ++i)
                nodes[i]->compactNextToInduced(nodes);

            std::vector<std::size_t> indeg(n_nodes, 0);
            for (std::size_t i = 0; i < n_nodes; ++i)
            {
                DagNode *u = nodes[i];
                u->setTs(0);
                for (DagNode *v : u->getNext())
                    ++indeg[static_cast<std::size_t>(v->getTmpIndex())];
            }

            // Use a vector-backed FIFO queue to avoid std::queue/deque overhead.
            // Reuse the buffer across iterations to avoid reallocations.
            kahn_q.clear();
            if (kahn_q.capacity() < n_nodes)
                kahn_q.reserve(n_nodes);
            for (std::size_t i = 0; i < n_nodes; ++i)
                if (indeg[i] == 0)
                    kahn_q.push_back(i);

            std::size_t q_head = 0;
            while (q_head < kahn_q.size())
            {
                const std::size_t ui = kahn_q[q_head++];
                DagNode *u = nodes[ui];
                ordered.push_back(u);
                const int next_ts = u->getTs() + 1;
                for (DagNode *v : u->getNext())
                {
                    const std::size_t vi = static_cast<std::size_t>(v->getTmpIndex());
                    if (v->getTs() < next_ts)
                        v->setTs(next_ts);
                    if (--indeg[vi] == 0)
                        kahn_q.push_back(vi);
                }
            }

            if (ordered.size() != n_nodes)
                throw std::logic_error("Graph contains a cycle; topological sort impossible");
        }
        else
        {
            ordered = DagNode::topologicalSort(nodes);
        }

        const auto t_order_end = Clock::now();

        // Keep the working set in topological order even without compression.
        // DagNode::expand() assumes METHOD predecessors are expanded before dependent ACTIONs.
        nodes = ordered;

        // If compress_dag is enabled, compress the full expanded DAG and keep working on it.
        // This drastically reduces the number of nodes carried to the next iteration.
        if (compress_dag)
        {

            const auto t_compress_start = Clock::now();

            // Full-DAG compression is part of the core LTP pipeline and must behave
            // identically regardless of SAT encoding/solving.
            const bool do_full_compress = true;

            // Fast full-DAG compression.
            // Key idea: avoid a global sort/unique over all rep edges.
            // Instead, build successor lists per representative, sort/unique within
            // each list, then insert each rep-edge once.
            auto compressFullDag = [&](const std::vector<DagNode*> &input_nodes) {
                const std::size_t n_in = input_nodes.size();
                for (std::size_t i = 0; i < n_in; ++i)
                    input_nodes[i]->setTmpIndex(static_cast<int>(i));

                auto packKey = [](DagNode *n) -> std::uint64_t {
                    const std::uint64_t type_bit = (n->getType() == NodeType::METHOD) ? 1ULL : 0ULL;
                    const std::uint64_t op = static_cast<std::uint64_t>(static_cast<std::uint32_t>(n->getOpId()));
                    const std::uint64_t ts = static_cast<std::uint64_t>(static_cast<std::uint32_t>(n->getTs()));
                    // [op:32][ts:31][type:1]
                    return (op << 32) | (ts << 1) | type_bit;
                };

                rep_index.clear();
                reps.clear();
                rep_of.assign(n_in, 0);

                // Create representatives (group by packed key) using a hash map.
                // Reserve only when n grows, to avoid repeated expensive rehashes.
                if (n_in > rep_index_reserve_hint)
                {
                    rep_index.reserve(n_in);
                    rep_index_reserve_hint = n_in;
                }
                if (n_in > reps_reserve_hint)
                {
                    reps.reserve(n_in);
                    reps_reserve_hint = n_in;
                }

                for (std::size_t i = 0; i < n_in; ++i)
                {
                    DagNode *n = input_nodes[i];
                    const std::uint64_t k = packKey(n);
                    auto it = rep_index.find(k);
                    if (it == rep_index.end())
                    {
                        DagNode *nn = new DagNode(n->getOpId(), n->isLastChild(), n->getType());
                        nn->setTs(n->getTs());
                        nn->setActive(n->isActive());
                        const std::size_t idx = reps.size();
                        rep_index.emplace(k, idx);
                        reps.push_back(nn);
                        rep_of[i] = idx;
                    }
                    else
                    {
                        DagNode *nn = reps[it->second];
                        nn->setLastChild(n->isLastChild());
                        nn->setActive(nn->isActive() || n->isActive());
                        rep_of[i] = it->second;
                    }
                }

                const std::size_t n_rep = reps.size();


                // NOTE: do NOT build conditional successors here; they are only needed
                // for the SAT encoding on the primitive compressed DAG.

                // Pass 1: count successor entries per rep (with duplicates).
                rep_outdeg_dup_full.assign(n_rep, 0);
                for (std::size_t i = 0; i < n_in; ++i)
                {
                    DagNode *n = input_nodes[i];
                    const std::size_t ui = rep_of[i];
                    for (DagNode *s : n->getNext())
                    {
                        const std::size_t si = static_cast<std::size_t>(s->getTmpIndex());
                        const std::size_t vi = rep_of[si];
                        if (ui == vi)
                            continue;
                        ++rep_outdeg_dup_full[ui];
                    }
                }

                // Build offsets for the duplicate successor list.
                rep_offsets_dup_full.assign(n_rep + 1, 0);
                for (std::size_t i = 0; i < n_rep; ++i)
                    rep_offsets_dup_full[i + 1] = rep_offsets_dup_full[i] + rep_outdeg_dup_full[i];

                rep_succ_full.resize(rep_offsets_dup_full[n_rep]);
                rep_cursor_dup_full = rep_offsets_dup_full;

                // Pass 2: fill duplicate successor list.
                for (std::size_t i = 0; i < n_in; ++i)
                {
                    DagNode *n = input_nodes[i];
                    const std::size_t ui = rep_of[i];
                    for (DagNode *s : n->getNext())
                    {
                        const std::size_t si = static_cast<std::size_t>(s->getTmpIndex());
                        const std::size_t vi = rep_of[si];
                        if (ui == vi)
                            continue;
                        const std::size_t pos = rep_cursor_dup_full[ui]++;
                        rep_succ_full[pos] = static_cast<std::uint32_t>(vi);
                    }
                }

                // Deduplicate per rep with a stamp array (O(E)) and compute indegrees on unique edges.
                rep_indeg_full.assign(n_rep, 0);
                rep_offsets_full.assign(n_rep + 1, 0);

                if (rep_seen_stamp.size() < n_rep)
                {
                    rep_seen_stamp.assign(n_rep, 0);
                    rep_seen_epoch = 1;
                }
                if (rep_seen_epoch == 0)
                {
                    std::fill(rep_seen_stamp.begin(), rep_seen_stamp.end(), 0);
                    rep_seen_epoch = 1;
                }

                std::size_t write_pos = 0;
                for (std::size_t ui = 0; ui < n_rep; ++ui)
                {
                    rep_offsets_full[ui] = write_pos;
                    const std::size_t b = rep_offsets_dup_full[ui];
                    const std::size_t e = rep_offsets_dup_full[ui + 1];

                    const std::uint32_t stamp = rep_seen_epoch++;
                    if (rep_seen_epoch == 0)
                    {
                        std::fill(rep_seen_stamp.begin(), rep_seen_stamp.end(), 0);
                        rep_seen_epoch = 1;
                    }

                    for (std::size_t p = b; p < e; ++p)
                    {
                        const std::uint32_t v32 = rep_succ_full[p];
                        const std::size_t vi = static_cast<std::size_t>(v32);
                        if (rep_seen_stamp[vi] == stamp)
                            continue;
                        rep_seen_stamp[vi] = stamp;
                        rep_succ_full[write_pos++] = v32;
                        ++rep_indeg_full[vi];
                    }

                    rep_offsets_full[ui + 1] = write_pos;
                }
                rep_succ_full.resize(write_pos);

                // Reserve adjacency vectors.
                for (std::size_t i = 0; i < n_rep; ++i)
                {
                    const std::size_t outdeg_u = rep_offsets_full[i + 1] - rep_offsets_full[i];
                    reps[i]->reserveNext(outdeg_u);
                    reps[i]->setTs(0);
                }

                // Insert unique rep-edges into the compressed DAG.
                for (std::size_t ui = 0; ui < n_rep; ++ui)
                {
                    for (std::size_t p = rep_offsets_full[ui]; p < rep_offsets_full[ui + 1]; ++p)
                    {
                        const std::size_t vi = static_cast<std::size_t>(rep_succ_full[p]);
                        reps[ui]->addNextNoBacklink(reps[vi]);
                    }
                }

                // Kahn order + longest-path timestamping on the compressed graph.
                // Vector-backed FIFO queue: faster than std::queue/deque here.
                // Reuse the buffer across depths to avoid reallocations.
                rep_kahn_q.clear();
                if (rep_kahn_q.capacity() < n_rep)
                    rep_kahn_q.reserve(n_rep);
                for (std::size_t i = 0; i < n_rep; ++i)
                    if (rep_indeg_full[i] == 0)
                        rep_kahn_q.push_back(i);

                std::vector<DagNode*> out;
                out.reserve(n_rep);

                std::size_t q_head = 0;
                while (q_head < rep_kahn_q.size())
                {
                    const std::size_t ui = rep_kahn_q[q_head++];
                    DagNode *u = reps[ui];
                    out.push_back(u);

                    const int next_ts = u->getTs() + 1;
                    for (std::size_t p = rep_offsets_full[ui]; p < rep_offsets_full[ui + 1]; ++p)
                    {
                        const std::size_t vi = static_cast<std::size_t>(rep_succ_full[p]);
                        DagNode *v = reps[vi];
                        if (v->getTs() < next_ts)
                            v->setTs(next_ts);
                        if (--rep_indeg_full[vi] == 0)
                            rep_kahn_q.push_back(vi);
                    }
                }

                if (out.size() != n_rep)
                    throw std::logic_error("Graph contains a cycle after compression; topological order impossible");

                struct Result { std::vector<DagNode*> nodes; std::size_t removed; };
                return Result{std::move(out), n_in - rep_index.size()};
            };

            if (do_full_compress)
            {
                auto compressed_full = compressFullDag(ordered);
                Log::i("Full-DAG compression removed %zu nodes\n", compressed_full.removed);
                nodes = std::move(compressed_full.nodes);

                // We now have a clean graph made of freshly allocated rep nodes.
                // It is safe (and good for memory) to delete the previous DAG.
                for (DagNode *n : ordered)
                    delete n;
                for (DagNode *m : expanded_methods_to_free)
                    delete m;
                expanded_methods_to_free.clear();
                ordered.clear();
            }

            const auto t_compress_end = Clock::now();

            if (profile_expansion)
            {
                const auto t_depth_end = Clock::now();
                fprintf(stderr,
                        "[profile_expansion] depth=%d expanded_nodes=%zu ordered_nodes=%zu frontier_nodes=%zu | expand=%.3fms order=%.3fms compress=%.3fms total=%.3fms\n",
                        current_depth,
                        expanded_nodes_count,
                        ordered.size(),
                        nodes.size(),
                        ms_since(t_expand_start, t_expand_end),
                        ms_since(t_order_start, t_order_end),
                        ms_since(t_compress_start, t_compress_end),
                        ms_since(t_depth_start, t_depth_end));
            }
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
                h ^= static_cast<std::uint64_t>(n->getNext().size());
                sig_xor ^= h;
                sig_sum += h;
            }
            const std::uint64_t frontier_sig = sig_xor ^ (sig_sum + 0x9e3779b97f4a7c15ULL + (sig_xor << 6) + (sig_xor >> 2));
            const std::size_t frontier_size = nodes.size();

            if (frontier_size == prev_frontier_size && frontier_sig == prev_frontier_sig)
                ++stable_frontier_iters;
            else
            {
                stable_frontier_iters = 0;
                prev_frontier_size = frontier_size;
                prev_frontier_sig = frontier_sig;
            }

            // Require stability twice to reduce risk of accidental collisions.
            frontier_stable_now = (stable_frontier_iters >= 2);
        }

        // Optional post-pass DAG compression (ACTION nodes only).
        // Builds conditional successors; keeps only edges between provided nodes.
        auto compressDag = [&](const std::vector<DagNode*> &input_nodes) {
            const std::size_t n_in = input_nodes.size();
            for (std::size_t i = 0; i < n_in; ++i)
                input_nodes[i]->setTmpIndex(static_cast<int>(i));

            auto in_induced = [&](DagNode *n) -> int {
                if (!n)
                    return -1;
                const int idx = n->getTmpIndex();
                if (idx < 0 || static_cast<std::size_t>(idx) >= n_in)
                    return -1;
                if (input_nodes[static_cast<std::size_t>(idx)] != n)
                    return -1;
                return idx;
            };

            auto packKey = [](DagNode *n) -> std::uint64_t {
                const std::uint64_t type_bit = (n->getType() == NodeType::METHOD) ? 1ULL : 0ULL;
                const std::uint64_t op = static_cast<std::uint64_t>(static_cast<std::uint32_t>(n->getOpId()));
                const std::uint64_t ts = static_cast<std::uint64_t>(static_cast<std::uint32_t>(n->getTs()));
                return (op << 32) | (ts << 1) | type_bit;
            };

            std::unordered_map<std::uint64_t, std::size_t> rep_index;
            rep_index.reserve(n_in);
            std::vector<DagNode*> reps;
            reps.reserve(n_in);
            std::vector<std::size_t> rep_of(n_in, 0);
            std::vector<std::uint32_t> rep_count;

            // Create representatives.
            for (std::size_t i = 0; i < n_in; ++i)
            {
                DagNode *n = input_nodes[i];
                const std::uint64_t k = packKey(n);
                auto it = rep_index.find(k);
                if (it == rep_index.end())
                {
                    DagNode *nn = new DagNode(n->getOpId(), n->isLastChild(), n->getType());
                    nn->setActive(n->isActive());
                    nn->setTs(n->getTs());
                    const std::size_t idx = reps.size();
                    rep_index.emplace(k, idx);
                    reps.push_back(nn);
                    rep_of[i] = idx;
                }
                else
                {
                    DagNode *nn = reps[it->second];
                    nn->setLastChild(n->isLastChild());
                    nn->setActive(nn->isActive() || n->isActive());
                    rep_of[i] = it->second;
                }
            }

            rep_count.assign(reps.size(), 0);
            for (std::size_t i = 0; i < n_in; ++i)
                ++rep_count[rep_of[i]];

            // Build unconditional edges (deduped at rep-level), filtered to stay forward in time.
            std::vector<std::uint64_t> rep_edges;
            rep_edges.reserve(n_in * 2);
            for (std::size_t ui_in = 0; ui_in < n_in; ++ui_in)
            {
                DagNode *u = input_nodes[ui_in];
                const std::size_t ui = rep_of[ui_in];
                DagNode *uu = reps[ui];
                for (DagNode *s : u->getNext())
                {
                    const int si_i = in_induced(s);
                    if (si_i < 0)
                        continue;
                    const std::size_t vi_in = static_cast<std::size_t>(si_i);
                    const std::size_t vi = rep_of[vi_in];
                    if (ui == vi)
                        continue;
                    DagNode *vv = reps[vi];
                    if (uu->getTs() >= vv->getTs())
                        continue;
                    rep_edges.push_back((static_cast<std::uint64_t>(static_cast<std::uint32_t>(ui)) << 32)
                                        | static_cast<std::uint64_t>(static_cast<std::uint32_t>(vi)));
                }
            }
            std::sort(rep_edges.begin(), rep_edges.end());
            rep_edges.erase(std::unique(rep_edges.begin(), rep_edges.end()), rep_edges.end());
            for (const std::uint64_t e : rep_edges)
            {
                const std::size_t ui = static_cast<std::size_t>(static_cast<std::uint32_t>(e >> 32));
                const std::size_t vi = static_cast<std::size_t>(static_cast<std::uint32_t>(e & 0xffffffffULL));
                reps[ui]->addNext(reps[vi]);
            }

            // Reverse adjacency for predecessor contexts (by original-node index).
            std::vector<std::size_t> rev_deg(n_in, 0);
            for (std::size_t ui_in = 0; ui_in < n_in; ++ui_in)
            {
                DagNode *u = input_nodes[ui_in];
                for (DagNode *s : u->getNext())
                {
                    const int si_i = in_induced(s);
                    if (si_i < 0)
                        continue;
                    ++rev_deg[static_cast<std::size_t>(si_i)];
                }
            }
            std::vector<std::size_t> rev_off(n_in + 1, 0);
            for (std::size_t i = 0; i < n_in; ++i)
                rev_off[i + 1] = rev_off[i] + rev_deg[i];
            std::vector<std::uint32_t> rev_pred;
            rev_pred.resize(rev_off[n_in]);
            std::vector<std::size_t> rev_cur = rev_off;
            for (std::size_t ui_in = 0; ui_in < n_in; ++ui_in)
            {
                DagNode *u = input_nodes[ui_in];
                for (DagNode *s : u->getNext())
                {
                    const int si_i = in_induced(s);
                    if (si_i < 0)
                        continue;
                    const std::size_t vi_in = static_cast<std::size_t>(si_i);
                    const std::size_t pos = rev_cur[vi_in]++;
                    if (pos < rev_pred.size())
                        rev_pred[pos] = static_cast<std::uint32_t>(ui_in);
                }
            }

            // Build conditional successors keyed by SINGLE predecessor.
            // (The encoding chooses exactly one incoming edge for a selected node.)
            const std::size_t n_rep = reps.size();
            std::vector<int> seen_succ(n_rep, -1);
            std::vector<int> seen_pred(n_rep, -1);
            std::vector<std::uint32_t> uniq_succ;
            std::vector<std::uint32_t> uniq_pred;
            uniq_succ.reserve(64);
            uniq_pred.reserve(64);
            int stamp = 0;

            for (std::size_t i = 0; i < n_in; ++i)
            {
                DagNode *n = input_nodes[i];
                const std::size_t nn_id = rep_of[i];
                // No merge => conditional successors are redundant.
                if (rep_count[nn_id] <= 1)
                    continue;
                DagNode *nn = reps[nn_id];

                ++stamp;
                uniq_succ.clear();
                uniq_pred.clear();

                // Unique successor representatives of this original node.
                for (DagNode *s : n->getNext())
                {
                    const int si_i = in_induced(s);
                    if (si_i < 0)
                        continue;
                    const std::size_t si = static_cast<std::size_t>(si_i);
                    const std::size_t rid = rep_of[si];
                    if (rid >= n_rep)
                        continue;
                    DagNode *ss = reps[rid];
                    if (ss == nn)
                        continue;
                    if (nn->getTs() >= ss->getTs())
                        continue;
                    if (seen_succ[rid] == stamp)
                        continue;
                    seen_succ[rid] = stamp;
                    uniq_succ.push_back(static_cast<std::uint32_t>(rid));
                }

                if (uniq_succ.empty())
                    continue;

                // Unique predecessor representatives of this original node.
                for (std::size_t p = rev_off[i]; p < rev_off[i + 1]; ++p)
                {
                    const std::size_t pi_in = static_cast<std::size_t>(rev_pred[p]);
                    const std::size_t rid = rep_of[pi_in];
                    if (rid >= n_rep)
                        continue;
                    DagNode *pp = reps[rid];
                    if (pp == nn)
                        continue;
                    if (seen_pred[rid] == stamp)
                        continue;
                    seen_pred[rid] = stamp;
                    uniq_pred.push_back(static_cast<std::uint32_t>(rid));
                }

                for (const std::uint32_t pid : uniq_pred)
                {
                    DagNode *pp = reps[static_cast<std::size_t>(pid)];
                    std::unordered_set<DagNode*> &cond_succs = nn->getOrCreateConditionalNextSetSingle(pp);
                    cond_succs.reserve(cond_succs.size() + uniq_succ.size());
                    for (const std::uint32_t sid : uniq_succ)
                        cond_succs.insert(reps[static_cast<std::size_t>(sid)]);
                }
            }

            struct Result { std::vector<DagNode*> nodes; std::size_t removed; };
            return Result{std::move(reps), n_in - rep_index.size()};
        };

        // First extract primitive ACTION nodes as before.
        Log::i("Pruning methods from the nodes...\n");
        const std::size_t dag_size_for_pruning = nodes.size();
        std::vector<DagNode*> primitive_nodes = DagNode::pruneMethods(nodes);
        Log::i("Number of primitive nodes: %d/%d\n", primitive_nodes.size(), dag_size_for_pruning);

        if (!compress_dag && dump_paths)
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

        // SAT encoder black box requirement:
        // Keep using the primitive nodes extracted from the main DAG.
        // (Aux join nodes are inserted only for METHOD children during expansion,
        // so primitive ACTION ordering edges remain explicit and unchanged here.)
        std::vector<DagNode*> primitive_nodes_for_encoding = primitive_nodes;

        // Treat pseudo actions as scaffolding: if we do not have any REAL primitive ACTION
        // yet (excluding init/goal/blank), there is nothing meaningful to encode.
        {
            int real_actions = 0;
            const int blank_id = _htn.getBlankAction().getId();
            const int init_id = _htn.getIdInitAction();
            const int goal_id = _htn.getIdGoalAction();
            for (DagNode *n : primitive_nodes_for_encoding)
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
            prim_set.reserve(primitive_nodes_for_encoding.size());
            for (DagNode *n : primitive_nodes_for_encoding)
                prim_set.insert(n);

            // Compute primitive indegree using forward edges only.
            std::unordered_map<DagNode*, int> prim_indeg;
            prim_indeg.reserve(primitive_nodes.size());
            for (DagNode *n : primitive_nodes)
                prim_indeg.emplace(n, 0);
            for (DagNode *u : primitive_nodes)
            {
                if (!u)
                    continue;
                for (DagNode *v : u->getNext())
                {
                    if (!v)
                        continue;
                    auto it = prim_indeg.find(v);
                    if (it != prim_indeg.end())
                        ++it->second;
                }
            }

            // Enforce: init -> (primitive roots) and (primitive leaves) -> goal.
            // Connect init to any primitive node that currently has no primitive predecessors.
            for (DagNode *n : primitive_nodes_for_encoding)
            {
                if (!n)
                    continue;
                const int op_id = n->getOpId();
                if (op_id == _htn.getIdInitAction() || op_id == _htn.getIdGoalAction())
                    continue;

                const auto it = prim_indeg.find(n);
                const int indeg = (it == prim_indeg.end()) ? 0 : it->second;
                if (indeg == 0)
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
            // primitive_nodes is already a subsequence of `ordered` (topological order),
            // so we can assign timestamps without re-sorting.
            std::vector<DagNode *> prim_order = primitive_nodes_for_encoding;
            const std::size_t n_prim = prim_order.size();
            for (std::size_t i = 0; i < n_prim; ++i)
                prim_order[i]->setTmpIndex(static_cast<int>(i));

            auto in_prim = [&](DagNode *n) -> bool {
                if (!n)
                    return false;
                const int idx = n->getTmpIndex();
                if (idx < 0 || static_cast<std::size_t>(idx) >= n_prim)
                    return false;
                return prim_order[static_cast<std::size_t>(idx)] == n;
            };

            // Longest-path DP in topological order using forward edges only.
            for (DagNode *n : prim_order)
                n->setTs(0);
            for (DagNode *u : prim_order)
            {
                const int next_ts = u->getTs() + 1;
                for (DagNode *v : u->getNext())
                {
                    if (!in_prim(v))
                        continue;
                    if (v->getTs() < next_ts)
                        v->setTs(next_ts);
                }
            }

            // Keep primitive_nodes in a deterministic nondecreasing-ts order.
            std::stable_sort(prim_order.begin(), prim_order.end(),
                             [](DagNode *a, DagNode *b) { return a->getTs() < b->getTs(); });
            primitive_nodes_for_encoding = std::move(prim_order);
        }

        std::vector<DagNode*> nodes_for_encoding;
        if (compress_dag)
        {
            auto compressed = compressDag(primitive_nodes_for_encoding);
            Log::i("Compression removed %zu nodes\n", compressed.removed);
            nodes_for_encoding = std::move(compressed.nodes);
            if (dump_paths)
                debug_write_all_paths_in_file(nodes_for_encoding, "compressed_primitive_nodes_" + std::to_string(current_depth) + ".txt");
        }
        else
        {
            nodes_for_encoding = primitive_nodes_for_encoding;
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

        // In -no_sat_encoding mode we still run the exact same pipeline up to the
        // SAT input DAG construction, but we skip calling the encoder/solver.
        if (no_sat_encoding)
            continue;

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
                DagNode *pred = kv.first;
                if (pred)
                    file << nodeLabel(pred);
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
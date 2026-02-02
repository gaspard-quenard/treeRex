#include "data/dag_node.h"
#include "util/log.h"
#include "util/names.h"

#include <cstdint>

int DagNode::counter_id = 0; // Initialize the static unique_id counter

void DagNode::compactNextToInduced(const std::vector<DagNode*> &induced)
{
    const std::size_t n = induced.size();
    std::size_t w = 0;
    for (DagNode *v : _next)
    {
        if (!v)
            continue;
        const int idx = v->getTmpIndex();
        if (idx < 0 || static_cast<std::size_t>(idx) >= n)
            continue;
        if (induced[static_cast<std::size_t>(idx)] != v)
            continue;
        _next[w++] = v;
    }
    _next.resize(w);
}

void DagNode::rebuildPrevFromNext(const std::vector<DagNode*> &nodes)
{
    const std::size_t n = nodes.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        DagNode *u = nodes[i];
        if (u)
            u->clearPrevShallow();
    }

    for (std::size_t ui = 0; ui < n; ++ui)
    {
        DagNode *u = nodes[ui];
        if (!u)
            continue;
        for (DagNode *v : u->getNext())
        {
            if (!v)
                continue;
            const int vi = v->getTmpIndex();
            if (vi < 0 || static_cast<std::size_t>(vi) >= n)
                continue;
            if (nodes[static_cast<std::size_t>(vi)] != v)
                continue;
            v->addPrev(u);
        }
    }
}

void DagNode::rebuildPrevForMethodsFromNext(const std::vector<DagNode*> &nodes)
{
    const std::size_t n = nodes.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        DagNode *v = nodes[i];
        if (v && v->isMethod())
            v->clearPrevShallow();
    }

    for (std::size_t ui = 0; ui < n; ++ui)
    {
        DagNode *u = nodes[ui];
        if (!u)
            continue;
        for (DagNode *v : u->getNext())
        {
            if (!v || !v->isMethod())
                continue;
            v->addPrev(u);
        }
    }
}

namespace {
struct ChildSpec {
    int op_id;
    NodeType type;
    bool last_child;
};

struct MethodExpansionTemplate {
    // layers[j] are the nodes created for subtask j.
    std::vector<std::vector<ChildSpec>> layers;
};

static const MethodExpansionTemplate &get_or_build_template(HtnInstance &htn, int method_id)
{
    // One Treerex process handles a single HTN instance in practice, so a
    // process-wide cache is safe and avoids plumbing this through HtnInstance.
    static std::unordered_map<int, MethodExpansionTemplate> cache;
    auto it = cache.find(method_id);
    if (it != cache.end())
        return it->second;

    MethodExpansionTemplate tpl;
    const Method &method = htn.getMethodById(method_id);
    const std::size_t n_subtasks = method.getSubtasksIdx().size();
    tpl.layers.resize(n_subtasks);
    const std::size_t last_j = (n_subtasks == 0) ? 0 : (n_subtasks - 1);

    for (std::size_t j = 0; j < n_subtasks; ++j)
    {
        const int subtask_idx = method.getSubtasksIdx()[j];
        const bool is_last_child = (j == last_j);

        if (htn.isAbstractTask(subtask_idx))
        {
            const AbstractTask &task = htn.getAbstractTaskById(subtask_idx);
            auto &layer = tpl.layers[j];
            layer.reserve(task.getDecompositionMethodIdx().size());
            for (int sub_method_idx : task.getDecompositionMethodIdx())
                layer.push_back(ChildSpec{sub_method_idx, NodeType::METHOD, is_last_child});
        }
        else
        {
            tpl.layers[j].push_back(ChildSpec{subtask_idx, NodeType::ACTION, is_last_child});
        }
    }

    auto [ins_it, _] = cache.emplace(method_id, std::move(tpl));
    return ins_it->second;
}
} // namespace

std::vector<DagNode *> DagNode::expand(HtnInstance &htn, bool detach_and_rewire)
{

    // Fast path for ACTION nodes: once an action has no METHOD predecessors left
    // and it already has no outgoing edges (as per the expansion procedure),
    // further expand() calls are a no-op.
    if (!isMethod())
    {
        bool has_method_prev = false;
        for (DagNode *p : _prev)
        {
            if (p && p->getType() == NodeType::METHOD)
            {
                has_method_prev = true;
                break;
            }
        }

        if (!has_method_prev && _next.empty())
            return std::vector<DagNode *>();
    }

    // Rewrite predecessors: replace METHOD predecessors by their expanded last-children.
    // Use vector + sort/unique to avoid unordered_set hashing/allocations here.
    std::vector<DagNode *> prev_layer;
    prev_layer.reserve(_prev.size());
    for (DagNode *prev_node : _prev)
    {
        if (!prev_node)
            continue;
        if (prev_node->getType() == NodeType::METHOD)
        {
            const std::vector<DagNode*> &kids = prev_node->getLastChildren();
            if (!kids.empty())
            {
                for (DagNode *child : kids)
                    if (child)
                        prev_layer.push_back(child);
            }
            else
            {
                // Predecessor METHOD not expanded yet: keep it as a predecessor.
                prev_layer.push_back(prev_node);
            }
        }
        else
        {
            prev_layer.push_back(prev_node);
        }
    }
    std::sort(prev_layer.begin(), prev_layer.end());
    prev_layer.erase(std::unique(prev_layer.begin(), prev_layer.end()), prev_layer.end());

    if (!isMethod())
    {
        // ACTION nodes are kept across depth iterations.
        // With successor rewiring performed when expanding METHOD nodes, ACTION nodes
        // no longer need to rewrite their predecessor sets each depth.
        return std::vector<DagNode *>();
    }

    // Defensive: if expand() is (unexpectedly) called twice on the same METHOD node,
    // don't keep accumulating last-children.
    _last_children.clear();

    // We return all nodes created by this expansion so the planner can add them
    // to the next frontier. This is separate from _last_children.
    std::vector<DagNode *> created_nodes;

    // Template-based expansion: the internal structure of a METHOD expansion
    // depends only on the method definition (subtasks + abstract-task choices).
    // Timestamps depend only on the max timestamp of the incoming frontier.
    const MethodExpansionTemplate &tpl = get_or_build_template(htn, op_id);

    int max_prev_ts = -1;
    for (DagNode *p : prev_layer)
        if (p)
            max_prev_ts = std::max(max_prev_ts, p->getTs());

    const int base_ts = max_prev_ts + 1;
    for (std::size_t layer_idx = 0; layer_idx < tpl.layers.size(); ++layer_idx)
    {
        const auto &layer_specs = tpl.layers[layer_idx];
        std::vector<DagNode *> cur_layer;
        cur_layer.reserve(layer_specs.size());

        const int layer_ts = base_ts + static_cast<int>(layer_idx);
        for (const ChildSpec &spec : layer_specs)
        {
            DagNode *child = new DagNode(spec.op_id, spec.last_child, spec.type);
            child->setTs(layer_ts);
            // Each child in this layer receives an edge from every node in prev_layer.
            if (detach_and_rewire)
                child->reservePrev(prev_layer.size());
            cur_layer.push_back(child);
            created_nodes.push_back(child);
        }

        if (detach_and_rewire)
        {
            // Exact behavior: complete bipartite between previous frontier and this layer.
            for (DagNode *prev_node : prev_layer)
            {
                prev_node->reserveNextAdditional(cur_layer.size());
                for (DagNode *child : cur_layer)
                    prev_node->addNext(child);
            }
        }
        else
        {
            // Fast behavior: complete bipartite between previous frontier and this layer
            // using forward-only insertion (no backlink maintenance).
            for (DagNode *prev_node : prev_layer)
            {
                prev_node->reserveNextAdditional(cur_layer.size());
                for (DagNode *child : cur_layer)
                    prev_node->addNextNoBacklink(child);
            }
        }

        prev_layer.swap(cur_layer);
    }

    // Identify the current "last-children" frontier of this expanded method.
    std::vector<DagNode*> last_children;
    last_children.reserve(prev_layer.size());
    for (DagNode *n : prev_layer)
        if (n)
            last_children.push_back(n);
    _last_children = last_children;

    // Rewire: last_children -> each successor.
    // In detach_and_rewire=false mode we keep the old edges intact (faster),
    // and rely on the caller to rebuild a clean graph before freeing memory.
    {
        std::vector<DagNode*> succs;
        succs.reserve(_next.size());
        for (DagNode *s : _next)
            if (s)
                succs.push_back(s);

        for (DagNode *s : succs)
        {
            if (detach_and_rewire)
                removeNext(s);
            for (DagNode *lc : last_children)
            {
                if (detach_and_rewire)
                    lc->addNext(s);
                else
                    lc->addNextNoBacklink(s);
            }
        }

        if (detach_and_rewire)
        {
            // Remove predecessor -> this edges, since this METHOD is being replaced.
            // Snapshot current predecessors before mutating backlinks.
            std::vector<DagNode*> preds;
            preds.reserve(_prev.size());
            for (DagNode *p : _prev)
                if (p)
                    preds.push_back(p);
            for (DagNode *p : preds)
                p->removeNext(this);

            // Clear remaining adjacency of this node.
            _prev.clear();
            _next.clear();
        }
    }

    return created_nodes;
}

std::vector<DagNode *> DagNode::expandWithPreds(HtnInstance &htn, DagNode *const *preds, std::size_t n_preds, bool detach_and_rewire)
{
    if (!isMethod())
        return std::vector<DagNode *>();

    // Rewrite predecessors: replace METHOD predecessors by their expanded last-children.
    std::vector<DagNode *> prev_layer;
    prev_layer.reserve(n_preds);
    for (std::size_t i = 0; i < n_preds; ++i)
    {
        DagNode *prev_node = preds[i];
        if (!prev_node)
            continue;
        if (prev_node->getType() == NodeType::METHOD)
        {
            const std::vector<DagNode*> &kids = prev_node->getLastChildren();
            if (!kids.empty())
            {
                for (DagNode *child : kids)
                    if (child)
                        prev_layer.push_back(child);
            }
            else
            {
                // Predecessor METHOD not expanded yet: keep it as a predecessor.
                prev_layer.push_back(prev_node);
            }
        }
        else
        {
            prev_layer.push_back(prev_node);
        }
    }
    std::sort(prev_layer.begin(), prev_layer.end());
    prev_layer.erase(std::unique(prev_layer.begin(), prev_layer.end()), prev_layer.end());

    // Defensive: if expand() is (unexpectedly) called twice on the same METHOD node,
    // don't keep accumulating last-children.
    _last_children.clear();

    std::vector<DagNode *> created_nodes;

    const MethodExpansionTemplate &tpl = get_or_build_template(htn, op_id);

    int max_prev_ts = -1;
    for (DagNode *p : prev_layer)
        if (p)
            max_prev_ts = std::max(max_prev_ts, p->getTs());

    const int base_ts = max_prev_ts + 1;
    for (std::size_t layer_idx = 0; layer_idx < tpl.layers.size(); ++layer_idx)
    {
        const auto &layer_specs = tpl.layers[layer_idx];
        std::vector<DagNode *> cur_layer;
        cur_layer.reserve(layer_specs.size());

        const int layer_ts = base_ts + static_cast<int>(layer_idx);
        for (const ChildSpec &spec : layer_specs)
        {
            DagNode *child = new DagNode(spec.op_id, spec.last_child, spec.type);
            child->setTs(layer_ts);
            if (detach_and_rewire)
                child->reservePrev(prev_layer.size());
            cur_layer.push_back(child);
            created_nodes.push_back(child);
        }

        if (detach_and_rewire)
        {
            for (DagNode *prev_node : prev_layer)
            {
                prev_node->reserveNextAdditional(cur_layer.size());
                for (DagNode *child : cur_layer)
                    prev_node->addNext(child);
            }
        }
        else
        {
            for (DagNode *prev_node : prev_layer)
            {
                prev_node->reserveNextAdditional(cur_layer.size());
                for (DagNode *child : cur_layer)
                    prev_node->addNextNoBacklink(child);
            }
        }

        prev_layer.swap(cur_layer);
    }

    std::vector<DagNode*> last_children;
    last_children.reserve(prev_layer.size());
    for (DagNode *n : prev_layer)
        if (n)
            last_children.push_back(n);
    _last_children = last_children;

    // Rewire: last_children -> each successor.
    {
        std::vector<DagNode*> succs;
        succs.reserve(_next.size());
        for (DagNode *s : _next)
            if (s)
                succs.push_back(s);

        for (DagNode *s : succs)
        {
            if (detach_and_rewire)
                removeNext(s);
            for (DagNode *lc : last_children)
            {
                if (detach_and_rewire)
                    lc->addNext(s);
                else
                    lc->addNextNoBacklink(s);
            }
        }

        if (detach_and_rewire)
        {
            // Remove predecessor -> this edges, since this METHOD is being replaced.
            std::vector<DagNode*> preds_snapshot;
            preds_snapshot.reserve(_prev.size());
            for (DagNode *p : _prev)
                if (p)
                    preds_snapshot.push_back(p);
            for (DagNode *p : preds_snapshot)
                p->removeNext(this);

            _prev.clear();
            _next.clear();
        }
    }

    return created_nodes;
}

std::vector<DagNode *> DagNode::topologicalSort(const std::vector<DagNode *> &nodes)
{

    // Fast topological sort over the subgraph induced by --nodes--.
    // The previous implementation used unordered_map updates per edge;
    // on some instances this became very slow. Here we index nodes and
    // store indegrees in a flat vector.
    const std::size_t n_nodes = nodes.size();

    std::unordered_map<DagNode *, std::size_t> idx;
    idx.reserve(n_nodes);
    for (std::size_t i = 0; i < n_nodes; ++i)
        idx.emplace(nodes[i], i);

    // Duplicates would break the "induced subgraph" assumption.
    if (idx.size() != n_nodes)
        throw std::logic_error("Graph contains duplicate node pointers; topological sort impossible");

    std::vector<std::size_t> indeg(n_nodes, 0);

    // Compute indegrees restricted to the induced subgraph.
    for (std::size_t i = 0; i < n_nodes; ++i)
    {
        DagNode *u = nodes[i];
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

    std::vector<DagNode *> order;
    order.reserve(n_nodes);

    while (!q.empty())
    {
        const std::size_t ui = q.front();
        q.pop();
        DagNode *u = nodes[ui];
        order.push_back(u);

        for (DagNode *v : u->getNext())
        {
            auto it = idx.find(v);
            if (it == idx.end())
                continue;
            const std::size_t vi = it->second;
            if (--indeg[vi] == 0)
                q.push(vi);
        }
    }

    if (order.size() != n_nodes)
        throw std::logic_error("Graph contains a cycle; topological sort impossible");

    return order;
}

std::vector<DagNode *> DagNode::pruneMethods(const std::vector<DagNode *> &nodes)
{
    const std::size_t n_nodes = nodes.size();
    if (n_nodes == 0)
        return {};

    // Index nodes for fast induced-subgraph membership tests.
    for (std::size_t i = 0; i < n_nodes; ++i)
        nodes[i]->setTmpIndex(static_cast<int>(i));

    auto in_induced = [&](DagNode *n) -> int {
        if (!n)
            return -1;
        const int idx = n->getTmpIndex();
        if (idx < 0 || static_cast<std::size_t>(idx) >= n_nodes)
            return -1;
        if (nodes[static_cast<std::size_t>(idx)] != n)
            return -1;
        return idx;
    };

    // Compute indegree within the induced subgraph.
    std::vector<std::size_t> indeg(n_nodes, 0);
    for (std::size_t ui = 0; ui < n_nodes; ++ui)
    {
        DagNode *u = nodes[ui];
        for (DagNode *succ : u->getNext())
        {
            const int vi_i = in_induced(succ);
            if (vi_i < 0)
                continue;
            ++indeg[static_cast<std::size_t>(vi_i)];
        }
    }

    // Phase 1: forward reachability from roots (roots that are not methods).
    std::vector<unsigned char> fwd(n_nodes, 0);
    std::queue<std::size_t> q;
    for (std::size_t i = 0; i < n_nodes; ++i)
    {
        if (indeg[i] == 0 && !nodes[i]->isMethod())
        {
            fwd[i] = 1;
            q.push(i);
        }
    }
    while (!q.empty())
    {
        const std::size_t ui = q.front();
        q.pop();
        DagNode *cur = nodes[ui];
        for (DagNode *succ : cur->getNext())
        {
            if (!succ || succ->isMethod())
                continue;
            const int vi_i = in_induced(succ);
            if (vi_i < 0)
                continue;
            const std::size_t vi = static_cast<std::size_t>(vi_i);
            if (!fwd[vi])
            {
                fwd[vi] = 1;
                q.push(vi);
            }
        }
    }

    // Phase 2: backward reachability from (real) non-method leaves.
    // We do not rely on stored predecessor lists, since they may not be
    // maintained in performance-critical pipelines. Instead, build a compact
    // reverse adjacency for the induced subgraph.
    std::vector<std::size_t> rev_deg(n_nodes, 0);
    for (std::size_t ui = 0; ui < n_nodes; ++ui)
    {
        DagNode *u = nodes[ui];
        if (!u || u->isMethod())
            continue;
        for (DagNode *succ : u->getNext())
        {
            if (!succ || succ->isMethod())
                continue;
            const int vi_i = in_induced(succ);
            if (vi_i < 0)
                continue;
            ++rev_deg[static_cast<std::size_t>(vi_i)];
        }
    }

    std::vector<std::size_t> rev_off(n_nodes + 1, 0);
    for (std::size_t i = 0; i < n_nodes; ++i)
        rev_off[i + 1] = rev_off[i] + rev_deg[i];

    std::vector<std::uint32_t> rev_pred;
    rev_pred.resize(rev_off[n_nodes]);
    std::vector<std::size_t> rev_cur = rev_off;

    for (std::size_t ui = 0; ui < n_nodes; ++ui)
    {
        DagNode *u = nodes[ui];
        if (!u || u->isMethod())
            continue;
        for (DagNode *succ : u->getNext())
        {
            if (!succ || succ->isMethod())
                continue;
            const int vi_i = in_induced(succ);
            if (vi_i < 0)
                continue;
            const std::size_t vi = static_cast<std::size_t>(vi_i);
            const std::size_t pos = rev_cur[vi]++;
            if (pos < rev_pred.size())
                rev_pred[pos] = static_cast<std::uint32_t>(ui);
        }
    }

    std::vector<unsigned char> bwd(n_nodes, 0);
    for (std::size_t i = 0; i < n_nodes; ++i)
    {
        DagNode *n = nodes[i];
        if (!n || n->isMethod())
            continue;
        // A non-method leaf in the *non-method induced* subgraph.
        bool has_non_method_succ = false;
        for (DagNode *succ : n->getNext())
        {
            if (succ && !succ->isMethod() && in_induced(succ) >= 0)
            {
                has_non_method_succ = true;
                break;
            }
        }
        if (!has_non_method_succ)
        {
            bwd[i] = 1;
            q.push(i);
        }
    }

    while (!q.empty())
    {
        const std::size_t ui = q.front();
        q.pop();
        for (std::size_t p = rev_off[ui]; p < rev_off[ui + 1]; ++p)
        {
            const std::size_t pi = static_cast<std::size_t>(rev_pred[p]);
            if (!bwd[pi])
            {
                bwd[pi] = 1;
                q.push(pi);
            }
        }
    }

    // Intersection: nodes that survive both passes.
    std::vector<DagNode *> surviving;
    surviving.reserve(n_nodes);
    for (std::size_t i = 0; i < n_nodes; ++i)
    {
        if (fwd[i] && bwd[i])
        {
            nodes[i]->setActive(true);
            surviving.push_back(nodes[i]);
        }
    }

    return surviving;
}


void DagNode::assignTs(const std::vector<DagNode*>& primitive_nodes) 
{
    int ts = 0;
    for (DagNode *node : primitive_nodes)
    {
        // Get the max timestamp of the previous nodes which are primitive
        int max_prev_ts = -1; // To have 0 for the nodes with no previous nodes     
        for (DagNode *prev_node : node->getPrev())
        {
            if (!prev_node->isActive()) continue; // Skip inactive nodes

            max_prev_ts = std::max(max_prev_ts, prev_node->getTs());
        }
        node->setTs(max_prev_ts + 1);
    }
}
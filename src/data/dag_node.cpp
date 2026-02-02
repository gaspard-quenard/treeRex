#include "data/dag_node.h"
#include "util/log.h"
#include "util/names.h"

int DagNode::counter_id = 0; // Initialize the static unique_id counter

std::vector<DagNode *> DagNode::expand(HtnInstance &htn)
{

    std::unordered_set<DagNode *> old_prev = _prev;
    std::unordered_set<DagNode *> new_prev;
    for (DagNode *prev_node : old_prev)
    {
        if (prev_node->getType() == NodeType::METHOD)
        {
            for (DagNode *child : prev_node->getChildren())
            {
                if (child->isLastChild())
                    new_prev.insert(child);
            }
        }
        else
        {
            new_prev.insert(prev_node);
        }
    }

    if (!isMethod())
    {
        // ACTION nodes are kept across depth iterations.
        // Rewire predecessors with proper backlink maintenance, otherwise old METHOD
        // predecessors (that got expanded/removed) keep stale next-edges to this node.

        // Maintain a consistent "time" label for this ACTION: it must occur strictly
        // after all of its current predecessors.
        int max_prev_ts = -1;
        for (DagNode *p : new_prev)
            max_prev_ts = std::max(max_prev_ts, p->getTs());
        setTs(max_prev_ts + 1);

        rewirePrev(new_prev);

        // IMPORTANT: do NOT remove backlinks when clearing successors here.
        // Successor nodes still rely on their `_prev` sets during the same and
        // subsequent depth iterations (to rewrite METHOD predecessors to their last
        // children). Removing backlinks here disconnects the DAG and collapses the
        // primitive graph, making the SAT encoding trivially UNSAT.
        clearNextShallow();
        return std::vector<DagNode *>();
    }

    // Defensive: if expand() is (unexpectedly) called twice on the same METHOD node,
    // don't keep accumulating children.
    _children.clear();

    const Method &method = htn.getMethodById(op_id);
    // Log::i("Children of method %s\n", TOSTR(method));
    std::unordered_set<DagNode *> prev = new_prev;
    int num_offset = method.getSubtasksIdx().size() - 1;
    for (size_t j = 0; j < method.getSubtasksIdx().size(); ++j)
    {
        std::unordered_set<DagNode *> next_nodes;
        int max_prev_ts = -1;
        for (DagNode *p : prev)
            max_prev_ts = std::max(max_prev_ts, p->getTs());

        int subtask_idx = method.getSubtasksIdx()[j];
        if (htn.isAbstractTask(subtask_idx))
        {
            // Log::i("  Subtask %d is the abstract task %s\n", j, TOSTR(htn.getAbstractTaskById(subtask_idx)));
            const AbstractTask &task = htn.getAbstractTaskById(subtask_idx);
            // Add all the methods of the abstract task to the j-th children
            for (int sub_method_idx : task.getDecompositionMethodIdx())
            {
                // Log::i("At subtask %d, adding method %s\n", j, TOSTR(htn.getMethodById(sub_method_idx)));
                // Create a new dag node for the sub-method
                DagNode *sub_method_node = new DagNode(sub_method_idx, j == num_offset, NodeType::METHOD);
                sub_method_node->setTs(max_prev_ts + 1);
                // All previous nodes have this node as next
                for (DagNode *prev_node : prev)
                {
                    prev_node->addNext(sub_method_node);
                }
                next_nodes.insert(sub_method_node);
                _children.push_back(sub_method_node);
            }
        }
        else
        {
            // Log::i("  Subtask %d is the action %s\n", j, TOSTR(htn.getActionById(subtask_idx)));
            // Create a new dag node for the action
            DagNode *action_node = new DagNode(subtask_idx, j == num_offset, NodeType::ACTION);
            action_node->setTs(max_prev_ts + 1);
            // All previous nodes have this node as next
            for (DagNode *prev_node : prev)
            {
                prev_node->addNext(action_node);
            }
            next_nodes.insert(action_node);
            _children.push_back(action_node);
        }
        prev = next_nodes;
        next_nodes.clear();
    }

    // Detach this METHOD node from the working graph so it doesn't leave stale edges
    // behind (it is replaced by its children in the frontier).
    rewirePrev(std::unordered_set<DagNode *>{});
    // Same rationale as for ACTION nodes above: keep successor `_prev` pointers intact
    // so they can rewrite METHOD predecessors to last-children in later iterations.
    clearNextShallow();
    return _children;
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
    // Restrict to the subgraph induced by --nodes--.
    std::unordered_set<DagNode *> node_set;
    node_set.reserve(nodes.size());
    for (DagNode *n : nodes)
        node_set.insert(n);

    /*------------  Build reverse adjacency & indegree --------------*/
    std::unordered_map<DagNode *, std::vector<DagNode *>> preds;
    std::unordered_map<DagNode *, std::size_t> indeg;
    preds.reserve(nodes.size());
    indeg.reserve(nodes.size());

    for (DagNode *n : nodes)
    {
        indeg.try_emplace(n, 0);
        for (DagNode *succ : n->getNext())
        {
            if (!node_set.count(succ))
                continue;
            preds[succ].push_back(n);
            ++indeg[succ];
        }
    }

    /*------------  Phase 1: forward reachability from roots --------*/
    std::unordered_set<DagNode *> fwd; // reachable & non-method
    std::queue<DagNode *> q;

    for (DagNode *n : nodes)
        if (indeg[n] == 0 && !n->isMethod()) // roots that are not methods
        {
            fwd.insert(n);
            q.push(n);
        }

    while (!q.empty())
    {
        DagNode *cur = q.front();
        q.pop();
        for (DagNode *succ : cur->getNext())
            if (!succ->isMethod() && fwd.insert(succ).second)
                q.push(succ);
    }

    /*------------  Phase 2: backward reachability from leaves ------*/
    std::unordered_set<DagNode *> bwd; // can reach a non-method leaf
    for (DagNode *n : nodes)
        if (!n->isMethod() && n->getNext().empty()) // real leaves
        {
            bwd.insert(n);
            q.push(n);
        }

    while (!q.empty())
    {
        DagNode *cur = q.front();
        q.pop();
        for (DagNode *pred : preds[cur])
            if (!pred->isMethod() && bwd.insert(pred).second)
                q.push(pred);
    }

    /*------------  Intersection: nodes that survive both passes ----*/
    std::vector<DagNode *> surviving;
    surviving.reserve(nodes.size());
    for (DagNode *n : nodes)
        if (fwd.count(n) && bwd.count(n)) {
            n->setActive(true); // Mark as active
            surviving.push_back(n);
        }

    /*------------  Optional: sort the surviving DAG ---------------*/
    // return DagNode::topologicalSort(surviving); // Not sure it is useful if nodes is already sorted
    return surviving; // return the surviving nodes
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
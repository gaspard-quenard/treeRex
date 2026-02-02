#ifndef DAG_NODE_H
#define DAG_NODE_H

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stdexcept>  
#include <vector>
#include <algorithm>

#include "data/htn_instance.h"
#include "util/names.h"


enum class NodeType
{
    METHOD,
    ACTION
};


class DagNode
{

// static Id 
static int counter_id; // Static variable to keep track of unique IDs

    static int getUniqueId() {
        return DagNode::counter_id++;
    }
private:

    int unique_id = getUniqueId(); // Unique identifier for this DagNode instance
    int _last_child;

    int op_id; // Unique identifier for the operation
    // NOTE: For performance, adjacency is stored as vectors.
    // This makes edge insertion during expansion O(1) and avoids hashing.
    // Deduplication is handled by higher-level passes where needed.
    std::vector<DagNode *> _prev;
    std::vector<DagNode *> _next;
    NodeType _type;
    int _ts;
    int sat_var;
    // For METHOD nodes, store only the last-children frontier created by expansion.
    // Successor METHOD expansions use this to replace METHOD predecessors with their
    // realized frontier nodes.
    std::vector<DagNode*> _last_children;
    bool _is_active = false;


    // Ephemeral index used by planner-side passes (e.g., topo/timestamp) to avoid
    // building pointer->index hash maps each iteration.
    // Valid only if the caller also checks membership (e.g., nodes[tmp]==this).
    int _tmp_index = -1;

    // Conditional successors keyed by SINGLE predecessor.
    // This matches the encoder assumption (it chooses exactly one incoming edge).
    // Using DagNode* as key avoids allocating/hashing std::vector keys during compression.
    std::unordered_map<DagNode*, std::unordered_set<DagNode*>> _cond_next_single;

    void addPrev(DagNode *node) { _prev.push_back(node); }
    void removePrev(DagNode *node)
    {
        _prev.erase(std::remove(_prev.begin(), _prev.end(), node), _prev.end());
    }

public:
    DagNode(int id, bool last_child, NodeType type) : op_id(id), _last_child(last_child), _type(type) {}

    int getId() const { return unique_id; } // Getter for unique_id
    int getOpId() const { return op_id; }
    NodeType getType() const { return _type; }

    bool isMethod() const { return _type == NodeType::METHOD; }


    void reservePrev(std::size_t n) { _prev.reserve(n); }
    void reserveNext(std::size_t n) { _next.reserve(n); }

    // Reserve space for appending edges without pathological repeated reallocations.
    // This uses exponential growth rather than reserving the exact new size.
    void reserveNextAdditional(std::size_t additional)
    {
        const std::size_t needed = _next.size() + additional;
        if (needed <= _next.capacity())
            return;
        std::size_t cap = _next.capacity();
        if (cap == 0)
            cap = 1;
        while (cap < needed)
            cap *= 2;
        _next.reserve(cap);
    }

    void reservePrevAdditional(std::size_t additional)
    {
        const std::size_t needed = _prev.size() + additional;
        if (needed <= _prev.capacity())
            return;
        std::size_t cap = _prev.capacity();
        if (cap == 0)
            cap = 1;
        while (cap < needed)
            cap *= 2;
        _prev.reserve(cap);
    }

    
    void addNext(DagNode *node)
    {
        _next.push_back(node);
        node->addPrev(this);
    }

    // Forward-only edge insertion: does NOT update the successor's _prev.
    // This is a major speed win when building huge temporary graphs.
    void addNextNoBacklink(DagNode *node)
    {
        _next.push_back(node);
    }

    void removeNext(DagNode *node) {
        _next.erase(std::remove(_next.begin(), _next.end(), node), _next.end());
        node->removePrev(this);
    }

    void clearNext()
    {
        // Fast unlink: remove backlinks from successors, then clear.
        // Equivalent to calling removeNext(succ) for all succ, but avoids
        // copying the set and re-hashing on each erase.
        for (DagNode *succ : _next)
            if (succ)
                succ->removePrev(this);
        _next.clear();
    }

    void clearNextShallow()
    {
        _next.clear();
    }

    const std::vector<DagNode *> &getPrev() const { return _prev; }
    const std::vector<DagNode *> &getNext() const { return _next; }

    void clearPrevShallow() { _prev.clear(); }

    bool hasConditionalNext() const { return !_cond_next_single.empty(); }
    const std::unordered_map<DagNode*, std::unordered_set<DagNode*>> &getConditionalNext() const { return _cond_next_single; }

    void addConditionalNext(std::vector<DagNode*> ctx, DagNode *succ)
    {
        // store canonicalized context
        std::sort(ctx.begin(), ctx.end());
        ctx.erase(std::unique(ctx.begin(), ctx.end()), ctx.end());
        if (ctx.size() != 1)
            throw std::logic_error("Conditional successor contexts must be singleton");
        _cond_next_single[ctx[0]].insert(succ);
    }

    // Fast path for singleton contexts.
    void addConditionalNextSingle(DagNode *pred, DagNode *succ)
    {
        _cond_next_single[pred].insert(succ);
    }

    std::unordered_set<DagNode*> &getOrCreateConditionalNextSetSingle(DagNode *pred)
    {
        return _cond_next_single[pred];
    }

    bool operator==(const DagNode &other) const { return op_id == other.op_id && _type == other._type; }

    // Expand a METHOD node into its children.
    // If detach_and_rewire is true, we unlink this METHOD from predecessors and
    // successors (so it can be deleted immediately after expansion).
    // If false, we only add the new edges (pred->children, last_children->succ),
    // leaving old edges intact. This is much faster but requires the caller to
    // keep the old nodes alive until it rebuilds a clean graph.
    std::vector<DagNode*> expand(HtnInstance &htn, bool detach_and_rewire);

    // Same as expand(), but uses an explicit predecessor list instead of this node's
    // stored _prev adjacency. This is useful in compress_dag mode where we avoid
    // maintaining _prev and compute predecessors on-demand from forward edges.
    // Only intended for METHOD nodes.
    std::vector<DagNode*> expandWithPreds(HtnInstance &htn, DagNode *const *preds, std::size_t n_preds, bool detach_and_rewire);
    const std::vector<DagNode*> &getLastChildren() const { return _last_children; }
    bool isLastChild() const { return _last_child; }

    void setLastChild(bool last_child) { _last_child = _last_child || last_child; }

    void setLastChildren(std::vector<DagNode*> last_children) { _last_children = std::move(last_children); }

    void setActive(bool active) { _is_active = active; }
    bool isActive() const { return _is_active; }

    void setTs(int ts) { _ts = ts; }
    int getTs() const { return _ts; }

    void setSatVar(int var) { sat_var = var; }
    int getSatVar() const { return sat_var; }

    void setTmpIndex(int idx) { _tmp_index = idx; }
    int getTmpIndex() const { return _tmp_index; }

    // For performance-critical passes (notably in compress_dag mode), callers can
    // pre-filter outgoing edges to the induced subgraph so later traversals can
    // skip expensive membership checks.
    // WARNING: This does not update successor back-links (_prev). It is intended
    // only when the caller will shortly discard/rebuild the graph.
    void compactNextToInduced(const std::vector<DagNode*> &induced);

    // Rebuild _prev for each node from its _next lists.
    // Assumes nodes have valid tmpIndex (0..n-1) matching their position and
    // that each node's _next has already been compacted to this induced set.
    static void rebuildPrevFromNext(const std::vector<DagNode*> &nodes);

    // LTP fast path: only METHOD nodes need predecessor lists for expansion.
    // Rebuild _prev only for METHOD nodes from successor lists.
    static void rebuildPrevForMethodsFromNext(const std::vector<DagNode*> &nodes);

    /*--------------------------------------------------------------------
       Topologically sorts the *sub-DAG* induced by --nodes--
       Returns  : vector of nodes in valid evaluation order.
       Throws   : std::logic_error if the graph has a cycle
     --------------------------------------------------------------------*/
    static std::vector<DagNode*>
    topologicalSort(const std::vector<DagNode*>& nodes);


        /*--------------------------------------------------------------------
        Returns:  vector containing all NON-method nodes that belong to at
                  least one method-free path root → … → leaf.
                  The vector is in *topological order* (same Kahn routine
                  we wrote earlier).

        Throws :  std::logic_error if the graph is cyclic.
     --------------------------------------------------------------------*/
    static std::vector<DagNode*> pruneMethods(const std::vector<DagNode*>& nodes);

    static void assignTs(const std::vector<DagNode*>& primitive_nodes);

    std::string getName(HtnInstance& htn) const {
        std::string type_str = (_type == NodeType::METHOD) ? "M" : "A";
        std::string op_str;
        if (_type == NodeType::METHOD)
        {
            // Keep method naming safe: method ids might be invalid in some debug/compressed graphs.
            // Using numeric ids avoids out-of-range accesses when assertions are compiled out.
            op_str = "id" + std::to_string(op_id);
        }
        else
        {
            // ACTION nodes: only resolve names when the id is known to be valid.
            // (Avoid out-of-range accesses when assertions are compiled out.)
            const int goal_id = htn.getIdGoalAction();
            const int blank_id = htn.getBlankAction().getId();

            if (op_id == goal_id || op_id == blank_id || (!htn.isAbstractTask(op_id) && op_id >= 0))
                op_str = Names::to_string(htn.getActionById(op_id));
            else
                op_str = "id" + std::to_string(op_id);
        }
        return type_str + "_" + op_str + "%" + std::to_string(unique_id); // + "__ts" + std::to_string(_ts);
    }
};


#endif // DAG_NODE_H
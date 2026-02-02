#ifndef DAG_NODE_H
#define DAG_NODE_H

#include <unordered_set>
#include <unordered_map>
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
    std::unordered_set<DagNode *> _prev;
    std::unordered_set<DagNode *> _next;
    NodeType _type;
    int _ts;
    int sat_var;
    std::vector<DagNode*> _children;
    bool _is_active = false;

    struct PrevContextHash
    {
        std::size_t operator()(const std::vector<DagNode*> &ctx) const
        {
            std::size_t h = 0;
            for (DagNode *p : ctx)
            {
                std::size_t x = std::hash<DagNode*>{}(p);
                h ^= x + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            }
            return h;
        }
    };

    // Conditional successors: for a given predecessor-context (set of incoming nodes used
    // to reach this node instance), which successors are valid.
    std::unordered_map<std::vector<DagNode*>, std::unordered_set<DagNode*>, PrevContextHash> _cond_next;

    void addPrev(DagNode *node) { _prev.insert(node); }
    void removePrev(DagNode *node) { _prev.erase(node); }

public:
    DagNode(int id, bool last_child, NodeType type) : op_id(id), _last_child(last_child), _type(type) {}

    int getId() const { return unique_id; } // Getter for unique_id
    int getOpId() const { return op_id; }
    NodeType getType() const { return _type; }

    bool isMethod() const { return _type == NodeType::METHOD; }

    
    void addNext(DagNode *node)
    {
        // Avoid redundant work if the edge already exists.
        if (_next.insert(node).second)
            node->addPrev(this);
    }

    void removeNext(DagNode *node) {
        if (_next.erase(node) > 0) {
            node->removePrev(this);
        }
    }

    void clearNext()
    {
        std::unordered_set<DagNode *> old_next = _next;
        for (DagNode *succ : old_next)
            removeNext(succ);
    }

    void clearNextShallow()
    {
        _next.clear();
    }

    void rewirePrev(const std::unordered_set<DagNode *> &new_prev)
    {
        // Remove existing backlinks (copy to avoid iterator invalidation)
        std::unordered_set<DagNode *> old_prev = _prev;
        for (DagNode *old : old_prev)
            old->removeNext(this);

        _prev.clear(); // now clean
        // Add new backlinks
        for (DagNode *p : new_prev)
        {
            p->addNext(this);
        }
    }

    const std::unordered_set<DagNode *> &getPrev() const { return _prev; }
    const std::unordered_set<DagNode *> &getNext() const { return _next; }

    bool hasConditionalNext() const { return !_cond_next.empty(); }
    const std::unordered_map<std::vector<DagNode*>, std::unordered_set<DagNode*>, PrevContextHash> &getConditionalNext() const { return _cond_next; }

    static std::vector<DagNode*> canonicalizePrevContext(const std::unordered_set<DagNode*> &prev)
    {
        std::vector<DagNode*> ctx;
        ctx.reserve(prev.size());
        for (DagNode *p : prev)
            ctx.push_back(p);
        std::sort(ctx.begin(), ctx.end());
        ctx.erase(std::unique(ctx.begin(), ctx.end()), ctx.end());
        return ctx;
    }

    void addConditionalNext(std::vector<DagNode*> ctx, DagNode *succ)
    {
        // store canonicalized context
        std::sort(ctx.begin(), ctx.end());
        ctx.erase(std::unique(ctx.begin(), ctx.end()), ctx.end());
        _cond_next[std::move(ctx)].insert(succ);
    }

    bool operator==(const DagNode &other) const { return op_id == other.op_id && _type == other._type; }

    std::vector<DagNode*> expand(HtnInstance &htn);
    std::vector<DagNode*> getChildren() const { return _children; }
    bool isLastChild() const { return _last_child; }

    void setLastChild(bool last_child) { _last_child = _last_child || last_child; }
    void addChild(DagNode *child) { _children.push_back(child); }

    void setActive(bool active) { _is_active = active; }
    bool isActive() const { return _is_active; }

    void setTs(int ts) { _ts = ts; }
    int getTs() const { return _ts; }

    void setSatVar(int var) { sat_var = var; }
    int getSatVar() const { return sat_var; }

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
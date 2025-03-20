#ifndef METHOD_H
#define METHOD_H
#include <vector>

#include "data/predicate.h"

class Method // : public HTNOp
{

private:
    int _id;
    std::string _name;
    int _parent_task_idx;
    std::vector<int> _subtasks_idx;

public:
    Method(int id, std::string name, int parent_task_idx, std::vector<int> subtasks_idx) : _id(id), _name(name), _parent_task_idx(parent_task_idx), _subtasks_idx(subtasks_idx) {}

    const std::string getName() const
    {
        return _name;
    }

    const std::vector<int> &getSubtasksIdx() const
    {
        return _subtasks_idx;
    }
};

#endif // METHOD_H
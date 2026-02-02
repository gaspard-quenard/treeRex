#ifndef METHOD_H
#define METHOD_H
#include <vector>

#include "data/predicate.h"

class Method
{

private:
    int _id;
    std::string _name;
    int _parent_task_idx;
    std::vector<int> _subtasks_idx;

    std::vector<int> _preconditions_idx;
    std::vector<int> _pos_effs_idx;
    std::vector<int> _neg_effs_idx;
    std::vector<int> _poss_pos_effs_idx;
    std::vector<int> _poss_neg_effs_idx;

public:
    Method(int id, std::string name, int parent_task_idx, std::vector<int> subtasks_idx) : _id(id), _name(name), _parent_task_idx(parent_task_idx), _subtasks_idx(subtasks_idx) {}

    const std::string getName() const { return _name; }
    const int getId() const { return _id; }
    const std::vector<int> &getSubtasksIdx() const { return _subtasks_idx; }

    void setPreconditions(std::vector<int> preconditions_idx) { _preconditions_idx = preconditions_idx; }
    void setPositiveEffects(std::vector<int> pos_effs_idx) { _pos_effs_idx = pos_effs_idx; }
    void setNegativeEffects(std::vector<int> neg_effs_idx) { _neg_effs_idx = neg_effs_idx; }
    void setPossiblePositiveEffects(std::vector<int> poss_pos_effs_idx) { _poss_pos_effs_idx = poss_pos_effs_idx; }
    void setPossibleNegativeEffects(std::vector<int> poss_neg_effs_idx) { _poss_neg_effs_idx = poss_neg_effs_idx; }

    void removeFirstSubtask()
    {
        if (!_subtasks_idx.empty())
        {
            _subtasks_idx.erase(_subtasks_idx.begin());
        }
    }

    const std::vector<int> &getPreconditionsIdx() const { return _preconditions_idx; }
    const std::vector<int> &getPosEffsIdx() const { return _pos_effs_idx; }
    const std::vector<int> &getNegEffsIdx() const { return _neg_effs_idx; }
    const std::vector<int> &getPossPosEffsIdx() const { return _poss_pos_effs_idx; }
    const std::vector<int> &getPossNegEffsIdx() const { return _poss_neg_effs_idx; }
};

#endif // METHOD_H
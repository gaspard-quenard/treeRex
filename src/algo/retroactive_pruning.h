#ifndef RETROACTIVE_PRUNING_H
#define RETROACTIVE_PRUNING_H

#include "data/htn_instance.h"
#include "data/pdt_node.h"
#include "sat/encoding.h"

class RetroactivePruning
{

private:
    HtnInstance &_htn;
    Encoding &_enc;

    int num_pruned_actions = 0;
    int num_pruned_methods = 0;

public:
    RetroactivePruning(HtnInstance &htn, Encoding &enc)
        : _htn(htn), _enc(enc) {}

    void prune(PdtNode *node, int op_idx, OpType op_type);
    const int getNumPrunedActions() const { return num_pruned_actions; }
    const int getNumPrunedMethods() const { return num_pruned_methods; }
};

#endif // RETROACTIVE_PRUNING_H
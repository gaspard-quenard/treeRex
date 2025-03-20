#include "util/names.h"

#include <vector>

std::vector<Predicate>* g_predicates;
std::vector<Action>* g_actions;
std::vector<AbstractTask>* g_abstr_tasks;
std::vector<Method>* g_methods;


namespace Names {
    void init(std::vector<Predicate>& predicates, std::vector<Action>& actions, std::vector<AbstractTask>& abstr_tasks, std::vector<Method>& methods) {
        g_predicates = &predicates;
        g_actions = &actions;
        g_abstr_tasks = &abstr_tasks;
        g_methods = &methods;
    }

    std::string to_string(const Action& a) {
        std::string out;
        // First, print all the preconditions
        for (int idx : a.getPreconditionsIdx()) {
            out += g_predicates->at(idx).getName() + " ";
        }
        out += " { " + a.getName() + " } ";
        // Next, print all the positive effects
        for (int idx : a.getPosEffsIdx()) {
            out += g_predicates->at(idx).getName() + " ";
        }
        // And finally, print all the negative effects
        for (int idx : a.getNegEffsIdx()) {
            out += "not " + g_predicates->at(idx).getName() + " ";
        }
        return out;
    }

    std::string to_string(const AbstractTask& at) {
        std::string out = at.getName() + " { ";
        for (int idx : at.getDecompositionMethodIdx()) {
            out += g_methods->at(idx).getName() + " ";
        }
        out += "}";
        return out;
    }

    std::string to_string(const Method& m) {
        std::string out = m.getName() + " { ";
        for (int idx : m.getSubtasksIdx()) {
            if (idx > g_actions->size()) {
                out += g_abstr_tasks->at(idx - g_actions->size()).getName() + " ";
            } else {
                out += g_actions->at(idx).getName() + " ";
            }
        }
        out += "}";
        return out;
    }

    std::string to_string(const Predicate& p) {
        return p.getName();
    }
}
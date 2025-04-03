#ifndef HTN_INSTANCE_H
#define HTN_INSTANCE_H

#include <vector>
#include <string>
#include <optional>
#include <fstream>
#include <unordered_set>
#include "data/action.h"
#include "data/method.h"
#include "data/abstract_task.h"
#include "util/params.h"

class HtnInstance
{
private:
    Parameters &_params;

    std::vector<Predicate> _predicates;
    std::vector<Action> _actions;
    std::vector<AbstractTask> _abstr_tasks;
    std::vector<Method> _methods;
    int _id_goal_action;
    Action *_blankAction;

    int _root_task_idx;
    std::unordered_set<int> _init_state;
    std::unordered_set<int> _goal_state;
    std::vector<int> _all_fact_vars_goal;

    /**
     * Parse the domain and problem files using pandaPIparser.
     *
     * @param domain_filename The domain file to parse.
     * @param problem_filename The problem file to parse.
     * @return The filepath of the parsed problem, or std::nullopt if parsing fails.
     */
    std::optional<std::string> parseProblem(const std::string &domain_filename, const std::string &problem_filename);

    /**
     * Ground the parsed problem using pandaPIgrounder.
     *
     * @param parsed_problem_filepath The parsed problem file to ground.
     * @return The filepath of the grounded problem, or std::nullopt if grounding fails.
     */
    std::optional<std::string> groundProblem(const std::string &parsed_problem_filepath);

    /**
     * Load the grounded problem from file and populate internal structures.
     *
     * @param grounded_problem_filepath The grounded problem file to load.
     */
    void loadGroundedProblem(const std::string &grounded_problem_filepath);

    /**
     * Skip lines in the file until a specific target line is found.
     *
     * @param file The file to read from.
     * @param target The target line to search for.
     * @param line_idx The index of the current line (updated to the found line).
     */
    void skipUntil(std::ifstream &file, const std::string &target, int &line_idx);

    /**
     * Read a space-separated list of integers from a file.
     *
     * @param file The file to read from.
     * @param line_idx The current line index (incremented after reading).
     * @return A vector containing the parsed integers.
     */
    std::vector<int> parseIntegerList(std::ifstream &file, int &line_idx);

    /**
     * Extract predicates from the grounded problem and store them in `_predicates`.
     *
     * @param file The input file stream.
     * @param line_idx The index of the line to start extraction from.
     */
    void extractPredicates(std::ifstream &file, int &line_idx);

    /**
     * Extract actions from the grounded problem and store them in `_actions`.
     *
     * @param file The input file stream.
     * @param line_idx The index of the line to start extraction from.
     */
    void extractActions(std::ifstream &file, int &line_idx);

    /**
     * Extract initial and goal states from the grounded problem.
     *
     * @param file The input file stream.
     * @param line_idx The index of the line to start extraction from.
     */
    void extractInitGoalStates(std::ifstream &file, int &line_idx);

    /**
     * Extract task names and populate `_abstr_tasks` and `_actions`.
     *
     * @param file The input file stream.
     * @param line_idx The index of the line to start extraction from.
     */
    void extractTasksNames(std::ifstream &file, int &line_idx);

    /**
     * Extract the root task index from the grounded problem file.
     *
     * @param file The input file stream.
     * @param line_idx The index of the line to start extraction from.
     */
    void extractInitRootTaskIdx(std::ifstream &file, int &line_idx);

    /**
     * Extract methods and store them in `_methods`.
     *
     * @param file The input file stream.
     * @param line_idx The index of the line to start extraction from.
     */
    void extractMethods(std::ifstream &file, int &line_idx);

    /**
     * Sort subtasks based on ordering constraints.
     *
     * @param subtasks_id The vector of subtasks to be sorted.
     * @param ordering_constraints The vector of ordering constraints.
     */
    void sortSubtasks(std::vector<int> &subtasks_id, std::vector<std::pair<int, int>> &ordering_constraints);

public:
    /**
     * Constructor: Initializes and processes the HTN instance.
     *
     * @param params The parameters for parsing and grounding.
     */
    HtnInstance(Parameters &params);

    Parameters &getParams() const;

    const AbstractTask &getAbstractTaskById(int task_id) const;
    const Action &getActionById(int action_id) const;
    const Method &getMethodById(int method_id) const;
    const Predicate &getPredicateById(int pred_id) const;
    const bool isAbstractTask(int task_id) const;
    const Action &getBlankAction() const;
    const AbstractTask &getRootTask() const;
    const bool isRootTask(const AbstractTask &task) const;
    const std::vector<Predicate> &getPredicates() const;
    // Action getGoalAction() const;
    const std::unordered_set<int> &getInitState() const;
    const std::unordered_set<int> &getGoalState() const;
    const int getNumPredicates() const;
    const std::vector<int> &getFactVarsGoal() const;
};

#endif // HTN_INSTANCE_H

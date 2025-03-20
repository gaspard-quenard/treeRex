#include "data/htn_instance.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_map>

#include "util/log.h"
#include "util/command_utils.h"
#include "util/project_utils.h"
#include "util/names.h"

HtnInstance::HtnInstance(Parameters &params) : _params(params)
{
    auto parsed_problem = parseProblem(params.getDomainFilename(), params.getProblemFilename());
    if (!parsed_problem)
        return;

    auto grounded_problem = groundProblem(*parsed_problem);
    if (!grounded_problem)
        return;

    loadGroundedProblem(*grounded_problem);
}

std::optional<std::string> HtnInstance::parseProblem(const std::string &domain_filepath, const std::string &problem_filepath)
{
    std::filesystem::path parser_path = getProjectRootDir() / "lib" / "pandaPIparser";
    std::string output_filepath = (getProblemProcessingDir() / "problem.parsed").string();
    std::string command = parser_path.string() + " --no-split-parameters " + domain_filepath + " " + problem_filepath + " " + output_filepath;

    if (runCommand(command, "Parsing the domain and problem files failed.") != 0)
        return std::nullopt;

    return output_filepath;
}

std::optional<std::string> HtnInstance::groundProblem(const std::string &parsed_problem_filepath)
{
    if (!std::filesystem::exists(parsed_problem_filepath))
    {
        Log::e("Error: The parsed problem file does not exist.\n");
        return std::nullopt;
    }

    std::filesystem::path grounder_path = getProjectRootDir() / "lib" / "pandaPIgrounder";
    std::string output_filepath = (getProblemProcessingDir() / "problem.grounded").string();
    std::string command = grounder_path.string() + " --write-full-methods-name " + parsed_problem_filepath + " " + output_filepath;

    if (runCommand(command, "Grounding the parsed problem failed.") != 0)
        return std::nullopt;

    return output_filepath;
}

void HtnInstance::loadGroundedProblem(const std::string &grounded_problem_filepath)
{
    if (!std::filesystem::exists(grounded_problem_filepath))
    {
        Log::e("Error: The grounded problem file does not exist.\n");
        return;
    }

    std::ifstream file(grounded_problem_filepath);
    if (!file)
    {
        Log::e("Error: Unable to open the grounded problem file.\n");
        return;
    }

    int line_idx = 0;
    extractPredicates(file, line_idx);
    extractActions(file, line_idx);
    extractInitGoalStates(file, line_idx);
    extractTasksNames(file, line_idx);
    extractInitRootTaskIdx(file, line_idx);
    extractMethods(file, line_idx);

    Names::init(_predicates, _actions, _abstr_tasks, _methods);

    // To debug, print all structures
    // Log::i("Predicates:\n");
    // for (const Predicate &pred : _predicates)
    // {
    //     Log::i("%s\n", TOSTR(pred));
    // }
    // Log::i("Actions:\n");
    // for (const Action &act : _actions)
    // {
    //     Log::i("%s\n", TOSTR(act));
    // }
    // Log::i("Abstract tasks:\n");
    // for (const AbstractTask &task : _abstr_tasks)
    // {
    //     Log::i("%s\n", TOSTR(task));
    // }
    // Log::i("Methods:\n");
    // for (const Method &method : _methods)
    // {
    //     Log::i("%s\n", TOSTR(method));
    // }
    // Log::i("Initial state:\n");
    // for (int pred_id : _init_state)
    // {
    //     Log::i("%s\n", TOSTR(_predicates[pred_id]));
    // }
    // Log::i("Goal state:\n");
    // for (int pred_id : _goal_state)
    // {
    //     Log::i("%s\n", TOSTR(_predicates[pred_id]));
    // }
}

void HtnInstance::skipUntil(std::ifstream &file, const std::string &target, int &line_idx)
{
    std::string line;
    while (std::getline(file, line))
    {
        ++line_idx;
        if (line == target)
            return;
    }
    // Indicate that the target was not found
    Log::e("Error: Target string '%s' not found in the file.\n", target.c_str());
}

std::vector<int> HtnInstance::parseIntegerList(std::ifstream &file, int &line_idx)
{
    std::vector<int> numbers;
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    int value;
    while (iss >> value)
    {
        if (value == -1)
            break;
        numbers.push_back(value);
    }
    ++line_idx;
    return numbers;
}

void HtnInstance::extractPredicates(std::ifstream &file, int &line_idx)
{
    skipUntil(file, ";; #state features", line_idx);

    std::string line;
    std::getline(file, line);
    int num_predicates = std::stoi(line);
    _predicates.reserve(num_predicates);
    ++line_idx;

    int pred_id = 0;
    while (std::getline(file, line) && !line.empty())
    {
        bool is_positive = line[0] == '+';
        _predicates.emplace_back(pred_id++, is_positive, line);
        ++line_idx;
    }
}

void HtnInstance::extractActions(std::ifstream &file, int &line_idx)
{
    skipUntil(file, ";; Actions", line_idx);

    std::string line;
    std::getline(file, line);
    int num_actions = std::stoi(line);
    _actions.reserve(num_actions);
    ++line_idx;

    /**
     * Each block of $4$ lines describes an action.
     *
     * The first line of each action description contains a single integer -- the cost of that action.
       The second line contains a space-separated list of integers.
       The last integer is always $-1$ and all other integers are non-negative.
       Each such non-negative integer $i$ describes that the state feature $i$ is a precondition of this action.
       The third and fourth line per action have the same format.
       They consist of a sequence of blocks of integers.
       Each block starts with an integer $\ell$.
       The line ends with block with $\ell = -1$ which carries no further semantics.
       For each other block, $\ell+1$ non-negative integers follow after the value $\ell$.
       Each such block describes a conditional effect of the action.
       The first $\ell$ integers after the first integer $\ell$ of each block describe the conditions of this conditional effect.
       Each such integer $i$ refers to the state feature $i$.
       The last integer $e$ of each block describes the effect of the conditional effect.
       In the third line of each action this is the effect of adding the state feature $e$, while in the fourth line, it is the effect of deleting the state feature $e$.
       Note that there might be *two* space between the last integer of each block and the next start of a block.
     */

    int action_id = 0;
    while (std::getline(file, line) && !line.empty())
    {
        int cost = std::stoi(line);
        ++line_idx;

        std::vector<int> preconditions = parseIntegerList(file, line_idx);
        std::vector<int> positive_conditional_and_unconditional_effects = parseIntegerList(file, line_idx);
        std::vector<int> negative_conditional_and_unconditional_effects = parseIntegerList(file, line_idx);

        // Check that all positive and negative effects are unconditional (i.e., no conditional effects)
        // To do so, all the even idx elements of the effects list should be 0
        // and all the odd idx elements should be the predicate ID
        std::vector<int> positive_effects, negative_effects;
        bool even = true;
        for (int effect : positive_conditional_and_unconditional_effects)
        {
            if (even)
            {
                if (effect != 0)
                {
                    Log::e("Error: Conditional effects are not supported.\n");
                    return;
                }
            }
            else
            {
                positive_effects.push_back(effect);
            }
            even = !even;
        }
        even = true;

        for (int effect : negative_conditional_and_unconditional_effects)
        {
            if (even)
            {
                if (effect != 0)
                {
                    Log::e("Error: Conditional effects are not supported.\n");
                    return;
                }
            }
            else
            {
                negative_effects.push_back(effect);
            }
            even = !even;
        }

        _actions.emplace_back(action_id++, preconditions, positive_effects, negative_effects);
    }
    Log::i("There are %d actions in the grounded problem.\n", num_actions);
}

void HtnInstance::extractInitGoalStates(std::ifstream &file, int &line_idx)
{

    /**
     * Each such non-negative integer $i$ indicates that the state feature $i$ holds in the initial state.
       Any state feature that does not occur in this line is false in the initial state
     */
    skipUntil(file, ";; initial state", line_idx);
    _init_state = parseIntegerList(file, line_idx);

    skipUntil(file, ";; goal", line_idx);
    _goal_state = parseIntegerList(file, line_idx);
}

void HtnInstance::extractTasksNames(std::ifstream &file, int &line_idx)
{
    skipUntil(file, ";; tasks (primitive and abstract)", line_idx);

    std::string line;
    std::getline(file, line);
    Log::i("Line: %s\n", line.c_str());
    int num_tasks = std::stoi(line);
    _abstr_tasks.reserve(num_tasks - _actions.size());
    ++line_idx;

    int task_id = 0;
    while (std::getline(file, line) && !line.empty())
    {
        bool is_abstract = line[0] == '1';
        std::string task_name = line.substr(2);

        if (is_abstract)
        {
            _abstr_tasks.emplace_back(task_id, task_name);
        }
        else
        {
            _actions[task_id].addName(task_name);
        }
        ++line_idx;
        ++task_id;
    }
}

void HtnInstance::extractInitRootTaskIdx(std::ifstream &file, int &line_idx)
{
    skipUntil(file, ";; initial abstract task", line_idx);

    std::string line;
    std::getline(file, line);
    _root_task_idx = std::stoi(line);
    ++line_idx;
}

void HtnInstance::extractMethods(std::ifstream &file, int &line_idx)
{
    skipUntil(file, ";; methods", line_idx);

    std::string line;
    std::getline(file, line);
    int num_methods = std::stoi(line);
    _methods.reserve(num_methods);
    ++line_idx;

    /**
     * Each method is described by a block of $4$ consecutive lines.
     *
     * The first line per decomposition method always contains a string without any whitespace character -- the name of the method.
       The second line contains a single non-negative integer $a$ -- the abstract task this method decomposes.
       The third line contains a space-separated list of integers.
       The last integer is always $-1$, while all other integers are non-negative.
       The $i$th (0-indexed) of these integers describes the $i$th subtask of this method.
       The value of the $i$th integer indicates the task of this subtask.
       The fourth line contains an odd number of integers.
       The last integer is always $-1$, while the other integers are non-negative.
       The number of non-negative integers is thus always even.
       Hence this list of non-negative integers can be interpreted as sequence of pairs $\langle (o_1^-,o_1^+), \dots, (o_n^-,o_n^+) \rangle$.
       Each such pair $(o_i^-,o_i^+)$ describes an ordering constraint on the methods subtasks.
       More precisely, each such pair stipulates that the $o_i^-$th subtask must occur before the $o_i^+$th subtask.
       No guarantees are made w.r.t. to the question whether the set of ordering constraints is transitively closed or whether it contains
       contains transitively implied ordering.
     */

    int method_id = 0;
    while (std::getline(file, line) && !line.empty())
    {
        std::string method_name = line;
        Log::i("Method name: %s\n", method_name.c_str());
        ++line_idx;

        std::vector<int> abstract_task_ids = parseIntegerList(file, line_idx);

        if (abstract_task_ids.empty())
        {
            Log::e("Error: No abstract task ID found for method %s\n", method_name.c_str());
            return;
        }
        if (abstract_task_ids.size() > 1)
        {
            Log::e("Error: Multiple abstract task IDs found for method %s\n", method_name.c_str());
            return;
        }

        int abstract_task_id = abstract_task_ids[0];
        std::vector<int> subtasks_ids = parseIntegerList(file, line_idx);

        std::vector<std::pair<int, int>> ordering_constains;
        std::getline(file, line);
        std::istringstream iss(line);
        int value1, value2;
        while (iss >> value1)
        {
            if (value1 == -1)
                break;
            iss >> value2;
            ordering_constains.emplace_back(value1, value2);
        }
        ++line_idx;
        sortSubtasks(subtasks_ids, ordering_constains);

        _methods.emplace_back(method_id, method_name, abstract_task_id, subtasks_ids);

        // Add the method to the abstract task
        _abstr_tasks[abstract_task_id - _actions.size()].addDecompositionMethod(method_id);
        ++method_id;
    }
}

void HtnInstance::sortSubtasks(std::vector<int> &subtasks_id, std::vector<std::pair<int, int>> &ordering_constains)
{
    // Create a map to store the index of each subtask
    std::unordered_map<int, int> subtasks_idx;
    for (int i = 0; i < subtasks_id.size(); i++)
    {
        subtasks_idx[subtasks_id[i]] = i;
    }

    // Sort the subtasks according to the ordering constraints
    for (int i = 0; i < ordering_constains.size(); i++)
    {
        int subtask1_id = ordering_constains[i].first;
        int subtask2_id = ordering_constains[i].second;
        int subtask1_idx = subtasks_idx[subtask1_id];
        int subtask2_idx = subtasks_idx[subtask2_id];
        if (subtask1_idx > subtask2_idx)
        {
            // Swap the subtasks
            int tmp = subtasks_id[subtask1_idx];
            subtasks_id[subtask1_idx] = subtasks_id[subtask2_idx];
            subtasks_id[subtask2_idx] = tmp;
            // Update the index of the subtasks
            subtasks_idx[subtask1_id] = subtask2_idx;
            subtasks_idx[subtask2_id] = subtask1_idx;
        }
    }
}

Parameters& HtnInstance::getParams() const
{
    return _params;
}

const AbstractTask& HtnInstance::getAbstractTaskById(int task_id) const
{
    return _abstr_tasks[task_id - _actions.size()];
}

const Action& HtnInstance::getActionById(int action_id) const
{
    return _actions[action_id];
}

const Method& HtnInstance::getMethodById(int method_id) const
{
    return _methods[method_id];
}

const Predicate& HtnInstance::getPredicateById(int pred_id) const
{
    return _predicates[pred_id];
}

const AbstractTask& HtnInstance::getRootTask() const
{
    return _abstr_tasks[_root_task_idx - _actions.size()];
}
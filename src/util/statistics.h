#ifndef STATISTICS_H
#define STATISTICS_H

#include <vector>
#include <map>
#include <algorithm>
#include <chrono>
#include <assert.h>

#include "util/log.h"

// Parts of the encoding. Used to count how many clauses (and literals) each part produces.
enum EncodingStage
{
    STAGE_INIT_GOAL_ROOT,
    STAGE_ACTIONS,
    STAGE_PRIMITIVENESS,
    STAGE_FRAME_AXIOMS,
    STAGE_AT_MOST_ONE_ACTION,
    STAGE_AT_MOST_ONE_METHOD,
    STAGE_HIERARCHY,
    STAGE_MUTEXES,
    NUM_STAGES
};

enum class TimingStage
{
    INIT_GROUNDING,
    INIT_MUTEXES,
    PLANNER,
    EXPANSION,
    ENCODING,
    SOLVER,
    ENCODING_MUTEXES,
    TOTAL
};

class Statistics
{
public:
    /**
     * Return the singleton instance of Statistics.
     * Ensures only one instance exists in the entire program.
     */
    static Statistics &getInstance()
    {
        static Statistics instance; // Constructed only once
        return instance;
    }

    // Deleted copy/assignment to enforce singleton property
    Statistics(const Statistics &) = delete;
    Statistics &operator=(const Statistics &) = delete;

    // Start counting the clauses of an encoding stage (stages are not nested)
    void begin(EncodingStage stage)
    {
        assert(_current_stage == NUM_STAGES);
        _current_stage = stage;
        _num_cls_at_stage_start = _num_cls;
        _num_lits_at_stage_start = _num_lits;
    }

    // Stop counting the clauses of an encoding stage
    void end(EncodingStage stage)
    {
        assert(_current_stage == stage);
        _num_cls_per_stage[stage] += _num_cls - _num_cls_at_stage_start;
        _num_lits_per_stage[stage] += _num_lits - _num_lits_at_stage_start;
        _current_stage = NUM_STAGES;
    }

    // Print a summary of stages and timing
    void printStats()
    {
        Log::i("Total amount of clauses encoded: %lld (%lld literals)\n", _num_cls, _num_lits);

        // Print the clauses and literals of each part of the encoding
        for (int stage = 0; stage < NUM_STAGES; stage++)
        {
            if (_num_cls_per_stage[stage] == 0)
                continue;
            Log::i("- %-22s : %12lld cls (%5.1f%%) %13lld lits (%5.1f%%)\n", STAGES_NAMES[stage],
                   _num_cls_per_stage[stage], 100.0 * _num_cls_per_stage[stage] / std::max(1LL, _num_cls),
                   _num_lits_per_stage[stage], 100.0 * _num_lits_per_stage[stage] / std::max(1LL, _num_lits));
        }

        // Print timing statistics
        if (!_stage_times_ms.empty())
        {
            for (const auto &[stage, time] : _stage_times_ms)
            {
                Log::i("* %s : %lli ms\n", toString(stage), time);
            }
        }

        // Warn if some timing stages were not closed
        if (!_active_timings.empty())
        {
            Log::w("\nWarning: Some timing stages were not properly closed:\n");
            for (const auto &[stage, _] : _active_timings)
            {
                Log::w("* %s\n", toString(stage));
            }
        }
    }

    void beginTiming(TimingStage stage)
    {
        if (_active_timings.count(stage) > 0)
        {
            Log::w("Warning: Attempted to start timing for stage %s which is already running\n",
                   toString(stage));
            return;
        }
        _active_timings[stage] = std::chrono::high_resolution_clock::now();
    }

    void endTiming(TimingStage stage)
    {
        auto it = _active_timings.find(stage);
        if (it == _active_timings.end())
        {
            Log::w("Warning: Attempted to end timing for stage %s which was not started\n",
                   toString(stage));
            return;
        }

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - it->second)
                            .count();
        _stage_times_ms[stage] += duration;
        _active_timings.erase(it);
    }

    long long getTiming(TimingStage stage)
    {
        return _stage_times_ms[stage];
    }

    // Utility to convert a TimingStage enum to string
    static const char *toString(TimingStage stage)
    {
        switch (stage)
        {
        case TimingStage::EXPANSION:
            return "time expansion";
        case TimingStage::ENCODING:
            return "time encoding";
        case TimingStage::SOLVER:
            return "time solver";
        case TimingStage::INIT_GROUNDING:
            return "time grounding";
        case TimingStage::INIT_MUTEXES:
            return "time compute mutexes";
        case TimingStage::PLANNER:
            return "time planner";
        case TimingStage::ENCODING_MUTEXES:
            return "time encoding mutexes";
        case TimingStage::TOTAL:
            return "time total";
        default:
            return "UNKNOWN_TIMING_STAGE";
        }
    }

    // Public data members (if needed externally)
    long long _num_cls = 0;
    long long _num_lits = 0;
    int _num_asmpts = 0;

private:
    // Private constructor for singleton
    Statistics() {}

    // Destructor prints the final stats automatically
    ~Statistics()
    {
    }

private:
    // Stage names (same order as EncodingStage)
    const char *STAGES_NAMES[NUM_STAGES] = {
        "init/goal state, root", "action prec. & effects", "primitiveness", "frame axioms",
        "at most one action", "at most one method", "hierarchy", "mutexes"};

    // Clauses and literals added by each stage
    long long _num_cls_per_stage[NUM_STAGES] = {};
    long long _num_lits_per_stage[NUM_STAGES] = {};

    // Stage currently counted (NUM_STAGES if none)
    EncodingStage _current_stage = NUM_STAGES;
    long long _num_cls_at_stage_start = 0;
    long long _num_lits_at_stage_start = 0;

    // Timing-related members
    std::map<TimingStage, std::chrono::time_point<std::chrono::high_resolution_clock>> _active_timings;
    std::map<TimingStage, long long> _stage_times_ms;
};

#endif // STATISTICS_H

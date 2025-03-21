#include "algo/planner.h"

int Planner::findPlan()
{
    // Initialize Tree

    // Encode the inital layer

    // Run main loop
    bool solved = false;
    int max_depth = 100;
    int current_depth = 0;
    while (!solved && current_depth < max_depth)
    {
        current_depth++;
        // Expand the Tree with a new layer

        // Encode the new layer

        // Launch the SAT solver

    }

    // If solved, extract the plan and verify it


    return 0;
}
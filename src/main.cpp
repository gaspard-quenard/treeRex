
#include <iostream>
#include <assert.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <sys/wait.h>
#include <exception>
#include <execinfo.h>
#include <signal.h>


#include "util/log.h"
#include "util/params.h"
#include "util/timer.h"
#include "util/signal_manager.h"
#include "util/statistics.h"
#include "data/htn_instance.h"
#include "algo/planner.h"

#ifndef TREEREX_VERSION
#define TREEREX_VERSION "(dbg)"
#endif

#ifndef IPASIRSOLVER
#define IPASIRSOLVER "(unknown)"
#endif


const char* TREEREX_ASCII = 
" A#######B                      A######B                \n"
"    A#B    A#####B  A######B A######B A#B     A#B A######B A#B    A#B \n"
"    A#B    A#B    A#B A#B      A#B      A#B     A#B A#B       A#B  A#B  \n"
"    A#B    A#B    A#B A#####B  A#####B  A######B  A#####B    A##B   \n"
"    A#B    A#####B  A#B      A#B      A#B   A#B   A#B        A##B   \n"
"    A#B    A#B   A#B  A#B      A#B      A#B    A#B  A#B       A#B  A#B  \n"
"    A#B    A#B    A#B A######B A######B A#B     A#B A######B A#B    A#B \n";
                                                

void outputBanner(bool colors) {

    const char* ascii = TREEREX_ASCII;

    for (size_t i = 0; i < strlen(ascii); i++) {
        char c = ascii[i];
        switch (c) {
        case 'A': if (colors) std::cout << Modifier(Code::FG_GREEN).str(); break;
        case 'B': if (colors) std::cout << Modifier(Code::FG_CYAN).str(); break;
        case 'C': if (colors) std::cout << Modifier(Code::FG_LIGHT_BLUE).str(); break;
        case 'D': if (colors) std::cout << Modifier(Code::FG_LIGHT_YELLOW).str(); break;
        default : std::cout << std::string(1, c);
        }
    }
    std::cout << Modifier(Code::FG_DEFAULT).str();
}



void handleSignal(int signum) {
    SignalManager::signalExit();
}

void run(Parameters& params) {

    Statistics::getInstance().beginTiming(TimingStage::TOTAL);

    // Parse, ground the problem and create the relevant HDDL structures
    HtnInstance htn(params);

    // Create the planner and find a plan
    Planner planner(htn);
    int result = planner.findPlan();

    Statistics::getInstance().endTiming(TimingStage::TOTAL);
    Statistics::getInstance().printStats();

    if (result == 0 && !params.isNonzero("cleanup")) {
        // Exit directly -- avoid to clean up :)
        Log::i("Exiting happily (no cleaning up).\n");
        exit(result);
    }
    Log::i("Exiting happily.\n");
    return;
}

int main(int argc, char** argv) {
    
    signal(SIGTERM, handleSignal);
    signal(SIGINT, handleSignal);

    Timer::init();

    Parameters params;
    params.init(argc, argv);

    int verbosity = params.getIntParam("v");
    Log::init(verbosity, /*coloredOutput=*/params.isNonzero("co"));

    if (verbosity >= Log::V2_INFORMATION) {
        outputBanner(params.isNonzero("co"));
        Log::log_notime(Log::V0_ESSENTIAL, "T r e e R e x");
        Log::log_notime(Log::V0_ESSENTIAL, "  version %s\n", TREEREX_VERSION);
        Log::log_notime(Log::V0_ESSENTIAL, "using SAT solver %s\n", IPASIRSOLVER);
        Log::log_notime(Log::V0_ESSENTIAL, "\n");
        
    }

    if (params.isSet("h") || params.isSet("help")) {
        params.printUsage();
        exit(0);
    }

    if (params.getProblemFilename() == "") {
        Log::w("Please specify both a domain file and a problem file. Use -h for help.\n");
        exit(1);
    }

    run(params);
    return 0;
}
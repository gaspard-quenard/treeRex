#include "util/command_utils.h"
#include "util/log.h"
#include <cstdlib>

int runCommand(const std::string &command, const std::string &error_message)
{
    Log::i("Executing command: %s\n", command.c_str());
    int ret = std::system(command.c_str());
    if (ret != 0)
    {
        Log::e("Error: %s\n", error_message.c_str());
    }
    return ret;
}

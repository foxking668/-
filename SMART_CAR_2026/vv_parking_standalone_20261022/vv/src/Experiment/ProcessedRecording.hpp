#pragma once
#include "VideoRecorder.hpp"
#include <string>
namespace Experiment {
// Linux resolves /proc/self/exe, so output is beside the binary even when invoked through ./vvv.
std::string executableDirectory();
bool startProcessedRecording(const std::string& parameterPath,std::string& error);
}

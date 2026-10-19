#pragma once

#include "RealCarExperiment.hpp"
#include <istream>
#include <ostream>
#include <string>
#include <array>

namespace Experiment {
struct TerminalSelection {
    std::string label;
    int speed = 6;
    int group = 1;
    int method = 4;
};

struct TerminalHistory {
    unsigned long long next = 1;
    unsigned long long logDirectories = 0;
};

struct DetailedOutcome {
    RunOutcome legacy;
    int completedLaps = -1;
    int lineTouchCount = -1;
    int repositionCount = -1;
    bool confirmed = false;
};

bool parseTerminalSelection(const std::string &label, int speed, TerminalSelection &selection);
TerminalHistory terminalHistory(const std::string &logRoot, const TerminalSelection &selection);
std::array<int, 4> terminalMethodOrder(int speed, unsigned long long round);
struct TerminalRoundPlan {
    unsigned long long round = 1;
    std::array<int, 4> order{{1,2,3,4}};
    std::array<TerminalHistory, 4> histories;
    int nextMethod = 1;
};
TerminalRoundPlan terminalRoundPlan(const std::string &logRoot, int speed);
void printTerminalRoundGuide(std::ostream &output, const std::string &logRoot,
                             int speed, int selectedMethod = 0);
bool prepareTerminalSession(std::istream &input, std::ostream &output,
                            const std::string &paramsPath, const std::string &logRoot);
DetailedOutcome readDetailedOutcome(std::istream &input, std::ostream &output);
void saveDetailedOutcome(const std::string &runDirectory, const std::string &runId,
                         const DetailedOutcome &outcome);
}

#pragma once

#include <cctype>
#include <string>
#include <stdexcept>

namespace Experiment
{
    inline int parseExperimentInteger(const std::string &text)
    {
        std::size_t end = 0;
        const int value = std::stoi(text, &end);
        if (end != text.size()) throw std::invalid_argument("expected an integer, not a method label or trailing text");
        return value;
    }

    inline int parseExperimentSwitch(const std::string &text)
    {
        const int value = parseExperimentInteger(text);
        if (value != 0 && value != 1) throw std::invalid_argument("expected 0 or 1");
        return value;
    }

    inline bool isExperimentMethodValid(int method, int group = 0)
    {
        return group == 0 ? (method == 3 || method == 4)
                          : (group == 1 && method >= 1 && method <= 4);
    }

    inline bool isRunIdValid(const std::string &runId)
    {
        if (runId.empty() || runId.size() > 64)
        {
            return false;
        }

        for (unsigned char ch : runId)
        {
            if (!(std::isalnum(ch) || ch == '_' || ch == '-'))
            {
                return false;
            }
        }
        return true;
    }
}

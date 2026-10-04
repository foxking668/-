#pragma once
#include "core.hpp"

namespace car2026 {
// Incremental parser is also tested on Windows. No vendor SDK is required.
class ImuParser {
public:
    explicit ImuParser(std::string format) : format_(std::move(format)) {}
    std::vector<double> feed(const uint8_t* data,size_t count);
private:
    std::string format_,text_;
    std::vector<uint8_t> bytes_;
};
} // namespace car2026

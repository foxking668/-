#pragma once
#include <cerrno>
#include <cstring>
#include <string>
namespace ZebraTest { namespace IoStatus {
// Driver errors are captured in the same thread, then copied into event records.
inline thread_local std::string lastError;
inline void set(const std::string& operation,const std::string& path,int code) {
    lastError=operation+" path="+path+" errno="+std::to_string(code)+" "+(code?std::strerror(code):"stream failure without errno");
}
} }

#pragma once
#include "IoStatus.hpp"
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

namespace ZebraTest { namespace SysfsIo {
// Never create a missing node. Detect errors from write AND close, before errno is lost.
inline bool writeValue(const std::string& path, const std::string& value) {
    int fd;
    do { fd = ::open(path.c_str(), O_WRONLY | O_TRUNC); } while (fd < 0 && errno == EINTR);
    if (fd < 0) { IoStatus::set("SYSFS_OPEN", path, errno); return false; }
    decltype(::write(fd, value.data(), value.size())) written;
    do { written = ::write(fd, value.data(), value.size()); } while (written < 0 && errno == EINTR);
    const int writeError = written < 0 ? errno : (written == static_cast<decltype(written)>(value.size()) ? 0 : EIO);
    const int closed = ::close(fd);
    const int closeError = closed < 0 ? errno : 0;
    if (writeError || closeError) {
        IoStatus::set(writeError ? "SYSFS_WRITE" : "SYSFS_CLOSE", path, writeError ? writeError : closeError);
        IoStatus::lastError += " requested=" + value;
        return false;
    }
    return true;
}
} }

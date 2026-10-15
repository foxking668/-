#pragma once

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#else
#include <dirent.h>
#endif

// Small directory API shared by terminal startup and its hardware-free host tests.
namespace SessionFiles {
class path {
    std::string value_;
public:
    path() = default;
    path(const char *value) : value_(value) {}
    path(std::string value) : value_(std::move(value)) {}
    const std::string &string() const { return value_; }
    path filename() const { return value_.substr(value_.find_last_of("/\\") + 1); }
    path parent() const {
        const auto slash = value_.find_last_of("/\\");
        return slash == std::string::npos ? path{} : path(value_.substr(0, slash));
    }
    path operator/(const path &other) const { return value_.empty() ? other : path(value_ + "/" + other.value_); }
    bool operator==(const path &other) const { return value_ == other.value_; }
};
inline bool exists(const path &p) {
    struct stat data = {};
    if (stat(p.string().c_str(), &data) == 0) return true;
    if (errno == ENOENT) return false;
    throw std::runtime_error("Cannot inspect path: " + p.string());
}
inline bool is_directory(const path &p) {
    struct stat data = {};
    if (stat(p.string().c_str(), &data) != 0) throw std::runtime_error("Cannot inspect directory: " + p.string());
    return (data.st_mode & S_IFDIR) != 0;
}
inline bool create_directory(const path &p) {
#ifdef _WIN32
    const int result = _mkdir(p.string().c_str());
#else
    const int result = mkdir(p.string().c_str(), 0755);
#endif
    if (result == 0) return true;
    if (errno == EEXIST && is_directory(p)) return false;
    throw std::runtime_error("Cannot create directory: " + p.string());
}
inline void create_directories(const path &p) {
    if (p.string().empty()) return;
    if (exists(p)) {
        if (!is_directory(p)) throw std::runtime_error("Not a directory: " + p.string());
        return;
    }
    create_directories(p.parent());
    create_directory(p);
}
class entry {
    SessionFiles::path path_;
public:
    explicit entry(SessionFiles::path p) : path_(std::move(p)) {}
    const SessionFiles::path &path() const { return path_; }
};
inline std::vector<entry> directory_iterator(const path &root) {
    std::vector<entry> entries;
#ifdef _WIN32
    _finddata_t data = {};
    const auto handle = _findfirst((root / "*").string().c_str(), &data);
    if (handle == -1) {
        if (errno == ENOENT && is_directory(root)) return entries;
        throw std::runtime_error("Cannot enumerate: " + root.string());
    }
    do {
        const std::string name = data.name;
        if (name != "." && name != "..") entries.emplace_back(root / name);
    } while (_findnext(handle, &data) == 0);
    const int scanError = errno;
    _findclose(handle);
    if (scanError != ENOENT) throw std::runtime_error("Directory enumeration failed: " + root.string());
#else
    DIR *directory = opendir(root.string().c_str());
    if (!directory) throw std::runtime_error("Cannot enumerate: " + root.string());
    int scanError = 0;
    while (true) {
        errno = 0;
        dirent *item = readdir(directory);
        if (!item) { scanError = errno; break; }
        const std::string name = item->d_name;
        if (name != "." && name != "..") entries.emplace_back(root / name);
    }
    closedir(directory);
    if (scanError) throw std::runtime_error("Directory enumeration failed: " + root.string());
#endif
    return entries;
}
inline void remove(const path &p) {
    if (std::remove(p.string().c_str()) != 0) throw std::runtime_error("Cannot remove: " + p.string());
}
inline void rename(const path &from, const path &to) {
    if (std::rename(from.string().c_str(), to.string().c_str()) != 0) throw std::runtime_error("Cannot rename test directory.");
}
inline path temp_directory_path() {
#ifdef _WIN32
    const char *value = std::getenv("TEMP");
    if (!value) throw std::runtime_error("TEMP is unset.");
    return value;
#else
    return "/tmp";
#endif
}
}

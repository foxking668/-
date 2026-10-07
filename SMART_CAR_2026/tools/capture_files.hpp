#pragma once
#include "capture_data.hpp"
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
namespace car2026 { namespace capture {
namespace fs=std::filesystem;
inline std::ofstream outputFile(const fs::path& path) {
    std::ofstream file(path.string());if(!file) throw std::runtime_error("Cannot create "+path.string());
    file.exceptions(std::ios::failbit|std::ios::badbit);file<<std::setprecision(17);return file;
}
inline uint64_t nonemptyFileSize(const fs::path& path) {
    struct stat info{};
    if(stat(path.string().c_str(),&info)!=0)
        throw std::runtime_error("Cannot stat "+path.string()+": "+std::strerror(errno));
    if(info.st_size<=0) throw std::runtime_error("Output file is empty: "+path.string());
    return uint64_t(info.st_size);
}
inline void syncFile(const fs::path& path) {
    const int fd=open(path.string().c_str(),O_RDWR|O_CLOEXEC);
    if(fd<0) throw std::runtime_error("Cannot sync-open "+path.string()+": "+std::strerror(errno));
    int result;do {result=fsync(fd);} while(result<0 && errno==EINTR);
    const int error=errno;const int closed=close(fd);
    if(result<0) throw std::runtime_error("fsync "+path.string()+": "+std::strerror(error));
    if(closed<0) throw std::runtime_error("Close after fsync failed: "+path.string());
}
inline void closeText(std::ofstream& file,const fs::path& path) {
    file.flush();file.close(); // Exceptions are enabled, including delayed flush/close failures.
    nonemptyFileSize(path);syncFile(path);
}
inline uint64_t verifyTextFile(const fs::path& path,const std::string& firstLine) {
    nonemptyFileSize(path);
    std::ifstream input(path.string(),std::ios::binary);
    if(!input) throw std::runtime_error("Cannot reopen output: "+path.string());
    try {return car2026::capture::verifyTextOutput(input,firstLine);}
    catch(const std::exception& error) {throw std::runtime_error(path.string()+": "+error.what());}
}
}}

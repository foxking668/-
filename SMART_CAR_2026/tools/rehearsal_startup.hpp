#pragma once
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
namespace car2026 { namespace capture {
// One successful preparation permits one subsequent invocation of the same
// stage/profile in this boot. It never authorizes a motor/servo stage action.
class RehearsalStartupGate {
public:
    RehearsalStartupGate(std::filesystem::path marker,std::string identity)
        :marker_(std::move(marker)),identity_(std::move(identity)) {}
    bool consumePrepared() const {
        namespace fs=std::filesystem;
        if(!fs::exists(marker_)) return false;
        if(fs::is_symlink(fs::symlink_status(marker_)) || !fs::is_regular_file(marker_))
            throw std::runtime_error("Invalid startup preparation marker");
        std::ifstream input(marker_,std::ios::binary);
        if(!input) throw std::runtime_error("Cannot read startup preparation marker");
        std::string saved((std::istreambuf_iterator<char>(input)),{});
        if(input.bad()) throw std::runtime_error("Cannot read startup preparation marker");
        input.close();
        if(!fs::remove(marker_)) throw std::runtime_error("Cannot consume startup preparation marker");
        return saved==identity_;
    }
    void markPrepared() const {
        namespace fs=std::filesystem;
        if(fs::exists(marker_)) throw std::runtime_error("Startup preparation marker already exists");
        const auto pending=fs::path(marker_.string()+".pending");
        if(fs::exists(pending) || fs::is_symlink(fs::symlink_status(pending)))
            throw std::runtime_error("Incomplete startup preparation marker; remove .pending before retrying");
        std::ofstream output(pending,std::ios::binary);output.exceptions(std::ios::failbit|std::ios::badbit);
        output<<identity_;output.close();fs::rename(pending,marker_);
    }
private:
    std::filesystem::path marker_;std::string identity_;
};
}}

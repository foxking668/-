#pragma once
#include "capture_data.hpp"
#include <filesystem>
namespace car2026 { namespace capture {
enum class SaveChoice {Yes,No,Invalid};
inline SaveChoice parseSaveChoice(const std::vector<std::string>& lines) {
    if(lines.size()!=1) return SaveChoice::Invalid;
    const auto value=trimmed(lines.front());
    if(value=="Y" || value=="y") return SaveChoice::Yes;
    if(value=="N" || value=="n") return SaveChoice::No;
    return SaveChoice::Invalid;
}
// N can remove only the exclusively created session, verified by parent and ownership token.
class SessionFiles {
public:
    explicit SessionFiles(const std::filesystem::path& directory):directory_(std::filesystem::canonical(directory)),root_(directory_.parent_path()),token_(directory_.filename().string()) {
        if(token_.rfind("rehearsal_",0)!=0) throw std::runtime_error("Invalid owned session name");
        const auto marker=directory_/".pending_session";
        if(std::filesystem::exists(marker)) throw std::runtime_error("Session ownership marker already exists");
        std::ofstream file(marker,std::ios::binary);file.exceptions(std::ios::failbit|std::ios::badbit);file<<token_;file.close();
    }
    void discard() const {
        verifyOwnership();std::filesystem::remove_all(directory_);
    }
    void saved() const {verifyOwnership();std::filesystem::remove(directory_/".pending_session");}
    const std::filesystem::path& directory() const {return directory_;}
private:
    void verifyOwnership() const {
        namespace fs=std::filesystem;
        if(fs::is_symlink(fs::symlink_status(directory_)) || fs::canonical(directory_)!=directory_ ||
           fs::canonical(directory_.parent_path())!=root_ || directory_.parent_path()!=root_)
            throw std::runtime_error("Session path changed; refusing removal");
        const auto marker=directory_/".pending_session";
        if(fs::is_symlink(fs::symlink_status(marker)) || !fs::is_regular_file(marker))
            throw std::runtime_error("Session ownership marker missing or replaced");
        std::ifstream file(marker,std::ios::binary);std::string token;
        std::getline(file,token);
        if(token!=token_ || file.bad() || file.peek()!=std::char_traits<char>::eof())
            throw std::runtime_error("Session ownership token mismatch");
    }
    std::filesystem::path directory_,root_;std::string token_;
};
}}

#pragma once
#include <cerrno>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
namespace car2026 { namespace capture {
// Temporary mode for the final Y/N prompt only. Keep signal handling enabled.
class SaveConfirmationTerminal {
public:
    explicit SaveConfirmationTerminal(int fd):fd_(fd) {
        if(tcgetattr(fd_,&original_)!=0) throw std::runtime_error("Cannot read save confirmation terminal mode");
        auto immediate=original_;
        immediate.c_lflag&=static_cast<tcflag_t>(~(ICANON|ECHO|ECHONL));
        immediate.c_cc[VMIN]=1;immediate.c_cc[VTIME]=0;
        // Discard controls typed before the prompt. Do not authorize from queued input.
        if(applyMode(immediate)!=0) throw std::runtime_error("Cannot enable single-key save confirmation");
        active_=true;
    }
    ~SaveConfirmationTerminal() noexcept {
        if(active_ && applyMode(original_)!=0)
            std::cerr<<"Cannot restore terminal mode after save confirmation\n";
    }
    SaveConfirmationTerminal(const SaveConfirmationTerminal&)=delete;
    SaveConfirmationTerminal& operator=(const SaveConfirmationTerminal&)=delete;
    char readKey() const {
        pollfd input{fd_,POLLIN,0};
        int ready;do {ready=poll(&input,1,-1);} while(ready<0 && errno==EINTR);
        if(ready<0 || input.revents&(POLLERR|POLLHUP|POLLNVAL))
            throw std::runtime_error("Save confirmation terminal disconnected or poll failed");
        char key;ssize_t size;
        do {size=read(fd_,&key,1);} while(size<0 && errno==EINTR);
        if(size!=1 || key=='\x04') throw std::runtime_error("Save confirmation terminal EOF or read failed");
        return key;
    }
    void restore() {
        if(active_ && applyMode(original_)!=0)
            throw std::runtime_error("Cannot restore terminal mode after save confirmation");
        active_=false;
    }
private:
    int applyMode(const termios& mode) const {
        int result;do {result=tcsetattr(fd_,TCSAFLUSH,&mode);} while(result<0 && errno==EINTR);
        return result;
    }
    int fd_;termios original_{};bool active_=false;
};
}}

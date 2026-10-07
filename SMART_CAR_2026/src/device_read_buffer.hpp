#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace car2026 {
// Small factory reads must not point directly at a one/two-byte stack scalar.
// Some incorrect drivers ignore the requested count. Padding detects modest
// over/underwrites before their bytes can corrupt neighboring stack objects.
// This is a diagnostic guard, not containment of arbitrary kernel memory bugs.
template<class T> class GuardedDeviceRead {
    static_assert(std::is_trivially_copyable<T>::value,"Binary read needs a POD-like value");
public:
    static constexpr std::size_t guardBytes=32;
    using Padding=std::array<uint8_t,guardBytes>;
    explicit GuardedDeviceRead(const T& initial,uint8_t guardValue=0xa5) : guardValue_(guardValue) {
        bytes_.fill(guardValue_);
        std::memcpy(bytes_.data()+guardBytes,&initial,sizeof(T));
    }
    void* data() {return bytes_.data()+guardBytes;}
    Padding paddingBefore() const {return paddingAt(0);}
    Padding paddingAfter() const {return paddingAt(guardBytes+sizeof(T));}
    bool guardDamaged() const {
        for(std::size_t i=0;i<guardBytes;++i)
            if(bytes_[i]!=guardValue_ || bytes_[guardBytes+sizeof(T)+i]!=guardValue_) return true;
        return false;
    }
    T value(const std::string& path) const {
        if(guardDamaged())
            throw std::runtime_error("Device read wrote outside requested "+std::to_string(sizeof(T))+
                                     "-byte payload: '"+path+"'");
        T result{};std::memcpy(&result,bytes_.data()+guardBytes,sizeof(T));return result;
    }
private:
    Padding paddingAt(std::size_t offset) const {
        Padding result{};std::memcpy(result.data(),bytes_.data()+offset,result.size());return result;
    }
    uint8_t guardValue_;
    alignas(T) std::array<uint8_t,sizeof(T)+2*guardBytes> bytes_{};
};
} // namespace car2026

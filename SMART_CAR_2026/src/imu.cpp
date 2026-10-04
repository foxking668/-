#include "imu.hpp"

namespace car2026 {
std::vector<double> ImuParser::feed(const uint8_t* data,size_t count) {
    std::vector<double> angles;
    if(format_=="ascii") {
        for(size_t i=0;i<count;++i) {
            const char c=static_cast<char>(data[i]);
            if(c=='\n') {
                if(text_.rfind("YAW,",0)==0) try {
                    size_t end=0;const auto value=text_.substr(4);double yaw=std::stod(value,&end);
                    if(end==value.size() && std::isfinite(yaw) && yaw>=-180 && yaw<=180) angles.push_back(yaw);
                } catch(const std::exception&) {}
                text_.clear();
            } else if(c!='\r') { text_+=c;if(text_.size()>128) text_.clear(); }
        }
    } else {
        bytes_.insert(bytes_.end(),data,data+count);
        while(bytes_.size()>=11) {
            if(bytes_[0]!=0x55) {bytes_.erase(bytes_.begin());continue;}
            uint8_t checksum=0;for(int i=0;i<10;++i) checksum=uint8_t(checksum+bytes_[i]);
            if(checksum!=bytes_[10]) {bytes_.erase(bytes_.begin());continue;}
            if(bytes_[1]==0x53) {
                int raw=bytes_[6]|(int(bytes_[7])<<8);if(raw>=32768) raw-=65536;
                angles.push_back(raw/32768.*180.);
            }
            bytes_.erase(bytes_.begin(),bytes_.begin()+11);
        }
        if(bytes_.size()>1024) bytes_.clear();
    }
    return angles;
}
} // namespace car2026

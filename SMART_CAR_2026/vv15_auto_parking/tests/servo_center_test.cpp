#include "Parking/ServoCenter.hpp"
#include <filesystem>
#include <iostream>
#include <limits>
#include <vector>
using namespace AutoParking;
int checks=0;
void check(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
struct Motor {
    int left=99,right=99;bool failLeft=false,failRight=false;std::vector<int> writes;
    bool setLeftPwm(int v){left=v;writes.push_back(v);return !failLeft;}
    bool setRightPwm(int v){right=v;writes.push_back(v);return !failRight;}
};
struct Steer {
    Motor* motor;int writes=0;float offset=0,angle=0;bool fail=false;
    void setSteerOffset(float v){offset=v;}
    bool setAngle(float v){if(motor->left || motor->right)throw std::runtime_error("steering before both motor zeros");++writes;angle=v;return !fail;}
};
int main(int argc,char** argv){try{
    check(argc==2,"temp directory required");const std::filesystem::path root=argv[1];std::filesystem::create_directories(root);
    const auto path=(root/"image_params.txt").string();
    auto put=[&](const std::string& s){std::ofstream(path)<<s;};
    auto rejects=[&](const std::string& s){put(s);bool failed=false;try{ServoCenter::readOffset(path);}catch(...){failed=true;}check(failed,"invalid offset rejected");};
    put("# original unrelated parameters remain allowed\ncenter_pratio=0.65\nsteer_offset_deg=95\n");
    check(ServoCenter::readOffset(path)==95,"original offset read");
    put("\xEF\xBB\xBFsteer_offset_deg = -12.5 # note\r\n");check(ServoCenter::readOffset(path)==-12.5,"BOM/comment/decimal supported");
    rejects("steer_offset_deg=nan\n");rejects("steer_offset_deg=151\n");rejects("steer_offset_deg=-151\n");
    rejects("steer_offset_deg=95degrees\n");rejects("steer_offset_deg=1\nsteer_offset_deg=2\n");rejects("center_pratio=.65\n");
    Motor motor;Steer steer{&motor};
    check(ServoCenter::apply(&motor,&steer,95,0),"authorized center write");
    check(steer.angle==90 && steer.offset==95,"original 90+command adapter and existing offset");
    check(ServoCenter::apply(&motor,&steer,90,-5) && steer.angle==85,"small negative direction probe");
    check(ServoCenter::apply(&motor,&steer,90,5) && steer.angle==95,"small positive direction probe");
    const int prior=steer.writes;
    check(!ServoCenter::apply(&motor,&steer,90,40) && steer.writes==prior,"large probe refused");
    check(!ServoCenter::apply(&motor,&steer,151,0) && steer.writes==prior,"invalid offset never written");
    check(!ServoCenter::apply(&motor,&steer,std::numeric_limits<double>::quiet_NaN(),0),"NaN refused");
    motor.failLeft=true;const auto count=motor.writes.size();
    check(!ServoCenter::apply(&motor,&steer,90,0) && motor.writes.size()==count+2 && steer.writes==prior,"left failure still zeros right and blocks servo");
    motor.failLeft=false;motor.failRight=true;
    check(!ServoCenter::apply(&motor,&steer,90,0) && steer.writes==prior,"right zero failure blocks servo");
    motor.failRight=false;steer.fail=true;check(!ServoCenter::apply(&motor,&steer,90,0),"servo error reported");
    for(const int value:motor.writes)check(value==0,"no nonzero motor value in calibration");
    check(!ServoCenter::apply<Motor,Steer>(nullptr,&steer,90,0),"null motor refused");
    check(!ServoCenter::apply<Motor,Steer>(&motor,nullptr,90,0),"null servo refused");
    std::cout<<"PASS "<<checks<<" stationary servo parser/output checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}

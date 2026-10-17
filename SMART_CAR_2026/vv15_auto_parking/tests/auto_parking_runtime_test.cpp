#include "Parking/AutoParkingRuntime.hpp"
#include "headfile.hpp"
#include <filesystem>
#include <fstream>
#include <stdexcept>
void check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(int argc,char** argv) {
    try {
        check(argc==2 || argc==3,"temporary test directory and optional manual scenario required");const bool manual=argc==3;std::filesystem::create_directories(argv[1]);
        std::filesystem::current_path(argv[1]);std::filesystem::create_directory("config");
        std::ofstream cfg("config/parking_tuning.vv.ini");
        cfg<<"[session]\n"<<(manual?"start_mode=manual\n":"")<<"enabled=1\ntrigger_zebra_count=2\nreverse_right_only=1\nline_lost_s=0.35\n";
        for(int i=1;i<=6;++i)cfg<<"[stage_"<<i<<"]\nsteer_command="<<(i==2?-40:0)
            <<"\nsettle_time_s=0.5\nhold_time_s=0\nmotor_pwm_ns="<<(i<3?9000:0)
            <<"\nmotor_run_time_s="<<(i<3?.12:0)<<'\n';
        cfg.close();std::ofstream("image_params.txt")<<"steer_offset_deg=95\n";
        AutoParking::initialize("config/parking_tuning.vv.ini");Contral::Motor m;Contral::Steer s;
        check(AutoParking::manualMode()==manual,"correct entry mode");
        if(manual) {
            check(AutoParking::takeoverActive() && AutoParking::awaitingManualStart(),"manual owns motors from initialization");
            AutoParking::publishLane(true,-15);AutoParking::motorStep(&m,&s,0,0,true);
            check(m.left==0 && m.right==0 && s.writes==0,"manual no motor or servo action before Enter");
            check(!std::filesystem::exists("captures"),"no parking session before authorization");
            AutoParking::zebraDetected(2);check(AutoParking::awaitingManualStart(),"zebra cannot authorize manual");
            check(AutoParking::authorizeManualStart(),"manual entry authorized");
            check(!AutoParking::authorizeManualStart(),"second authorization ignored");
            AutoParking::motorStep(&m,&s,0,0,true);
            check(m.left==0 && m.right==0,"pre-Enter frame cannot start motion");
        } else {
            AutoParking::zebraDetected(1);check(!AutoParking::takeoverActive(),"first zebra no handoff");
            AutoParking::zebraDetected(2);check(AutoParking::takeoverActive(),"second zebra handoff");
            AutoParking::publishLane(true,-15);AutoParking::motorStep(&m,&s,0,0,true);
            check(m.left==0 && m.right==0,"wait stop");
        }
        int writes=s.writes;
        AutoParking::writeCruiseSteering(&s,50);check(s.writes==writes,"late image cannot overwrite steering");
        AutoParking::publishLane(true,-15);AutoParking::motorStep(&m,&s,0,0,manual);
        check(m.left==9000 && m.right==9000 && s.angle==75,"real bridge uses raw pwm and original 90+error");
        std::this_thread::sleep_for(std::chrono::milliseconds(140));AutoParking::motorStep(&m,&s,2,2,false);
        check(m.left==0 && m.right==0 && !AutoParking::cameraNeeded(),"motor deadline ends camera stage");
        AutoParking::motorStep(&m,&s,0,0,false);check(s.angle==50,"first reverse steering");
        std::this_thread::sleep_for(std::chrono::milliseconds(510));AutoParking::motorStep(&m,&s,0,0,false);
        check(m.left==0 && m.right==-9000,"right-only output through bridge");
        m.failRight=true;AutoParking::motorStep(&m,&s,0,0,false);
        check(m.left==0 && m.right==0,"write failure cuts both motors");AutoParking::shutdown();
        std::size_t csv=0;for(auto& session:std::filesystem::directory_iterator("captures/parking_auto"))
            for(auto& f:std::filesystem::directory_iterator(session.path()))if(f.path().extension()==".csv")++csv;
        check(csv==7,"six stage files plus event log");std::cout<<"PASS runtime entry/acknowledgement/ownership/logging checks\n";return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}

#include "parking_simulation.hpp"
#include <iomanip>
#include <iostream>

int main(int argc,char** argv) {
    using namespace car2026::parking;
    try {
        if(argc==2&&std::string(argv[1])=="--help") {
            std::cout<<"parking_geometry_check --example [--csv FILE]\n"
                       "OFFLINE SYNTHETIC geometry/control example; no device access, no actuator output.\n"
                       "Actual radii, track dimensions and metric camera/encoder adapter are not calibrated.\n";return 0;
        }
        if(argc!=2&&argc!=4) throw std::runtime_error("Use --example [--csv FILE]; example is not a real parking calibration");
        if(std::string(argv[1])!="--example"||(argc==4&&std::string(argv[2])!="--csv")) throw std::runtime_error("Unknown arguments");
        std::ofstream csv;
        if(argc==4) {
            // Refuse an existing output; users choose a fresh diagnostic path.
            std::ifstream existing(argv[3],std::ios::binary);
            if(existing.good()) throw std::runtime_error("Output already exists; choose a new CSV path");
            csv.open(argv[3]);if(!csv) throw std::runtime_error("Cannot open output CSV");
            csv<<"time_s,forward_m,left_m,heading_rad,total_travel_m,phase,speed_request_m_s,steer_command_request\n"<<std::setprecision(17);
        }
        const auto g=exampleGeometry();const auto p=makePlan(g);const auto area=exampleArea(g);
        std::cout<<"SYNTHETIC EXAMPLE; NO HARDWARE. branch_deg="<<g.branchAngle*180/car2026::pi
                 <<" rear_extent_m="<<g.rearExtent<<" staging_forward_m="<<p.staging.forward
                 <<" branch_reverse_m="<<p.segments[1].length<<" swept_body_fits="<<fitsArea(p,area)<<'\n';
        const auto result=simulate(g,area,{},[&](const Feedback& f,const Decision& d){
            if(csv.is_open()) {
                csv<<f.time<<','<<f.pose.forward<<','<<f.pose.left<<','<<f.pose.heading<<','<<f.totalTravel<<','<<phaseName(d.phase)<<','<<d.speed<<',';
                if(d.steering) csv<<*d.steering;csv<<'\n';
            }
        });
        if(csv.is_open()) {csv.flush();if(!csv) throw std::runtime_error("Output write failed");}
        std::cout<<"result="<<(result.completed?"COMPLETE":"FAILED")<<" steps="<<result.steps<<" reason="<<result.fault<<'\n';
        return result.completed?0:1;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}

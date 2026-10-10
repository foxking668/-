#include "../tools/reference_cc_lane.hpp"
#include <iostream>
#include <limits>
using namespace car2026::capture;
int main() {
    int checks=0;
    try {
        auto check=[&](bool ok) {++checks;if(!ok) throw std::runtime_error("CC steering check failed");};
        for(double x: {0.,28.,33.6,34.,39.,79.}) {
            const double raw=cc_detail::centerToError(x,56,80,.42);
            check(std::abs(raw-std::atan2(x-33.6,28.)*180/std::acos(-1.)*4)<1e-10);
            CcLaneObservation obs;obs.line.reliable=true;obs.error=float(raw);
            for(double limit: {1.,8.,15.}) check(ccLaneServoCommand(obs,limit)==std::clamp(double(obs.error),-limit,limit));
        }
        check(cc_detail::centerToError(28,56,80,.42)<0);
        check(cc_detail::centerToError(39,56,80,.42)>0);
        CcLaneObservation invalid;
        bool rejected=false;try {ccLaneServoCommand(invalid,15);} catch(...) {rejected=true;}
        check(rejected);
        invalid.line.reliable=true;
        for(double limit: {0.,16.,std::numeric_limits<double>::quiet_NaN()}) {
            rejected=false;try {ccLaneServoCommand(invalid,limit);} catch(...) {rejected=true;}
            check(rejected);
        }
        invalid.error=std::numeric_limits<float>::infinity();
        rejected=false;try {ccLaneServoCommand(invalid,15);} catch(...) {rejected=true;}
        check(rejected);
        std::cout<<checks<<" CC steering checks passed\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}

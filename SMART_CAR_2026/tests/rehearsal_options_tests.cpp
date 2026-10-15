#include "../tools/rehearsal_options.hpp"
#include <iostream>
#include <vector>
using car2026::capture::RehearsalOptions;
namespace {
unsigned checks=0;
void check(bool result,const char* reason) {++checks;if(!result) throw std::runtime_error(reason);}
RehearsalOptions parse(std::initializer_list<const char*> values) {
    std::vector<std::string> text{"parking_rehearsal"};
    for(const auto* value:values) text.emplace_back(value);
    std::vector<char*> args;for(auto& value:text) args.push_back(value.data());
    return RehearsalOptions::parse(int(args.size()),args.data());
}
void rejects(std::initializer_list<const char*> values,const char* reason) {
    bool rejected=false;try {parse(values);}catch(const std::runtime_error&) {rejected=true;}
    check(rejected,reason);
}
}
int main() {
    try {
        check(!parse({}).noCamera,"legacy invocations retain camera acquisition");
        const auto direct=parse({"--sysfs-pwm-control","--no-camera","--allow-partial"});
        check(direct.noCamera && direct.pwmControl && direct.ccSpeed && direct.partial,"launcher selects camera-free sysfs speed mode");
        const auto preflight=parse({"--no-camera","--check-config","--hardware-config","temporary.ini"});
        check(preflight.noCamera && preflight.check && preflight.hardware=="temporary.ini","camera-free migration preflight stays check-only");
        const auto configured=parse({"--no-camera","--config","capture.ini","--vehicle-config","vehicle.ini","--tuning-config","tuning.ini","--output","records"});
        check(configured.capture=="capture.ini" && configured.vehicle=="vehicle.ini" && configured.tuning=="tuning.ini" && configured.output=="records","camera bypass preserves all input/output paths");
        check(parse({"--no-camera","--help"}).help,"camera-free help never starts a trial");
        rejects({"--no-camera","--no-camera"},"duplicate bypass flag rejected");
        rejects({"--no-camera","false"},"unexpected bypass value rejected");
        rejects({"--no-camera","--config"},"missing config path rejected");
        rejects({"--no-camera","--config","--check-config"},"option cannot become a file path");
        rejects({"--no-camera","--unknown"},"camera bypass never hides an unknown option");
        std::cout<<checks<<" rehearsal option checks passed (no hardware).\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}

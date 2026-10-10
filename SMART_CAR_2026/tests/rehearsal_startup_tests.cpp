#include "../tools/rehearsal_startup.hpp"
#include <chrono>
#include <iostream>
using namespace car2026::capture;
namespace fs=std::filesystem;
int checks=0;
void check(bool result,const char* message) {++checks;if(!result) throw std::runtime_error(message);}
int main() {
    const auto root=fs::temp_directory_path()/("rehearsal-startup-test-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try {
        fs::create_directory(root);const auto marker=root/"startup_ready";
        for(int stage=1;stage<=6;++stage) {
            RehearsalStartupGate gate(marker,"version/boot/profile/stage="+std::to_string(stage));
            check(!gate.consumePrepared(),"each stage starts with preparation only");
            gate.markPrepared();check(fs::is_regular_file(marker),"successful first exit saves preparation marker");
            check(gate.consumePrepared(),"second invocation enters formal trial");
            check(!fs::exists(marker),"formal trial consumes marker before motion");
            check(!gate.consumePrepared(),"next trial requires preparation again");
        }
        RehearsalStartupGate original(marker,"v1/boot1/profileA/stage2");
        for(const std::string identity:{"v1/boot1/profileA/stage4","v1/boot2/profileA/stage2","v1/boot1/profileB/stage2","v2/boot1/profileA/stage2"}) {
            original.markPrepared();RehearsalStartupGate changed(marker,identity);
            check(!changed.consumePrepared(),"changed stage, boot, profile or version requires fresh preparation");
            check(!fs::exists(marker),"stale preparation cannot later be reused");
        }
        check(!fs::exists(marker),"aborted initialization without markPrepared creates no marker");
        fs::create_directory(marker);bool rejected=false;
        try {original.consumePrepared();}catch(const std::exception&) {rejected=true;}
        check(rejected,"invalid marker cannot admit formal trial");fs::remove(marker);
        const auto pending=fs::path(marker.string()+".pending");std::ofstream(pending)<<"partial";
        rejected=false;try {original.markPrepared();}catch(const std::exception&) {rejected=true;}
        check(rejected,"incomplete marker cannot be mistaken for successful preparation");
        check(!original.consumePrepared(),"incomplete marker never permits second invocation");
        // Remove only this test's verified, exclusively created temp directory.
        fs::remove(pending);fs::remove(root);
        std::cout<<checks<<" startup-sequence checks passed (no hardware access).\n";
        return 0;
    }catch(const std::exception& error) {std::cerr<<error.what()<<"; test files retained at "<<root<<'\n';return 1;}
}

// Existing lane tests exercise VV15 race mode with the new parking feature disabled.
#include "Parking/AutoParkingRuntime.hpp"
namespace AutoParking {
bool enabled() {return false;}
bool takeoverActive() {return false;}
void zebraDetected(int) {}
}

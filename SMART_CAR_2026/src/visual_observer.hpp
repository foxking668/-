#pragma once
#include "core.hpp"

namespace car2026 {
enum class ManualMotion { Forward, Reverse };
ManualMotion parseManualMotion(const std::string& value);
const char* manualMotionName(ManualMotion motion);

struct SteeringObservation {
    std::string state="WAIT_REFERENCE";
    unsigned referenceId=0;
    bool hasReference=false,hasSuggestion=false;
    double frameAge=0,lateralError=0,headingFeatureError=0,suggestedCommand=0;
};

// Diagnostic image-reference hold only. No device interface or actuator writes.
// Image lateral/heading features are NOT metres or measured vehicle angles.
class VisualSteeringObserver {
public:
    VisualSteeringObserver(const Params& params,ManualMotion motion);
    SteeringObservation observe(const Observation& observation,double frameTime,double now);
    void reset();
private:
    ManualMotion motion_;
    double timeout_,minimumConfidence_,maximumCommand_;
    int confirmationFrames_;
    double lastFrameTime_=-1,lastSuggestion_=0;
    double candidateLateral_=0,candidateHeading_=0,referenceLateral_=0,referenceHeading_=0;
    double lastLateral_=0,lastHeading_=0;
    bool hasReference_=false,hasLastFeature_=false;
    unsigned referenceId_=0,referenceSequence_=0;
    int stableFrames_=0;
    SteeringObservation reject(const char* reason,double age=0);
};
} // namespace car2026

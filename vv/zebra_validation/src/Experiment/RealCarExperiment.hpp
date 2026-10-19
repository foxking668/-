#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace Experiment
{
    struct ExperimentSnapshot
    {
        bool enabled = false;
        int method = 4;
        int methodGroup = 0;
        bool videoEnabled = false;
        int videoFps = 15;
        bool taskFeaturesEnabled = true;
        bool logEnabled = false;
        std::string runId = "DISABLED";
        double targetSpeed = 0.0;
    };

    struct FrameObservation
    {
        int evaluatedRows = 0;
        int rawLeftValidRows = 0;
        int rawRightValidRows = 0;
        int bilateralRows = 0;
        int leftMissingRows = 0;
        int rightMissingRows = 0;
        int bothMissingRows = 0;
        std::string visibilityState = "NO_CONTOUR";
        int proxyRows = 0;
        int centerValidRows = 0;
    };

    struct FrameTelemetry
    {
        std::uint64_t frameIndex = 0;
        long long timeMs = 0;
        long long captureTimeNs = -1;
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;
        FrameObservation observation;
        bool outputValid = false;
        double errorDeg = 0.0;
        double steerTargetDeg = 90.0;
        double steerDeltaDeg = 0.0;
        double processingMs = 0.0;
        bool configChanged = false;
    };

    struct RunOutcome
    {
        std::string success = "UNRECORDED";
        int outOfTrackCount = -1;
        std::string manualIntervention = "UNRECORDED";
        std::string videoPath;
        std::string notes;
    };

    class RealCarExperiment
    {
    public:
        bool initialize(const ExperimentSnapshot &snapshot,
                        const std::string &logRoot,
                        std::string &error);

        void updateMotorSpeeds(double leftSpeed, double rightSpeed);
        void setFrameObservation(const FrameObservation &observation);
        void recordFrame(bool outputValid,
                         double errorDeg,
                         double steerTargetDeg,
                         double processingMs);
        void markConfigChanged();
        void setCaptureTimeNs(long long timestamp);
        std::uint64_t nextFrameIndex() const;
        void observeCurrentConfiguration(bool enabled,
                                         int method,
                                         bool taskFeaturesEnabled,
                                         bool logEnabled,
                                         const std::string &runId,
                                         double targetSpeed);
        bool finalize(bool stoppedByOperator, std::string &error);
        bool appendOutcome(const RunOutcome &outcome,
                           const std::string &outcomesFile,
                           std::string &error) const;

        bool enabled() const;
        int method() const;
        int methodGroup() const;
        std::string methodLabel() const;
        bool taskFeaturesEnabled() const;
        bool logEnabled() const;
        double targetSpeed() const;
        const std::string &runId() const;
        const std::string &runDirectory() const;

    private:
        ExperimentSnapshot snapshot_;
        std::uint64_t run_sequence_ = 0;
        std::string run_directory_name_;
        std::string run_directory_;
        bool initialized_ = false;
        bool finalized_ = false;
        bool config_changed_ = false;
        bool has_pending_observation_ = false;
        bool has_previous_steer_target_ = false;
        double latest_left_speed_ = 0.0;
        double latest_right_speed_ = 0.0;
        double previous_steer_target_ = 90.0;
        long long pending_capture_time_ns_ = -1;
        FrameObservation pending_observation_;
        std::vector<FrameTelemetry> frames_;
        std::chrono::steady_clock::time_point start_time_;
        mutable std::mutex mutex_;
    };

    extern RealCarExperiment gRealCarExperiment;
}

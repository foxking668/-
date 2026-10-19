#include "RealCarExperiment.hpp"

#include "ExperimentConfigValidation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#else
#include <dirent.h>
#endif

namespace
{
    bool directoryExists(const std::string &path)
    {
        struct stat info = {};
        return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR) != 0;
    }

    bool nonEmptyFileExists(const std::string &path)
    {
        struct stat info = {};
        return stat(path.c_str(), &info) == 0 && info.st_size > 0;
    }

    bool createDirectory(const std::string &path)
    {
#ifdef _WIN32
        const int result = _mkdir(path.c_str());
#else
        const int result = mkdir(path.c_str(), 0755);
#endif
        return result == 0 || errno == EEXIST;
    }

    bool parseRunSequence(const std::string &name, std::uint64_t &sequence)
    {
        const std::size_t separator = name.find('_');
        if (separator == std::string::npos || separator == 0)
        {
            return false;
        }
        for (std::size_t index = 0; index < separator; ++index)
        {
            if (!std::isdigit(static_cast<unsigned char>(name[index])))
            {
                return false;
            }
        }

        errno = 0;
        char *end = nullptr;
        const std::string prefix = name.substr(0, separator);
        const unsigned long long parsed = std::strtoull(prefix.c_str(), &end, 10);
        if (errno == ERANGE || end == nullptr || *end != '\0' || parsed == 0)
        {
            return false;
        }
        sequence = static_cast<std::uint64_t>(parsed);
        return true;
    }

    bool findNextRunSequence(const std::string &logRoot,
                             std::uint64_t &nextSequence,
                             std::string &error)
    {
        std::uint64_t maximum = 0;
#ifdef _WIN32
        struct _finddata_t entry = {};
        const std::string pattern = logRoot + "/*";
        const intptr_t handle = _findfirst(pattern.c_str(), &entry);
        if (handle == -1)
        {
            if (errno == ENOENT)
            {
                nextSequence = 1;
                return true;
            }
            error = "cannot enumerate experiment log root";
            return false;
        }
        do
        {
            if ((entry.attrib & _A_SUBDIR) == 0)
            {
                continue;
            }
            std::uint64_t sequence = 0;
            if (parseRunSequence(entry.name, sequence))
            {
                maximum = std::max(maximum, sequence);
            }
        } while (_findnext(handle, &entry) == 0);
        const int enumerateError = errno;
        _findclose(handle);
        if (enumerateError != 0 && enumerateError != ENOENT)
        {
            error = "cannot enumerate experiment log root";
            return false;
        }
#else
        DIR *directory = opendir(logRoot.c_str());
        if (directory == nullptr)
        {
            error = "cannot enumerate experiment log root";
            return false;
        }
        int enumerateError = 0;
        while (true)
        {
            errno = 0;
            dirent *entry = readdir(directory);
            if (entry == nullptr)
            {
                enumerateError = errno;
                break;
            }
            const std::string name = entry->d_name;
            if (name == "." || name == ".." ||
                !directoryExists(logRoot + "/" + name))
            {
                continue;
            }
            std::uint64_t sequence = 0;
            if (parseRunSequence(name, sequence))
            {
                maximum = std::max(maximum, sequence);
            }
        }
        closedir(directory);
        if (enumerateError != 0)
        {
            error = "cannot enumerate experiment log root";
            return false;
        }
#endif
        if (maximum == std::numeric_limits<std::uint64_t>::max())
        {
            error = "experiment run sequence is exhausted";
            return false;
        }
        nextSequence = maximum + 1;
        return true;
    }

    std::string formatRunDirectoryName(std::uint64_t sequence,
                                       const std::string &runId)
    {
        std::ostringstream name;
        name << std::setw(4) << std::setfill('0') << sequence << '_' << runId;
        return name.str();
    }

    bool createDirectoryExclusively(const std::string &path, bool &alreadyExists)
    {
        alreadyExists = false;
        errno = 0;
#ifdef _WIN32
        const int result = _mkdir(path.c_str());
#else
        const int result = mkdir(path.c_str(), 0755);
#endif
        if (result == 0)
        {
            return true;
        }
        if (errno == EEXIST)
        {
            alreadyExists = true;
        }
        return false;
    }

    double percentile(std::vector<double> values, double fraction)
    {
        if (values.empty())
        {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const double position = fraction * static_cast<double>(values.size() - 1);
        const std::size_t lower = static_cast<std::size_t>(position);
        const std::size_t upper = std::min(lower + 1, values.size() - 1);
        const double weight = position - static_cast<double>(lower);
        return values[lower] * (1.0 - weight) + values[upper] * weight;
    }

    bool writeAtomically(const std::string &path,
                         const std::string &content,
                         std::string &error)
    {
        const std::string temporary = path + ".tmp";
        {
            std::ofstream output(temporary, std::ios::out | std::ios::trunc);
            if (!output.is_open())
            {
                error = "cannot open temporary output file: " + temporary;
                return false;
            }
            output << content;
            output.flush();
            if (!output.good())
            {
                error = "cannot write temporary output file: " + temporary;
                output.close();
                std::remove(temporary.c_str());
                return false;
            }
        }
        if (std::rename(temporary.c_str(), path.c_str()) != 0)
        {
            error = "cannot finalize output file: " + path;
            std::remove(temporary.c_str());
            return false;
        }
        return true;
    }

    std::string csvEscape(const std::string &value)
    {
        if (value.find_first_of(",\"\r\n") == std::string::npos)
        {
            return value;
        }
        std::string escaped = "\"";
        for (char character : value)
        {
            if (character == '"')
            {
                escaped += "\"\"";
            }
            else if (character != '\r' && character != '\n')
            {
                escaped += character;
            }
        }
        escaped += '"';
        return escaped;
    }
}

namespace Experiment
{
    RealCarExperiment gRealCarExperiment;

    bool RealCarExperiment::initialize(const ExperimentSnapshot &snapshot,
                                       const std::string &logRoot,
                                       std::string &error)
    {
        snapshot_ = snapshot;
        initialized_ = false;
        finalized_ = false;
        config_changed_ = false;
        has_pending_observation_ = false;
        has_previous_steer_target_ = false;
        latest_left_speed_ = 0.0;
        latest_right_speed_ = 0.0;
        previous_steer_target_ = 90.0;
        pending_capture_time_ns_ = -1;
        frames_.clear();
        run_sequence_ = 0;
        run_directory_name_.clear();
        run_directory_.clear();

        if (snapshot.videoEnabled && (!snapshot.enabled || snapshot.videoFps < 1 || snapshot.videoFps > 30))
        {
            error = "recording requires experiment_enable=1 and experiment_video_fps=1..30";
            return false;
        }
        if (!snapshot.enabled)
        {
            initialized_ = true;
            return true;
        }
        if (!isExperimentMethodValid(snapshot.method, snapshot.methodGroup))
        {
            error = "group 0 supports A3/A4; group 1 supports M1/M2/M3/M4";
            return false;
        }
        if (snapshot.methodGroup == 1 && snapshot.taskFeaturesEnabled)
        {
            error = "M1--M4 comparison requires experiment_task_features_enable=0";
            return false;
        }
        if (!snapshot.logEnabled)
        {
            error = "experiment_log_enable must be 1";
            return false;
        }
        if (!isRunIdValid(snapshot.runId))
        {
            error = "experiment_run_id is invalid";
            return false;
        }
        if (!createDirectory(logRoot) || !directoryExists(logRoot))
        {
            error = "cannot create experiment log root";
            return false;
        }

        if (!findNextRunSequence(logRoot, run_sequence_, error))
        {
            return false;
        }

        while (true)
        {
            run_directory_name_ = formatRunDirectoryName(run_sequence_, snapshot.runId);
            run_directory_ = logRoot + "/" + run_directory_name_;
            bool alreadyExists = false;
            if (createDirectoryExclusively(run_directory_, alreadyExists))
            {
                break;
            }
            if (!alreadyExists)
            {
                error = "cannot create experiment run directory";
                return false;
            }
            if (run_sequence_ == std::numeric_limits<std::uint64_t>::max())
            {
                error = "experiment run sequence is exhausted";
                return false;
            }
            ++run_sequence_;
        }

        initialized_ = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    void RealCarExperiment::updateMotorSpeeds(double leftSpeed, double rightSpeed)
    {
        if (!enabled())
        {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latest_left_speed_ = leftSpeed;
        latest_right_speed_ = rightSpeed;
    }

    void RealCarExperiment::setFrameObservation(const FrameObservation &observation)
    {
        if (!enabled())
        {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        pending_observation_ = observation;
        has_pending_observation_ = true;
    }

    void RealCarExperiment::recordFrame(bool outputValid,
                                        double errorDeg,
                                        double steerTargetDeg,
                                        double processingMs)
    {
        if (!enabled())
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);

        FrameTelemetry frame;
        frame.frameIndex = static_cast<std::uint64_t>(frames_.size());
        frame.captureTimeNs = pending_capture_time_ns_;
        pending_capture_time_ns_ = -1;
        frame.timeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_).count();
        frame.leftSpeed = latest_left_speed_;
        frame.rightSpeed = latest_right_speed_;
        if (has_pending_observation_)
        {
            frame.observation = pending_observation_;
        }
        frame.outputValid = outputValid && frame.observation.centerValidRows > 0;
        frame.errorDeg = errorDeg;
        frame.steerTargetDeg = steerTargetDeg;
        frame.steerDeltaDeg = has_previous_steer_target_
            ? std::abs(steerTargetDeg - previous_steer_target_) : 0.0;
        frame.processingMs = processingMs;
        frame.configChanged = config_changed_;
        frames_.push_back(frame);

        previous_steer_target_ = steerTargetDeg;
        has_previous_steer_target_ = true;
        has_pending_observation_ = false;
    }

    void RealCarExperiment::markConfigChanged()
    {
        if (!enabled())
        {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        config_changed_ = true;
    }

    void RealCarExperiment::setCaptureTimeNs(long long timestamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_capture_time_ns_ = timestamp;
    }

    std::uint64_t RealCarExperiment::nextFrameIndex() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<std::uint64_t>(frames_.size());
    }

    void RealCarExperiment::observeCurrentConfiguration(
        bool enabledValue,
        int methodValue,
        bool taskFeaturesEnabledValue,
        bool logEnabledValue,
        const std::string &runIdValue,
        double targetSpeedValue)
    {
        if (!enabled())
        {
            return;
        }
        const bool changed = enabledValue != snapshot_.enabled ||
            methodValue != snapshot_.method ||
            taskFeaturesEnabledValue != snapshot_.taskFeaturesEnabled ||
            logEnabledValue != snapshot_.logEnabled ||
            runIdValue != snapshot_.runId ||
            std::abs(targetSpeedValue - snapshot_.targetSpeed) > 1e-9;
        if (changed)
        {
            markConfigChanged();
        }
    }

    bool RealCarExperiment::finalize(bool stoppedByOperator, std::string &error)
    {
        if (!enabled())
        {
            return true;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (finalized_)
        {
            error = "experiment run was already finalized";
            return false;
        }

        std::ostringstream config;
        config << std::fixed << std::setprecision(6)
               << "experiment_enable=1\n"
               << "experiment_method=" << snapshot_.method << "\n"
               << "experiment_method_group=" << snapshot_.methodGroup << "\n"
               << "method_label=" << methodLabel() << "\n"
               << "experiment_video_enable=" << (snapshot_.videoEnabled ? 1 : 0) << "\n"
               << "experiment_video_fps=" << snapshot_.videoFps << "\n"
               << "invalid_output_control=return_zero_error_existing_policy\n"
               << "visibility_source=method_dependent_not_ground_truth\n"
               << "experiment_task_features_enable="
               << (snapshot_.taskFeaturesEnabled ? 1 : 0) << "\n"
               << "experiment_log_enable=" << (snapshot_.logEnabled ? 1 : 0) << "\n"
               << "experiment_run_id=" << snapshot_.runId << "\n"
               << "experiment_run_sequence=" << run_sequence_ << "\n"
               << "experiment_run_directory=" << run_directory_name_ << "\n"
               << "motor_aim_speed=" << snapshot_.targetSpeed << "\n";

        std::ostringstream framesCsv;
        framesCsv << "run_id,frame_index,time_ms,method,target_speed,left_speed,right_speed,"
                  << "evaluated_rows,raw_left_valid_rows,raw_right_valid_rows,bilateral_rows,"
                  << "left_missing_rows,right_missing_rows,both_missing_rows,visibility_state,"
                  << "proxy_rows,center_valid_rows,output_valid,error_deg,steer_target_deg,"
                  << "steer_delta_deg,processing_ms,config_changed,method_group,method_label,capture_time_ns\n";
        framesCsv << std::fixed << std::setprecision(6);

        std::size_t validCount = 0;
        std::size_t naturalMissingCount = 0;
        std::size_t proxyFrameCount = 0;
        std::size_t proxyRowCount = 0;
        std::size_t currentInvalidStreak = 0;
        std::size_t maxInvalidStreak = 0;
        double steerTotalVariation = 0.0;
        double leftSpeedSum = 0.0;
        double rightSpeedSum = 0.0;
        std::vector<double> steerSteps;
        std::vector<double> processingTimes;

        for (const FrameTelemetry &frame : frames_)
        {
            const FrameObservation &observation = frame.observation;
            framesCsv << snapshot_.runId << ',' << frame.frameIndex << ',' << frame.timeMs
                      << ',' << snapshot_.method << ',' << snapshot_.targetSpeed
                      << ',' << frame.leftSpeed << ',' << frame.rightSpeed
                      << ',' << observation.evaluatedRows
                      << ',' << observation.rawLeftValidRows
                      << ',' << observation.rawRightValidRows
                      << ',' << observation.bilateralRows
                      << ',' << observation.leftMissingRows
                      << ',' << observation.rightMissingRows
                      << ',' << observation.bothMissingRows
                      << ',' << observation.visibilityState
                      << ',' << observation.proxyRows
                      << ',' << observation.centerValidRows
                      << ',' << (frame.outputValid ? 1 : 0)
                      << ',' << frame.errorDeg
                      << ',' << frame.steerTargetDeg
                      << ',' << frame.steerDeltaDeg
                      << ',' << frame.processingMs
                      << ',' << (frame.configChanged ? 1 : 0)
                      << ',' << snapshot_.methodGroup << ',' << methodLabel()
                      << ',' << frame.captureTimeNs << '\n';

            validCount += frame.outputValid ? 1U : 0U;
            const bool naturallyMissing = observation.leftMissingRows > 0 ||
                                          observation.rightMissingRows > 0 ||
                                          observation.bothMissingRows > 0;
            naturalMissingCount += naturallyMissing ? 1U : 0U;
            proxyFrameCount += observation.proxyRows > 0 ? 1U : 0U;
            proxyRowCount += static_cast<std::size_t>(std::max(0, observation.proxyRows));
            if (frame.outputValid)
            {
                currentInvalidStreak = 0;
            }
            else
            {
                ++currentInvalidStreak;
                maxInvalidStreak = std::max(maxInvalidStreak, currentInvalidStreak);
            }
            steerTotalVariation += frame.steerDeltaDeg;
            steerSteps.push_back(frame.steerDeltaDeg);
            processingTimes.push_back(frame.processingMs);
            leftSpeedSum += frame.leftSpeed;
            rightSpeedSum += frame.rightSpeed;
        }

        const double frameCount = static_cast<double>(frames_.size());
        const double validRate = frames_.empty() ? 0.0 :
            static_cast<double>(validCount) / frameCount;
        const double missingRate = frames_.empty() ? 0.0 :
            static_cast<double>(naturalMissingCount) / frameCount;
        const double meanLeftSpeed = frames_.empty() ? 0.0 : leftSpeedSum / frameCount;
        const double meanRightSpeed = frames_.empty() ? 0.0 : rightSpeedSum / frameCount;

        std::ostringstream summary;
        summary << "run_id,method,target_speed,frame_count,output_valid_rate,"
                << "natural_missing_frame_count,natural_missing_frame_rate,"
                << "proxy_frame_count,proxy_row_count,max_invalid_streak,"
                << "steer_total_variation_deg,steer_step_p95_deg,processing_p50_ms,"
                << "processing_p95_ms,mean_left_speed,mean_right_speed,config_changed,method_group,method_label\n";
        summary << std::fixed << std::setprecision(6)
                << snapshot_.runId << ',' << snapshot_.method << ',' << snapshot_.targetSpeed
                << ',' << frames_.size() << ',' << validRate
                << ',' << naturalMissingCount << ',' << missingRate
                << ',' << proxyFrameCount << ',' << proxyRowCount
                << ',' << maxInvalidStreak << ',' << steerTotalVariation
                << ',' << percentile(steerSteps, 0.95)
                << ',' << percentile(processingTimes, 0.50)
                << ',' << percentile(processingTimes, 0.95)
                << ',' << meanLeftSpeed << ',' << meanRightSpeed
                << ',' << (config_changed_ ? 1 : 0)
                << ',' << snapshot_.methodGroup << ',' << methodLabel() << '\n';

        std::ostringstream status;
        status << "status=" << (stoppedByOperator ? "STOPPED_BY_OPERATOR" : "PROCESS_EXIT") << '\n'
               << "run_id=" << snapshot_.runId << '\n'
               << "run_sequence=" << run_sequence_ << '\n'
               << "run_directory=" << run_directory_name_ << '\n'
               << "frame_count=" << frames_.size() << '\n'
               << "config_changed=" << (config_changed_ ? 1 : 0) << '\n';

        if (!writeAtomically(run_directory_ + "/config_snapshot.txt", config.str(), error) ||
            !writeAtomically(run_directory_ + "/frames.csv", framesCsv.str(), error) ||
            !writeAtomically(run_directory_ + "/summary.csv", summary.str(), error) ||
            !writeAtomically(run_directory_ + "/run_status.txt", status.str(), error))
        {
            return false;
        }

        finalized_ = true;
        return true;
    }

    bool RealCarExperiment::appendOutcome(const RunOutcome &outcome,
                                          const std::string &outcomesFile,
                                          std::string &error) const
    {
        if (!enabled())
        {
            return true;
        }
        if (outcome.success != "0" && outcome.success != "1" &&
            outcome.success != "UNRECORDED")
        {
            error = "outcome success must be 0, 1, or UNRECORDED";
            return false;
        }
        if (outcome.manualIntervention != "0" && outcome.manualIntervention != "1" &&
            outcome.manualIntervention != "UNRECORDED")
        {
            error = "manual intervention must be 0, 1, or UNRECORDED";
            return false;
        }
        if (outcome.outOfTrackCount < -1)
        {
            error = "out-of-track count must be nonnegative or -1";
            return false;
        }

        // The numbered directory is authoritative when run_id is reused.
        std::ostringstream localOutcome;
        localOutcome << "run_directory,method_label,run_id,success,out_of_track_count,manual_intervention,video_path,notes\n"
                     << run_directory_name_ << ',' << methodLabel() << ',' << snapshot_.runId << ','
                     << outcome.success << ',';
        if (outcome.outOfTrackCount >= 0) localOutcome << outcome.outOfTrackCount;
        else localOutcome << "UNRECORDED";
        localOutcome << ',' << outcome.manualIntervention << ',' << csvEscape(outcome.videoPath)
                     << ',' << csvEscape(outcome.notes) << '\n';
        if (!writeAtomically(run_directory_ + "/outcome.csv", localOutcome.str(), error)) return false;

        const bool writeHeader = !nonEmptyFileExists(outcomesFile);
        std::ofstream output(outcomesFile, std::ios::out | std::ios::app);
        if (!output.is_open())
        {
            error = "cannot open run outcome file: " + outcomesFile;
            return false;
        }
        if (writeHeader)
        {
            output << "run_id,success,out_of_track_count,manual_intervention,video_path,notes\n";
        }
        output << snapshot_.runId << ','
               << outcome.success << ',';
        if (outcome.outOfTrackCount >= 0)
        {
            output << outcome.outOfTrackCount;
        }
        else
        {
            output << "UNRECORDED";
        }
        output << ',' << outcome.manualIntervention
               << ',' << csvEscape(outcome.videoPath)
               << ',' << csvEscape(outcome.notes) << '\n';
        output.flush();
        if (!output.good())
        {
            error = "cannot write run outcome file: " + outcomesFile;
            return false;
        }
        return true;
    }

    bool RealCarExperiment::enabled() const
    {
        return initialized_ && snapshot_.enabled;
    }

    int RealCarExperiment::method() const
    {
        return snapshot_.method;
    }

    int RealCarExperiment::methodGroup() const { return snapshot_.methodGroup; }

    std::string RealCarExperiment::methodLabel() const
    {
        return std::string(snapshot_.methodGroup == 1 ? "M" : "A") + std::to_string(snapshot_.method);
    }

    bool RealCarExperiment::taskFeaturesEnabled() const
    {
        return snapshot_.taskFeaturesEnabled;
    }

    bool RealCarExperiment::logEnabled() const
    {
        return snapshot_.logEnabled;
    }

    double RealCarExperiment::targetSpeed() const
    {
        return snapshot_.targetSpeed;
    }

    const std::string &RealCarExperiment::runId() const
    {
        return snapshot_.runId;
    }

    const std::string &RealCarExperiment::runDirectory() const
    {
        return run_directory_;
    }
}

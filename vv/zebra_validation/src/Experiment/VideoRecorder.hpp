#pragma once

#include <opencv2/core.hpp>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>

namespace Experiment
{
    struct VideoOptions
    {
        int fps = 15;
        std::size_t queueCapacity = 8;
        std::string fileName = "front_camera.avi";
        std::string metadataHeader;
    };

    struct VideoStats
    {
        std::uint64_t written = 0;
        std::uint64_t decoded = 0;
        bool verified = false;
        std::uint64_t dropped = 0;
        std::uint64_t rateSkipped = 0;
        std::size_t maxQueued = 0;
        bool closed = false;
        std::string error;
    };

    enum class RecordSubmit { Disabled, Queued, RateSkipped, Overflow, Failed };

    bool verifyRecordedVideo(const std::string &path, cv::Size size, std::uint64_t expectedFrames,
                             std::uint64_t &decodedFrames, std::string &error);

    class VideoRecorder
    {
    public:
        ~VideoRecorder();
        bool start(const std::string &directory, cv::Size size, const VideoOptions &options, std::string &error);
        RecordSubmit submit(const cv::Mat &original, std::uint64_t frameIndex, long long captureTimeNs,
                            const std::string &metadata = {});
        bool shouldSelect(long long captureTimeNs) const;
        void stop();
        VideoStats stats() const;
        std::string videoPath() const;

    private:
        struct Frame { cv::Mat image; std::uint64_t index; long long timeNs; std::string metadata; };
        void worker();
        void fail(const std::string &error);
        bool writeStatus(const std::string &state);
        mutable std::mutex mutex_;
        std::condition_variable wake_;
        std::deque<Frame> queue_;
        std::thread thread_;
        VideoOptions options_;
        VideoStats stats_;
        std::string directory_;
        cv::Size size_;
        long long last_selected_ns_ = -1;
        bool ready_ = false;
        bool started_ = false;
        bool stopping_ = false;
    };

    extern VideoRecorder gVideoRecorder;
    extern VideoRecorder gProcessedVideoRecorder;
    extern std::atomic<bool> gProcessedRecordingEnabled;
}

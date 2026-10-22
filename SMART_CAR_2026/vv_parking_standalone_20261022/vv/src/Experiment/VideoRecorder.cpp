#include "VideoRecorder.hpp"
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/statvfs.h>
#include <pthread.h>
#include <sched.h>
#endif

namespace
{
    bool hasSpace(const std::string &directory)
    {
        constexpr unsigned long long reserve = 64ULL * 1024 * 1024;
#ifdef _WIN32
        ULARGE_INTEGER available;
        return GetDiskFreeSpaceExA(directory.c_str(), &available, nullptr, nullptr) && available.QuadPart > reserve;
#else
        struct statvfs info;
        return statvfs(directory.c_str(), &info) == 0 &&
            static_cast<unsigned long long>(info.f_bavail) * info.f_frsize > reserve;
#endif
    }
    bool exists(const std::string &path) { return std::ifstream(path).good(); }
}

namespace Experiment
{
    VideoRecorder gVideoRecorder;
    VideoRecorder gProcessedVideoRecorder;
    std::atomic<bool> gProcessedRecordingEnabled{false};

    bool verifyRecordedVideo(const std::string &path, cv::Size size, std::uint64_t expectedFrames,
                             std::uint64_t &decodedFrames, std::string &error)
    {
        decodedFrames = 0;
        error.clear();
        try
        {
            cv::VideoCapture video(path);
            if (!video.isOpened()) throw std::runtime_error("final AVI cannot be opened for verification");
            cv::Mat frame;
            while (video.read(frame))
            {
                if (frame.empty() || frame.size() != size)
                    throw std::runtime_error("final AVI frame size mismatch");
                ++decodedFrames;
                if (decodedFrames > expectedFrames) break;
            }
            if (decodedFrames != expectedFrames)
                throw std::runtime_error("final AVI decoded frame count differs from timestamp rows");
        }
        catch (const std::exception &e) { error = e.what(); return false; }
        return true;
    }

    VideoRecorder::~VideoRecorder() { stop(); }

    bool VideoRecorder::start(const std::string &directory, cv::Size size,
                              const VideoOptions &options, std::string &error)
    {
        error.clear();
        if (thread_.joinable()) { error = "recorder already running"; return false; }
        if (options.fps < 1 || options.fps > 30 || options.queueCapacity < 1 || options.queueCapacity > 32 ||
            size.width < 2 || size.height < 2 || size.width > 4096 || size.height > 4096 ||
            size.width % 2 || size.height % 2 || options.fileName.empty() ||
            options.fileName.find_first_of("/\\\r\n") != std::string::npos ||
            options.metadataHeader.find_first_of("\r\n") != std::string::npos)
        {
            error = "invalid recording fps/queue/original frame size";
            return false;
        }
        if (exists(directory + "/" + options.fileName) || exists(directory + "/video_timestamps.csv"))
        {
            error = "recording output exists; refusing to overwrite";
            return false;
        }
        if (!hasSpace(directory)) { error = "recording directory unavailable or free space <= 64 MiB"; return false; }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            directory_ = directory;
            size_ = size;
            options_ = options;
            stats_ = VideoStats();
            queue_.clear();
            stopping_ = ready_ = started_ = false;
            last_selected_ns_ = -1;
        }
        try { thread_ = std::thread(&VideoRecorder::worker, this); }
        catch (const std::exception &e) { error = e.what(); return false; }
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [this] { return ready_; });
            error = stats_.error;
        }
        if (!error.empty()) { stop(); return false; }
        return true;
    }

    RecordSubmit VideoRecorder::submit(const cv::Mat &original, std::uint64_t frameIndex, long long captureTimeNs, const std::string &metadata)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!stats_.error.empty()) return RecordSubmit::Failed;
        if (!started_ || stopping_ || stats_.closed) return RecordSubmit::Disabled;
        if (original.empty() || original.type() != CV_8UC3 || original.size() != size_ || captureTimeNs < 0 || metadata.find_first_of("\r\n") != std::string::npos ||
            (last_selected_ns_ >= 0 && captureTimeNs < last_selected_ns_))
        {
            stats_.error = "invalid original frame or nonmonotonic capture timestamp";
            wake_.notify_all();
            return RecordSubmit::Failed;
        }
        const long long interval = 1000000000LL / options_.fps;
        if (last_selected_ns_ >= 0 && captureTimeNs - last_selected_ns_ < interval)
        {
            ++stats_.rateSkipped;
            return RecordSubmit::RateSkipped;
        }
        last_selected_ns_ = captureTimeNs;
        if (queue_.size() >= options_.queueCapacity)
        {
            ++stats_.dropped;
            return RecordSubmit::Overflow;
        }
        try { queue_.push_back({original.clone(), frameIndex, captureTimeNs, metadata}); }
        catch (const std::exception &e) { stats_.error = e.what(); wake_.notify_all(); return RecordSubmit::Failed; }
        stats_.maxQueued = std::max(stats_.maxQueued, queue_.size());
        wake_.notify_one();
        return RecordSubmit::Queued;
    }

    bool VideoRecorder::shouldSelect(long long timeNs) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return started_ && !stopping_ && !stats_.closed && stats_.error.empty() &&
            (last_selected_ns_ < 0 || timeNs-last_selected_ns_ >= 1000000000LL/options_.fps);
    }

    void VideoRecorder::stop()
    {
        if (!thread_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        thread_.join();
    }

    VideoStats VideoRecorder::stats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
    std::string VideoRecorder::videoPath() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return started_ ? directory_ + "/" + options_.fileName : std::string();
    }
    void VideoRecorder::fail(const std::string &error)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stats_.error.empty()) stats_.error = error;
        stats_.dropped += queue_.size();
        queue_.clear();
    }

    bool VideoRecorder::writeStatus(const std::string &state)
    {
        const auto s = stats();
        std::ofstream output(directory_ + "/recording_status.txt", std::ios::trunc);
        output << "video_file=" << options_.fileName << "\n" << "state=" << state << "\ncodec=MJPG\ncontainer=AVI\n"
               << "width=" << size_.width << "\nheight=" << size_.height << "\nnominal_fps=" << options_.fps
               << "\nqueue_capacity=" << options_.queueCapacity << "\nwritten_frames=" << s.written
               << "\ndecoded_frames=" << s.decoded << "\nverification_passed=" << (s.verified ? 1 : 0)
               << "\nrate_skipped_frames=" << s.rateSkipped << "\nqueue_dropped_frames=" << s.dropped
               << "\nmax_queued_frames=" << s.maxQueued << "\nerror=" << s.error
               << "\ncapture_clock=steady_clock_nanoseconds_after_retrieve\n"
               << "timing_source=video_timestamps.csv_not_nominal_video_playback_time\n";
        output.flush();
        return output.good();
    }

    void VideoRecorder::worker()
    {
        cv::VideoWriter writer;
        std::ofstream timestamps;
        try
        {
#ifndef _WIN32
            // The vehicle process is realtime; video encoding must not inherit that priority.
            sched_param priority{};
            if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &priority) != 0)
                throw std::runtime_error("cannot lower video worker to normal scheduling priority");
#endif
            if (!writer.open(directory_ + "/" + options_.fileName, cv::VideoWriter::fourcc('M','J','P','G'),
                             options_.fps, size_, true))
                throw std::runtime_error("MJPG/AVI encoder unavailable; control threads not started");
            timestamps.open(directory_ + "/video_timestamps.csv", std::ios::trunc);
            timestamps << "video_frame_index,frame_index,capture_time_ns";
            if(!options_.metadataHeader.empty()) timestamps << "," << options_.metadataHeader;
            timestamps << "\n";
            timestamps.flush();
            if (!timestamps.good() || !writeStatus("RECORDING"))
                throw std::runtime_error("cannot create recording metadata");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                started_ = ready_ = true;
            }
            wake_.notify_all();
            for (;;)
            {
                Frame frame;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    wake_.wait(lock, [this] { return stopping_ || !queue_.empty() || !stats_.error.empty(); });
                    if (!stats_.error.empty()) throw std::runtime_error(stats_.error);
                    if (queue_.empty() && stopping_) break;
                    frame = std::move(queue_.front());
                    queue_.pop_front();
                }
                const auto index = stats().written;
                if (index % 15 == 0 && !hasSpace(directory_))
                    throw std::runtime_error("recording stopped: free space <= 64 MiB or storage unavailable");
                writer.write(frame.image);
                timestamps << index << ',' << frame.index << ',' << frame.timeNs;
                if(!options_.metadataHeader.empty()) timestamps << ',' << frame.metadata;
                timestamps << '\n';
                timestamps.flush();
                if (!timestamps.good()) throw std::runtime_error("recording timestamp write failed");
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++stats_.written;
                }
            }
        }
        catch (const std::exception &e) { fail(e.what()); }
        catch (...) { fail("unknown recording failure"); }
        try { writer.release(); }
        catch (const std::exception &e) { fail(e.what()); }
        timestamps.close();
        const auto beforeVerification = stats();
        if (beforeVerification.error.empty() && beforeVerification.written > 0)
        {
            // Only after the control threads stop: decode every saved frame, not CAP_PROP_FRAME_COUNT.
            std::cout << "[Recording] VERIFYING saved AVI; please wait before powering off" << std::endl;
            if (!writeStatus("VERIFYING")) fail("cannot write recording verification status");
            std::uint64_t decoded = 0;
            std::string verificationError;
            const bool verified = verifyRecordedVideo(directory_ + "/" + options_.fileName, size_,
                                                       beforeVerification.written, decoded, verificationError);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stats_.decoded = decoded;
                stats_.verified = verified;
            }
            if (!verified) fail(verificationError);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ready_ = true;
            stats_.closed = true;
        }
        wake_.notify_all();
        const auto s = stats();
        const std::string state = !s.error.empty() ? "FAILED" :
            (s.written == 0 ? "NO_FRAMES" : (s.dropped > 0 ? "CLOSED_WITH_DROPS" : "CLOSED"));
        if (!writeStatus(state)) fail("cannot finalize recording status file");
        if (!stats().error.empty()) std::cerr << "[Recording] " << stats().error << std::endl;
    }
}

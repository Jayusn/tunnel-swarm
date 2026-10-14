#pragma once
#include <array>
#include <chrono>
#include <cstdio>
#include <ros/ros.h>
#include <ros/file_log.h>
#include <unistd.h>

// Diagnostic only: steady-clock stage durations, buffered CSV, no flight decisions.
class PerformanceTrace {
public:
    static double now() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    explicit PerformanceTrace(bool enabled) {
        if (!enabled) return;
        const std::string path = ros::file_log::getLogDirectory() +
            "/fastlio_timing_" + std::to_string(getpid()) + ".csv";
        file_ = std::fopen(path.c_str(), "w");
        if (!file_) { ROS_WARN("Cannot open FAST-LIO timing log: %s", path.c_str()); return; }
        std::setvbuf(file_, buffer_.data(), _IOFBF, buffer_.size());
        std::fprintf(file_, "wall_time,lidar_stamp,points,loop_gap_ms,total_ms,callbacks_ms,sync_ms,imu_ms,map_segment_ms,downsample_ms,map_init_prepare_ms,debug_write_ms,match_ms,predict_odom_ms,map_update_ms,publish_ms,tail_ms,previous_log_ms\n");
        ROS_INFO("FAST-LIO timing log: %s", path.c_str());
    }
    ~PerformanceTrace() { if (file_) std::fclose(file_); }
    class Loop {
    public:
        explicit Loop(PerformanceTrace &owner) : owner_(owner) {
            if (!owner_.file_) return;
            begin_ = last_ = now();
            gap_ = owner_.last_begin_ ? begin_ - owner_.last_begin_ : 0;
            owner_.last_begin_ = begin_;
        }
        void mark(unsigned stage) {
            if (!owner_.file_) return;
            const double t = now();
            durations_[stage] += (t - last_) * 1000;
            last_ = t;
        }
        void frame(double stamp, size_t points) { stamp_ = stamp; points_ = points; }
        ~Loop() {
            if (!owner_.file_) return;
            mark(11);
            const double total = (last_ - begin_) * 1000;
            // Keep every frame and slow idle iterations; avoid 5000 CSV rows/s.
            if (!stamp_ && total < 20 && gap_ < 0.02) return;
            const double write_begin = now();
            std::fprintf(owner_.file_, "%.9f,%.9f,%zu,%.3f,%.3f", ros::WallTime::now().toSec(), stamp_, points_, gap_*1000, total);
            for (double value : durations_) std::fprintf(owner_.file_, ",%.3f", value);
            std::fprintf(owner_.file_, ",%.3f\n", owner_.previous_log_ms_);
            if (write_begin - owner_.last_flush_ >= 1.0) {
                std::fflush(owner_.file_);
                owner_.last_flush_ = write_begin;
            }
            owner_.previous_log_ms_ = (now() - write_begin) * 1000;
        }
    private:
        PerformanceTrace &owner_;
        std::array<double, 12> durations_{};
        double begin_=0, last_=0, gap_=0, stamp_=0;
        size_t points_=0;
    };
private:
    std::array<char, 65536> buffer_{};
    FILE *file_=nullptr;
    double last_begin_=0, last_flush_=0, previous_log_ms_=0;
};

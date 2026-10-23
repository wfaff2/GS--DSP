#ifndef DSP_POSE_CLOUD_SYNC_H
#define DSP_POSE_CLOUD_SYNC_H

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iterator>
#include <limits>
#include <mutex>
#include <utility>

namespace dsp_sync {

template <typename Pose, typename Cloud,
          typename WallClock = std::chrono::steady_clock>
class TimestampPairer {
public:
    using WallTime = typename WallClock::time_point;
    using WallDuration = typename WallClock::duration;

    struct Match {
        Cloud cloud;
        Pose pose_before;
        Pose pose_after;
        double interpolation_factor = 0.0;
        double cloud_stamp = 0.0;
        bool exact = false;
    };

    struct Counts {
        std::size_t waiting_clouds = 0;
        std::size_t matched_clouds = 0;
        std::size_t replaced_clouds = 0;
        std::size_t timed_out_clouds = 0;
        std::size_t nonmonotonic_clouds = 0;
        std::size_t invalid_stamp_clouds = 0;
        std::size_t shutdown_clouds = 0;
        std::size_t dropped_poses = 0;
        std::size_t duplicate_poses = 0;
    };

    enum class WaitEvent {
        kMatch,
        kTimedOut,
        kShutdown
    };

    struct CloudSubmission {
        bool accepted = false;
        bool waiting_for_pose = false;
        bool replaced_pending_cloud = false;
        bool dropped_nonmonotonic = false;
        bool dropped_invalid_stamp = false;
        bool dropped_shutdown = false;
    };

    TimestampPairer(std::size_t max_poses, WallDuration cloud_wait_timeout)
        : max_poses_(std::max<std::size_t>(2, max_poses)),
          cloud_wait_timeout_(cloud_wait_timeout) {}

    void addPose(double stamp, const Pose &pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        PoseSample incoming{stamp, pose};
        auto it = std::lower_bound(
                poses_.begin(), poses_.end(), stamp,
                [](const PoseSample &sample, double value) {
                    return sample.stamp < value;
                });
        if(it != poses_.end() && it->stamp == stamp) {
            it->pose = pose;
            ++counts_.duplicate_poses;
        } else {
            poses_.insert(it, std::move(incoming));
        }
        while(poses_.size() > max_poses_) {
            poses_.pop_front();
            ++counts_.dropped_poses;
        }
        condition_.notify_one();
    }

    CloudSubmission addCloud(double stamp, Cloud cloud,
                             WallTime now = WallClock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);
        CloudSubmission result;
        if(shutdown_) {
            result.dropped_shutdown = true;
            ++counts_.shutdown_clouds;
            return result;
        }
        if(!isPositiveFiniteStamp(stamp)) {
            result.dropped_invalid_stamp = true;
            ++counts_.invalid_stamp_clouds;
            return result;
        }
        if(has_accepted_cloud_ && stamp <= newest_accepted_cloud_stamp_) {
            result.dropped_nonmonotonic = true;
            ++counts_.nonmonotonic_clouds;
            return result;
        }
        if(pending_.valid) {
            result.replaced_pending_cloud = true;
            ++counts_.replaced_clouds;
        }
        pending_.stamp = stamp;
        pending_.cloud = std::move(cloud);
        pending_.deadline = now + cloud_wait_timeout_;
        pending_.valid = true;
        newest_accepted_cloud_stamp_ = stamp;
        has_accepted_cloud_ = true;
        result.accepted = true;
        result.waiting_for_pose = !hasMatchLocked(stamp);
        if(result.waiting_for_pose) {
            ++counts_.waiting_clouds;
        }
        condition_.notify_one();
        return result;
    }

    WaitEvent waitNext(Match &match) {
        std::unique_lock<std::mutex> lock(mutex_);
        for(;;) {
            condition_.wait(lock, [this] { return shutdown_ || pending_.valid; });
            if(shutdown_) {
                return WaitEvent::kShutdown;
            }

            if(makeMatchLocked(match)) {
                ++counts_.matched_clouds;
                return WaitEvent::kMatch;
            }

            const WallTime deadline = pending_.deadline;
            condition_.wait_until(lock, deadline, [this, deadline] {
                return shutdown_ || !pending_.valid ||
                       pending_.deadline != deadline ||
                       hasMatchLocked(pending_.stamp);
            });
            if(shutdown_) {
                return WaitEvent::kShutdown;
            }
            if(!pending_.valid || pending_.deadline != deadline) {
                continue;
            }
            if(makeMatchLocked(match)) {
                ++counts_.matched_clouds;
                return WaitEvent::kMatch;
            }
            if(WallClock::now() >= pending_.deadline) {
                pending_.valid = false;
                ++counts_.timed_out_clouds;
                return WaitEvent::kTimedOut;
            }
        }
    }

    bool latestPose(Pose &pose) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(poses_.empty()) {
            return false;
        }
        pose = poses_.back().pose;
        return true;
    }

    Counts counts() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return counts_;
    }

    std::size_t poseCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return poses_.size();
    }

    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        condition_.notify_all();
    }

private:
    static bool isPositiveFiniteStamp(double stamp) {
        static_assert(sizeof(double) == sizeof(std::uint64_t) &&
                      std::numeric_limits<double>::is_iec559,
                      "Timestamp validation requires an IEEE-754 binary64 double");
        std::uint64_t bits = 0;
        std::memcpy(&bits, &stamp, sizeof(bits));
        constexpr std::uint64_t sign_mask = UINT64_C(0x8000000000000000);
        constexpr std::uint64_t exponent_mask = UINT64_C(0x7ff0000000000000);
        constexpr std::uint64_t magnitude_mask = UINT64_C(0x7fffffffffffffff);
        return (bits & sign_mask) == 0 &&
               (bits & exponent_mask) != exponent_mask &&
               (bits & magnitude_mask) != 0;
    }

    struct PoseSample {
        double stamp;
        Pose pose;
    };

    struct PendingCloud {
        double stamp = 0.0;
        Cloud cloud{};
        WallTime deadline{};
        bool valid = false;
    };

    typename std::deque<PoseSample>::iterator lowerBoundLocked(double stamp) {
        return std::lower_bound(
                poses_.begin(), poses_.end(), stamp,
                [](const PoseSample &sample, double value) {
                    return sample.stamp < value;
                });
    }

    bool hasMatchLocked(double stamp) const {
        auto upper = std::lower_bound(
                poses_.begin(), poses_.end(), stamp,
                [](const PoseSample &sample, double value) {
                    return sample.stamp < value;
                });
        if(upper != poses_.end() && upper->stamp == stamp) {
            return true;
        }
        return upper != poses_.begin() && upper != poses_.end();
    }

    bool makeMatchLocked(Match &match) {
        if(!pending_.valid) {
            return false;
        }
        auto upper = lowerBoundLocked(pending_.stamp);
        if(upper == poses_.end()) {
            return false;
        }

        match.cloud_stamp = pending_.stamp;
        if(upper->stamp == pending_.stamp) {
            match.pose_before = upper->pose;
            match.pose_after = upper->pose;
            match.interpolation_factor = 0.0;
            match.exact = true;
            match.cloud = std::move(pending_.cloud);
            poses_.erase(poses_.begin(), upper);
            pending_.valid = false;
            return true;
        }
        if(upper == poses_.begin()) {
            return false;
        }

        auto lower = std::prev(upper);
        const double interval = upper->stamp - lower->stamp;
        if(interval <= 0.0) {
            return false;
        }
        match.pose_before = lower->pose;
        match.pose_after = upper->pose;
        match.interpolation_factor = (pending_.stamp - lower->stamp) / interval;
        match.exact = false;
        match.cloud = std::move(pending_.cloud);
        poses_.erase(poses_.begin(), lower);
        pending_.valid = false;
        return true;
    }

    const std::size_t max_poses_;
    const WallDuration cloud_wait_timeout_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<PoseSample> poses_;
    PendingCloud pending_;
    Counts counts_;
    double newest_accepted_cloud_stamp_ = 0.0;
    bool has_accepted_cloud_ = false;
    bool shutdown_ = false;
};

}  // namespace dsp_sync

#endif  // DSP_POSE_CLOUD_SYNC_H

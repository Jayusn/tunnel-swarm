#ifndef MOT_MAPPING_ONLINE_BACKGROUND_H
#define MOT_MAPPING_ONLINE_BACKGROUND_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mot_mapping {

struct OnlineBackgroundOptions {
    double voxel_size = 0.05;
    double residual = 0.03;
    int min_observations = 5;
    double min_duration = 0.5;
    double update_interval = 0.3;
    double local_radius = 8.0;
};

struct OnlineBackgroundUpdate {
    bool changed = false;
    bool paused = false;
    std::size_t added = 0;
    std::size_t removed = 0;
    std::size_t accepted = 0;
    std::size_t pending = 0;
};

// The caller supplies already registered world points and excludes candidates,
// confirmed tracks and ray-supported foreground. No input points are removed or
// rewritten. Each voxel receives exactly one centroid observation per callback.
class OnlineBackground {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZ>;

    explicit OnlineBackground(const OnlineBackgroundOptions& options = OnlineBackgroundOptions())
        : options_(options), accepted_cloud_(new Cloud) {
        if (!std::isfinite(options.voxel_size) || options.voxel_size <= 0.0 ||
            !std::isfinite(options.residual) || options.residual <= 0.0 ||
            options.min_observations < 4 || !std::isfinite(options.min_duration) ||
            options.min_duration <= 0.0 || !std::isfinite(options.update_interval) ||
            options.update_interval <= 0.0 || !std::isfinite(options.local_radius) ||
            options.local_radius <= 0.0)
            throw std::invalid_argument("invalid online background options");
    }

    Cloud::ConstPtr acceptedCloud() const { return accepted_cloud_; }
    std::size_t pendingSize() const { return pending_.size(); }
    void clearPending() { pending_.clear(); }

    OnlineBackgroundUpdate observe(const Cloud& points, const std::vector<uint8_t>& excluded,
                                   double stamp, bool paused,
                                   const Eigen::Vector3d& lidar_center) {
        if (!excluded.empty() && excluded.size() != points.size())
            throw std::invalid_argument("online background exclusion size does not match cloud");
        OnlineBackgroundUpdate result;
        result.paused = paused;
        if (!std::isfinite(stamp) || !lidar_center.allFinite() || stamp <= last_stamp_) {
            pending_.clear();
            return summary(result);
        }
        last_stamp_ = stamp;
        ++generation_;
        const double range_squared = options_.local_radius * options_.local_radius;
        const bool can_write = stamp - last_write_stamp_ + 1e-6 >= options_.update_interval;

        if (can_write) {
            for (auto entry = accepted_.begin(); entry != accepted_.end();) {
                if ((entry->second - lidar_center).squaredNorm() > range_squared) {
                    entry = accepted_.erase(entry);
                    ++result.removed;
                } else {
                    ++entry;
                }
            }
        }

        if (paused) {
            pending_.clear();
        } else {
            std::map<Key, Sample> samples;
            for (std::size_t index = 0; index < points.size(); ++index) {
                const Eigen::Vector3d point = points[index].getVector3fMap().cast<double>();
                if (!point.allFinite() || (point - lidar_center).squaredNorm() > range_squared) continue;
                const Key key = keyFor(point);
                if (accepted_.count(key)) continue;
                Sample& sample = samples[key];
                sample.sum += point;
                ++sample.count;
                sample.excluded = sample.excluded || (!excluded.empty() && excluded[index]);
            }

            for (const auto& sample_entry : samples) {
                const Key& key = sample_entry.first;
                const Sample& sample = sample_entry.second;
                if (sample.excluded) {
                    pending_.erase(key);
                    continue;
                }
                const Eigen::Vector3d center = sample.sum / static_cast<double>(sample.count);
                Pending& pending = pending_[key];
                if (pending.last_generation + 1 != generation_ ||
                    (pending.count > 0 && (center - pending.reference).norm() >= options_.residual))
                    pending = Pending();
                if (pending.count == 0) {
                    pending.first_stamp = stamp;
                    pending.reference = center;
                }
                pending.last_generation = generation_;
                ++pending.count;
                pending.recent.push_back(center);
                if (pending.recent.size() > 4) pending.recent.pop_front();

                bool recent_stable = pending.recent.size() == 4;
                for (std::size_t i = 0; recent_stable && i < pending.recent.size(); ++i)
                    for (std::size_t j = i + 1; j < pending.recent.size(); ++j)
                        if ((pending.recent[i] - pending.recent[j]).norm() >= options_.residual)
                            recent_stable = false;
                // A moving sequence cannot borrow its earlier duration after
                // stopping: any violated recent residual restarts its votes.
                if (!recent_stable && pending.recent.size() == 4) {
                    pending = Pending();
                    pending.first_stamp = stamp;
                    pending.reference = center;
                    pending.last_generation = generation_;
                    pending.count = 1;
                    pending.recent.push_back(center);
                    continue;
                }
                if (can_write && recent_stable &&
                    (pending.count >= static_cast<std::size_t>(options_.min_observations) ||
                     stamp - pending.first_stamp + 1e-6 >= options_.min_duration)) {
                    Eigen::Vector3d accepted_center = Eigen::Vector3d::Zero();
                    for (const Eigen::Vector3d& recent : pending.recent) accepted_center += recent;
                    accepted_[key] = accepted_center / static_cast<double>(pending.recent.size());
                    pending_.erase(key);
                    ++result.added;
                }
            }
            for (auto entry = pending_.begin(); entry != pending_.end();) {
                if (entry->second.last_generation != generation_ ||
                    (entry->second.reference - lidar_center).squaredNorm() > range_squared)
                    entry = pending_.erase(entry);
                else
                    ++entry;
            }
        }

        if (result.added || result.removed) {
            Cloud::Ptr next(new Cloud);
            next->reserve(accepted_.size());
            for (const auto& entry : accepted_)
                next->push_back(pcl::PointXYZ(entry.second.x(), entry.second.y(), entry.second.z()));
            accepted_cloud_ = next;
            result.changed = true;
            last_write_stamp_ = stamp;
        }
        return summary(result);
    }

private:
    using Key = std::array<int64_t, 3>;
    struct Sample {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        std::size_t count = 0;
        bool excluded = false;
    };
    struct Pending {
        Eigen::Vector3d reference = Eigen::Vector3d::Zero();
        std::deque<Eigen::Vector3d> recent;
        std::size_t count = 0;
        uint64_t last_generation = 0;
        double first_stamp = 0.0;
    };

    Key keyFor(const Eigen::Vector3d& point) const {
        return {{static_cast<int64_t>(std::floor(point.x() / options_.voxel_size)),
                 static_cast<int64_t>(std::floor(point.y() / options_.voxel_size)),
                 static_cast<int64_t>(std::floor(point.z() / options_.voxel_size))}};
    }
    OnlineBackgroundUpdate summary(OnlineBackgroundUpdate result) const {
        result.accepted = accepted_.size();
        result.pending = pending_.size();
        return result;
    }

    OnlineBackgroundOptions options_;
    std::map<Key, Pending> pending_;
    std::map<Key, Eigen::Vector3d> accepted_;
    Cloud::ConstPtr accepted_cloud_;
    uint64_t generation_ = 0;
    double last_stamp_ = -std::numeric_limits<double>::infinity();
    double last_write_stamp_ = -std::numeric_limits<double>::infinity();
};

}  // namespace mot_mapping
#endif

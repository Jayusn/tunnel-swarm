#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace PayloadMPC
{

struct DisturbanceSlewLimitResult
{
    Eigen::Vector3d value{Eigen::Vector3d::Zero()};
    bool valid{false};
    bool limited_xy{false};
    bool limited_z{false};
};

class DisturbanceSlewLimiter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DisturbanceSlewLimitResult update(const Eigen::Vector3d &target,
                                      double dt,
                                      double max_rate_xy,
                                      double max_rate_z,
                                      double max_norm = std::numeric_limits<double>::infinity())
    {
        DisturbanceSlewLimitResult result;
        if (!target.allFinite() || !std::isfinite(dt) || dt <= 0.0 ||
            !std::isfinite(max_rate_xy) || max_rate_xy <= 0.0 ||
            !std::isfinite(max_rate_z) || max_rate_z <= 0.0 ||
            std::isnan(max_norm) || max_norm <= 0.0)
        {
            reset();
            return result;
        }

        Eigen::Vector3d bounded_target = target;
        if (bounded_target.norm() > max_norm)
            bounded_target *= max_norm / bounded_target.norm();
        const Eigen::Vector3d previous = value_;
        const Eigen::Vector2d target_xy = bounded_target.head<2>();
        const Eigen::Vector2d delta_xy = target_xy - value_.head<2>();
        const double max_delta_xy = max_rate_xy * dt;
        const double delta_xy_norm = delta_xy.norm();
        if (delta_xy_norm > max_delta_xy)
        {
            value_.head<2>() += delta_xy * (max_delta_xy / delta_xy_norm);
            result.limited_xy = true;
        }
        else
        {
            value_.head<2>() = target_xy;
        }

        const double delta_z = bounded_target.z() - value_.z();
        const double max_delta_z = max_rate_z * dt;
        if (std::abs(delta_z) > max_delta_z)
        {
            value_.z() += std::copysign(max_delta_z, delta_z);
            result.limited_z = true;
        }
        else
        {
            value_.z() = bounded_target.z();
        }

        // XY/Z 分别限速可能在转向时暂时超出原有合力上限。
        // 此时沿旧值到目标的线段前进，同时满足合力上限与两轴变化率。
        if (value_.norm() > max_norm)
        {
            double fraction = 1.0;
            if (delta_xy_norm > max_delta_xy)
                fraction = std::min(fraction, max_delta_xy / delta_xy_norm);
            if (std::abs(delta_z) > max_delta_z)
                fraction = std::min(fraction, max_delta_z / std::abs(delta_z));
            value_ = previous + fraction * (bounded_target - previous);
        }

        result.value = value_;
        result.valid = true;
        return result;
    }

    void reset()
    {
        value_.setZero();
    }

    const Eigen::Vector3d &value() const
    {
        return value_;
    }

private:
    Eigen::Vector3d value_{Eigen::Vector3d::Zero()};
};

} // namespace PayloadMPC

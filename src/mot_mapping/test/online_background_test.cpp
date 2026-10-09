#include "online_background.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using mot_mapping::OnlineBackground;
using mot_mapping::OnlineBackgroundOptions;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

Cloud single(double x = 1.01) {
    Cloud cloud;
    cloud.push_back(pcl::PointXYZ(x, 0.01, 0.01));
    return cloud;
}

void stationaryAndOneVote() {
    OnlineBackground background;
    Cloud wall;
    for (int y = 0; y < 10; ++y)
        for (int z = 0; z < 10; ++z)
            for (int duplicate = 0; duplicate < 20; ++duplicate)
                wall.push_back(pcl::PointXYZ(1.01, y * 0.05 + 0.01, z * 0.05 + 0.01));
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for (int frame = 0; frame < 4; ++frame) {
        const auto result = background.observe(wall, {}, frame * 0.1, false, origin);
        require(result.accepted == 0, "duplicate points became multiple observations");
        require(result.pending == 100, "world voxel partition changed");
    }
    const auto result = background.observe(wall, {}, 0.4, false, origin);
    require(result.changed && result.added == 100 && result.accepted == 100,
            "stable new wall did not enter local background");
    require(result.pending == 0, "accepted voxels retained pending votes");
    require(background.acceptedCloud()->size() == 100, "accepted cloud does not match voxel map");
}

void durationAlternativeAndWriteInterval() {
    OnlineBackgroundOptions options;
    options.min_observations = 20;
    OnlineBackground background(options);
    for (int frame = 0; frame < 3; ++frame)
        background.observe(single(), {}, frame * 0.2, false, Eigen::Vector3d::Zero());
    auto result = background.observe(single(), {}, 0.6, false, Eigen::Vector3d::Zero());
    require(result.added == 1, "four stable observations spanning .5s were not accepted");
    // A distinct voxel has four stable observations spanning .6s while the
    // previous write is recent; it must wait for the .3s write interval.
    OnlineBackgroundOptions faster;
    faster.min_observations = 4;
    faster.min_duration = 0.5;
    OnlineBackground quick(faster);
    for (int frame = 0; frame < 4; ++frame)
        quick.observe(single(), {}, frame * 0.1, false, Eigen::Vector3d::Zero());
    for (int frame = 4; frame < 8; ++frame) {
        result = quick.observe(single(2.01), {}, 0.3 + (frame - 3) * 0.01,
                               false, Eigen::Vector3d::Zero());
        require(result.added == 0, "background committed twice inside .3s");
    }
    result = quick.observe(single(2.01), {}, 0.61, false, Eigen::Vector3d::Zero());
    require(result.added == 1, "ready voxel was not committed after write interval");
}

void interruptionAndExclusion() {
    for (int interruption = 0; interruption < 3; ++interruption) {
        OnlineBackground background;
        for (int frame = 0; frame < 4; ++frame)
            background.observe(single(), {}, frame * 0.1, false, Eigen::Vector3d::Zero());
        const Cloud cloud = interruption == 0 ? Cloud() : single();
        const std::vector<uint8_t> excluded = interruption == 1 ? std::vector<uint8_t>{1}
                                                               : std::vector<uint8_t>();
        background.observe(cloud, excluded, 0.4, interruption == 2, Eigen::Vector3d::Zero());
        require(background.pendingSize() == 0, "gap, foreground exclusion or pose pause retained votes");
        for (int frame = 0; frame < 4; ++frame) {
            const auto result = background.observe(single(), {}, 0.5 + frame * 0.1,
                                                   false, Eigen::Vector3d::Zero());
            require(result.accepted == 0, "new sequence borrowed interrupted observation history");
        }
        require(background.observe(single(), {}, 0.9, false, Eigen::Vector3d::Zero()).added == 1,
                "fresh stable sequence failed after interruption");
    }
    OnlineBackground mixed;
    Cloud same_voxel = single();
    same_voxel.push_back(pcl::PointXYZ(1.02, 0.01, 0.01));
    for (int frame = 0; frame < 20; ++frame)
        mixed.observe(same_voxel, {0, 1}, frame * 0.1, false, Eigen::Vector3d::Zero());
    require(mixed.acceptedCloud()->empty(), "unexcluded point absorbed candidate in the same voxel");
}

void unstableAndStopping() {
    OnlineBackground jitter;
    for (int frame = 0; frame < 20; ++frame)
        jitter.observe(single(frame % 2 ? 1.045 : 1.005), {}, frame * 0.1,
                       false, Eigen::Vector3d::Zero());
    require(jitter.acceptedCloud()->empty(), "4cm sampling displacement was accepted as stable");
    OnlineBackground moving;
    for (int frame = 0; frame < 10; ++frame)
        moving.observe(single(1.001 + (frame % 4) * 0.012), {}, frame * 0.1,
                       false, Eigen::Vector3d::Zero());
    require(moving.acceptedCloud()->empty(), "in-voxel moving point entered background");
    for (int frame = 0; frame < 4; ++frame) {
        const auto result = moving.observe(single(1.049), {}, 1.0 + frame * 0.1,
                                           false, Eigen::Vector3d::Zero());
        require(result.accepted == 0, "stopped moving point inherited earlier duration");
    }
    require(moving.observe(single(1.049), {}, 1.4, false, Eigen::Vector3d::Zero()).added == 1,
            "stable sequence following motion could not become background");
}

void sixPointTargetAndPrune() {
    OnlineBackground background;
    Cloud sphere;
    sphere.push_back(pcl::PointXYZ(1.11, 0.01, 0.01));
    sphere.push_back(pcl::PointXYZ(0.91, 0.01, 0.01));
    sphere.push_back(pcl::PointXYZ(1.01, 0.11, 0.01));
    sphere.push_back(pcl::PointXYZ(1.01, -0.09, 0.01));
    sphere.push_back(pcl::PointXYZ(1.01, 0.01, 0.11));
    sphere.push_back(pcl::PointXYZ(1.01, 0.01, -0.09));
    const std::vector<uint8_t> excluded(sphere.size(), 1);
    for (int frame = 0; frame < 20; ++frame)
        background.observe(sphere, excluded, frame * 0.1, false, Eigen::Vector3d::Zero());
    require(background.acceptedCloud()->empty(), "excluded 20cm six-point target was absorbed");
    Cloud plane;
    for (int x = 0; x < 3; ++x)
        for (int y = 0; y < 2; ++y)
            plane.push_back(pcl::PointXYZ(1.01 + x * 0.1, 0.01 + y * 0.2, 0.01));
    for (int frame = 0; frame < 20; ++frame)
        background.observe(plane, excluded, 2.0 + frame * 0.1,
                           false, Eigen::Vector3d::Zero());
    require(background.acceptedCloud()->empty(), "excluded 20cm six-point plane was absorbed");
    OnlineBackgroundOptions options;
    options.local_radius = 2.0;
    OnlineBackground local(options);
    for (int frame = 0; frame < 5; ++frame)
        local.observe(single(), {}, frame * 0.1, false, Eigen::Vector3d::Zero());
    const auto before = local.acceptedCloud();
    const auto result = local.observe(Cloud(), {}, 0.8, false, Eigen::Vector3d(5, 0, 0));
    require(result.changed && result.removed == 1 && result.accepted == 0,
            "local background retained distant old scene");
    require(before->size() == 1, "cloud snapshot mutated during background rebuild");
    for (int frame = 0; frame < 5; ++frame)
        local.observe(single(8.01), {}, 0.9 + frame * 0.1, false, Eigen::Vector3d(5, 0, 0));
    require(local.pendingSize() == 0 && local.acceptedCloud()->empty(),
            "out-of-range voxel made local background unbounded");
}

void repeatedTimestamp() {
    OnlineBackground background;
    for (int frame = 0; frame < 10; ++frame)
        background.observe(single(), {}, 0.1, false, Eigen::Vector3d::Zero());
    require(background.acceptedCloud()->empty(), "replayed timestamp counted as real observation");
}
}  // namespace

int main() {
    try {
        stationaryAndOneVote();
        durationAlternativeAndWriteInterval();
        interruptionAndExclusion();
        unstableAndStopping();
        sixPointTargetAndPrune();
        repeatedTimestamp();
        std::cout << "PASS online background: stable new wall, unique votes, four-frame residuals, "
                     "duration/write interval, foreground/pose/gap exclusion, stop motion, "
                     "20cm six-point target, local prune" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "FAIL online background: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}

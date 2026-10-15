#include "density_partition.h"

#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using mot_mapping::DensityPartitionOptions;
using mot_mapping::DensityPartitionResult;
using mot_mapping::densityPartition;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

pcl::PointIndices allIndices(const Cloud& cloud) {
    pcl::PointIndices indices;
    for (std::size_t i = 0; i < cloud.size(); ++i) indices.indices.push_back(static_cast<int>(i));
    return indices;
}

void checkPartition(const pcl::PointIndices& parent, const DensityPartitionResult& result) {
    std::map<int, int> occurrences;
    for (const pcl::PointIndices& group : result.clusters) {
        require(group.indices.size() >= 6, "published partition has fewer than six points");
        for (int index : group.indices) ++occurrences[index];
    }
    for (int index : result.leftovers.indices) ++occurrences[index];
    require(occurrences.size() == parent.indices.size(), "partition does not cover parent");
    for (int index : parent.indices)
        require(occurrences[index] == 1, "partition index missing or assigned repeatedly");
}

void checkSparseFallback() {
    Cloud sphere;
    sphere.push_back(pcl::PointXYZ(0.1, 0, 0));
    sphere.push_back(pcl::PointXYZ(-0.1, 0, 0));
    sphere.push_back(pcl::PointXYZ(0, 0.1, 0));
    sphere.push_back(pcl::PointXYZ(0, -0.1, 0));
    sphere.push_back(pcl::PointXYZ(0, 0, 0.1));
    sphere.push_back(pcl::PointXYZ(0, 0, -0.1));
    const pcl::PointIndices parent = allIndices(sphere);
    for (double radius : {0.08, 0.10}) {
        for (double alpha : {0.5, 0.7}) {
            DensityPartitionOptions options;
            options.radius = radius;
            options.alpha = alpha;
            const DensityPartitionResult result = densityPartition(sphere, parent, options);
            require(!result.split && result.clusters.size() == 1 && result.leftovers.indices.empty(),
                    "20 cm six-point sphere was split");
            require(result.clusters[0].indices == parent.indices, "sparse sphere lost points");
            checkPartition(parent, result);
        }
    }
    Cloud plane;
    for (int x = 0; x < 3; ++x)
        for (int y = 0; y < 2; ++y) plane.push_back(pcl::PointXYZ(x * 0.1, y * 0.1, 0));
    const pcl::PointIndices plane_parent = allIndices(plane);
    const DensityPartitionResult result = densityPartition(plane, plane_parent);
    require(!result.split && result.clusters[0].indices == plane_parent.indices,
            "20 cm six-point plane was split");
    checkPartition(plane_parent, result);
}

Cloud capWithRope(int samples, int rope_points) {
    Cloud cloud;
    const double spacing = samples == 3 ? 0.02 : 0.012;
    for (int x = 0; x < samples; ++x) {
        for (int y = 0; y < samples; ++y) {
            const double px = (x - (samples - 1) * 0.5) * spacing;
            const double py = (y - (samples - 1) * 0.5) * spacing;
            cloud.push_back(pcl::PointXYZ(px, py, -std::sqrt(0.01 - px * px - py * py)));
        }
    }
    for (int z = 0; z < rope_points; ++z)
        cloud.push_back(pcl::PointXYZ(0, 0, -0.06 + 0.04 * z));
    return cloud;
}

void checkBallAndRope() {
    for (int samples : {3, 7}) {
        const Cloud cloud = capWithRope(samples, 21);
        const pcl::PointIndices parent = allIndices(cloud);
        for (double radius : {0.08, 0.10}) {
            for (double alpha : {0.5, 0.7}) {
                DensityPartitionOptions options;
                options.radius = radius;
                options.alpha = alpha;
                const DensityPartitionResult result = densityPartition(cloud, parent, options);
                require(result.split && result.clusters.size() >= 2, "cap and long rope did not separate");
                checkPartition(parent, result);
                const pcl::PointIndices& body = result.clusters.front();
                for (int index = 0; index < samples * samples; ++index)
                    require(std::binary_search(body.indices.begin(), body.indices.end(), index),
                            "ball surface lost to rope");
                require(!std::binary_search(body.indices.begin(), body.indices.end(),
                                           static_cast<int>(cloud.size() - 1)),
                        "border expansion followed entire rope");
            }
        }
    }
    const Cloud cloud = capWithRope(3, 5);
    const pcl::PointIndices parent = allIndices(cloud);
    const DensityPartitionResult result = densityPartition(cloud, parent);
    require(result.split && result.clusters.size() == 1 && !result.leftovers.indices.empty(),
            "short rope residual should remain visible in leftovers");
    checkPartition(parent, result);
}

void checkUniformPlane() {
    Cloud cloud;
    for (int x = 0; x < 21; ++x)
        for (int y = 0; y < 21; ++y) cloud.push_back(pcl::PointXYZ(x * 0.02, y * 0.02, 0));
    const pcl::PointIndices parent = allIndices(cloud);
    for (double radius : {0.08, 0.10}) {
        for (double alpha : {0.5, 0.7}) {
            DensityPartitionOptions options;
            options.radius = radius;
            options.alpha = alpha;
            const DensityPartitionResult result = densityPartition(cloud, parent, options);
            require(!result.split && result.clusters.size() == 1 && result.leftovers.indices.empty(),
                    "uniform plane was arbitrarily cropped");
            require(result.clusters[0].indices == parent.indices, "uniform plane lost points");
            checkPartition(parent, result);
        }
    }
}

void checkConfiguredResidualRules() {
    const Cloud cloud = capWithRope(3, 21);
    const pcl::PointIndices parent = allIndices(cloud);
    for (int variant = 0; variant < 2; ++variant) {
        DensityPartitionOptions options;
        if (variant == 0) options.core_pts = 100;
        if (variant == 1) options.max_cluster = 6;
        const DensityPartitionResult result = densityPartition(cloud, parent, options);
        require(result.split && result.clusters.size() == 1 && !result.leftovers.indices.empty(),
                "configured residual rule was ignored");
        checkPartition(parent, result);
    }
    DensityPartitionOptions options;
    options.squared_tolerance = 0.00001;
    const DensityPartitionResult disconnected = densityPartition(cloud, parent, options);
    require(!disconnected.split && disconnected.clusters[0].indices == parent.indices,
            "configured core connection radius was ignored");
    checkPartition(parent, disconnected);
    options.squared_tolerance = 0.02;
    options.min_cluster = 20;
    const DensityPartitionResult result = densityPartition(cloud, parent, options);
    require(!result.split && result.clusters[0].indices == parent.indices,
            "core component smaller than configured minimum was accepted");
    checkPartition(parent, result);
}

void checkSeparatedSpherePatches() {
    for (double separation : {0.11, 0.15}) {
        Cloud cloud;
        // Two observed patches on the same physical 20 cm sphere. Their
        // nearest surface samples are 8.6/12.6 cm apart, beyond the density
        // radius but inside the original DBSCAN connection radius.
        for (double patch_z : {-separation / 2, separation / 2}) {
            for (double dy : {-0.012, 0.0, 0.012}) {
                for (double dz : {-0.012, 0.0, 0.012}) {
                    const double z = patch_z + dz;
                    cloud.push_back(pcl::PointXYZ(std::sqrt(0.01 - dy * dy - z * z), dy, 1 + z));
                }
            }
        }
        const pcl::PointIndices body_parent = allIndices(cloud);
        const DensityPartitionResult body = densityPartition(cloud, body_parent);
        require(!body.split && body.clusters.size() == 1 &&
                body.clusters[0].indices == body_parent.indices,
                "two real sphere surface patches were split into separate bodies");
        checkPartition(body_parent, body);

        const double rope_x = std::sqrt(0.01 - separation * separation / 4);
        for (int point = 0; point < 21; ++point)
            cloud.push_back(pcl::PointXYZ(rope_x, 0, 1 + separation / 2 + 0.065 + point * 0.04));
        const pcl::PointIndices parent = allIndices(cloud);
        const DensityPartitionResult result = densityPartition(cloud, parent);
        require(result.split && result.clusters.size() >= 2,
                "separated sphere patches and long rope did not partition");
        for (int index : body_parent.indices)
            require(std::binary_search(result.clusters[0].indices.begin(),
                                       result.clusters[0].indices.end(), index),
                    "one sphere surface patch was lost to rope residual");
        require(!std::binary_search(result.clusters[0].indices.begin(),
                                    result.clusters[0].indices.end(), cloud.size() - 1),
                "original connection radius extended through non-core rope");
        checkPartition(parent, result);
    }
}

void checkDeterministicTieAndParentIsolation() {
    Cloud cloud;
    for (double x : {-0.075, 0.075})
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; ++z) cloud.push_back(pcl::PointXYZ(x, y * 0.01, z * 0.01));
    cloud.push_back(pcl::PointXYZ(0, 0, 0));
    pcl::PointIndices parent = allIndices(cloud);
    // Points outside this parent would change its maximum degree if queried.
    for (int i = 0; i < 100; ++i) cloud.push_back(pcl::PointXYZ(0, 0, 0));
    DensityPartitionOptions options;
    options.alpha = 0.5;
    const DensityPartitionResult result = densityPartition(cloud, parent, options);
    require(result.split && result.clusters.size() == 2, "two dense cores were not separated");
    require(result.clusters[0].indices.size() == 10 && result.clusters[1].indices.size() == 9,
            "equidistant border did not choose smallest original component index");
    require(std::binary_search(result.clusters[0].indices.begin(), result.clusters[0].indices.end(), 18),
            "equidistant border went to wrong component");
    checkPartition(parent, result);
    std::reverse(parent.indices.begin(), parent.indices.end());
    const DensityPartitionResult reversed = densityPartition(cloud, parent, options);
    require(reversed.clusters.size() == result.clusters.size(), "parent order changed partition");
    for (std::size_t i = 0; i < result.clusters.size(); ++i)
        require(reversed.clusters[i].indices == result.clusters[i].indices,
                "parent order changed deterministic component assignment");
    require(reversed.leftovers.indices == result.leftovers.indices, "parent order changed leftovers");
    checkPartition(parent, reversed);
}

}  // namespace

int main() {
    try {
        checkSparseFallback();
        checkBallAndRope();
        checkUniformPlane();
        checkConfiguredResidualRules();
        checkSeparatedSpherePatches();
        checkDeterministicTieAndParentIsolation();
        std::cout << "PASS density partition: 20 cm six-point fallback, cap/rope, complete partition, "
                     "uniform plane, parent isolation, deterministic ties" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "FAIL density partition: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}

#ifndef MOT_MAPPING_DENSITY_PARTITION_H
#define MOT_MAPPING_DENSITY_PARTITION_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "DBSCAN_simple.h"

namespace mot_mapping {

struct DensityPartitionOptions {
    double radius = 0.08;  // Metres; unlike DBSCANSimpleCluster's squared radius.
    double alpha = 0.7;
    int min_cluster = 6;
    int max_cluster = 800;
    int core_pts = 3;
    double squared_tolerance = 0.02;
};

struct DensityPartitionResult {
    std::vector<pcl::PointIndices> clusters;
    pcl::PointIndices leftovers;
    bool split = false;
};

// Partition one existing cluster only. Every returned index refers to the input
// cloud, and remains in exactly one cluster or in leftovers. Sparse objects that
// do not contain a supported two-dimensional core keep their original cluster.
inline DensityPartitionResult densityPartition(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const pcl::PointIndices& parent,
    const DensityPartitionOptions& options = DensityPartitionOptions()) {
    DensityPartitionResult fallback;
    fallback.leftovers.header = parent.header;
    if (parent.indices.empty()) return fallback;

    pcl::PointIndices original = parent;
    std::sort(original.indices.begin(), original.indices.end());
    original.indices.erase(std::unique(original.indices.begin(), original.indices.end()),
                           original.indices.end());
    fallback.clusters.push_back(original);
    const std::size_t minimum_cluster_size = static_cast<std::size_t>(std::max(6, options.min_cluster));
    if (original.indices.size() < minimum_cluster_size || !std::isfinite(options.radius) ||
        options.radius <= 0.0 || !std::isfinite(options.alpha) ||
        options.alpha <= 0.0 || options.alpha > 1.0 || options.max_cluster < 1 ||
        options.core_pts < 1 || !std::isfinite(options.squared_tolerance) ||
        options.squared_tolerance <= 0.0) return fallback;

    pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
    local->reserve(original.indices.size());
    for (int index : original.indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud.size()) return fallback;
        const pcl::PointXYZ& point = cloud[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            return fallback;
        local->push_back(point);
    }

    pcl::search::KdTree<pcl::PointXYZ> tree(false);
    tree.setInputCloud(local);
    const std::size_t count = local->size();
    std::vector<std::vector<int>> neighbors(count);
    std::vector<float> distances;
    std::size_t maximum_degree = 0;
    for (std::size_t i = 0; i < count; ++i) {
        tree.radiusSearch(static_cast<int>(i), options.radius, neighbors[i], distances);
        maximum_degree = std::max(maximum_degree, neighbors[i].size());
    }
    const std::size_t minimum_degree = std::max<std::size_t>(
        6, static_cast<std::size_t>(std::ceil(options.alpha * maximum_degree)));
    std::vector<bool> core(count, false);
    for (std::size_t i = 0; i < count; ++i) {
        if (neighbors[i].size() < minimum_degree) continue;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (int neighbor : neighbors[i]) center += (*local)[neighbor].getVector3fMap().cast<double>();
        center /= static_cast<double>(neighbors[i].size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (int neighbor : neighbors[i]) {
            const Eigen::Vector3d delta = (*local)[neighbor].getVector3fMap().cast<double>() - center;
            covariance.noalias() += delta * delta.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() == Eigen::Success && solver.eigenvalues()[2] > 1e-12 &&
            solver.eigenvalues()[1] / solver.eigenvalues()[2] >= 0.1) core[i] = true;
    }

    std::vector<int> component(count, -1);
    std::vector<std::vector<int>> groups;
    std::vector<int> group_min_index;
    const double core_connection_radius = std::sqrt(options.squared_tolerance);
    std::vector<int> connected;
    std::vector<float> connected_distances;
    for (std::size_t i = 0; i < count; ++i) {
        if (!core[i] || component[i] >= 0) continue;
        const int label = static_cast<int>(groups.size());
        groups.emplace_back(1, static_cast<int>(i));
        group_min_index.push_back(original.indices[i]);
        component[i] = label;
        for (std::size_t head = 0; head < groups[label].size(); ++head) {
            const int current = groups[label][head];
            // Preserve the original cluster's geometric connection between
            // supported surface patches. Only cores may extend connectivity;
            // density/covariance and one-hop border attachment keep radius.
            tree.radiusSearch(current, core_connection_radius, connected, connected_distances);
            for (int neighbor : connected) {
                if (!core[neighbor] || component[neighbor] >= 0) continue;
                component[neighbor] = label;
                groups[label].push_back(neighbor);
            }
        }
    }
    if (groups.empty()) return fallback;

    // A border point may join its nearest core, but never becomes another
    // expansion seed. This prevents a thin chain from joining two surfaces.
    for (std::size_t i = 0; i < count; ++i) {
        if (core[i]) continue;
        double best_distance = std::numeric_limits<double>::infinity();
        int best_component = -1;
        for (int neighbor : neighbors[i]) {
            if (!core[neighbor]) continue;
            const int label = component[neighbor];
            const double distance = ((*local)[i].getVector3fMap().cast<double>() -
                                     (*local)[neighbor].getVector3fMap().cast<double>()).squaredNorm();
            if (distance < best_distance - 1e-12 ||
                (std::abs(distance - best_distance) <= 1e-12 &&
                 (best_component < 0 || group_min_index[label] < group_min_index[best_component]))) {
                best_distance = distance;
                best_component = label;
            }
        }
        if (best_component >= 0) {
            component[i] = best_component;
            groups[best_component].push_back(static_cast<int>(i));
        }
    }

    DensityPartitionResult result;
    result.leftovers.header = parent.header;
    std::vector<bool> assigned(count, false);
    for (const std::vector<int>& group : groups) {
        if (group.size() < minimum_cluster_size) continue;
        pcl::PointIndices cluster;
        cluster.header = parent.header;
        for (int index : group) {
            cluster.indices.push_back(original.indices[index]);
            assigned[index] = true;
        }
        std::sort(cluster.indices.begin(), cluster.indices.end());
        result.clusters.push_back(std::move(cluster));
    }
    if (result.clusters.empty()) return fallback;

    // Keep the existing DBSCAN rule for points outside supported cores. Its
    // nanoflann search argument is squared metres. Its options retain the
    // configured original DBSCAN neighbourhood rather than the core radius.
    pcl::PointCloud<pcl::PointXYZ>::Ptr residual(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> residual_to_local;
    for (std::size_t i = 0; i < count; ++i) {
        if (assigned[i]) continue;
        residual->push_back((*local)[i]);
        residual_to_local.push_back(static_cast<int>(i));
    }
    if (residual->size() >= minimum_cluster_size) {
        DBSCANSimpleCluster<pcl::PointXYZ> dbscan;
        dbscan.setInputCloud(residual);
        dbscan.setSearchMethod();
        dbscan.setClusterTolerance(options.squared_tolerance);
        dbscan.setCorePointMinPts(options.core_pts);
        dbscan.setMinClusterSize(static_cast<int>(minimum_cluster_size));
        dbscan.setMaxClusterSize(options.max_cluster);
        std::vector<pcl::PointIndices> residual_clusters;
        dbscan.extractNano(residual_clusters);
        for (const pcl::PointIndices& group : residual_clusters) {
            pcl::PointIndices cluster;
            cluster.header = parent.header;
            for (int residual_index : group.indices) {
                const int local_index = residual_to_local[residual_index];
                if (assigned[local_index]) continue;
                cluster.indices.push_back(original.indices[local_index]);
            }
            if (cluster.indices.size() < minimum_cluster_size) continue;
            for (int residual_index : group.indices) assigned[residual_to_local[residual_index]] = true;
            std::sort(cluster.indices.begin(), cluster.indices.end());
            result.clusters.push_back(std::move(cluster));
        }
    }
    for (std::size_t i = 0; i < count; ++i)
        if (!assigned[i]) result.leftovers.indices.push_back(original.indices[i]);
    if (result.clusters.size() == 1 && result.leftovers.indices.empty()) return fallback;
    std::sort(result.clusters.begin(), result.clusters.end(),
              [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                  return a.indices.front() < b.indices.front();
              });
    result.split = true;
    return result;
}

}  // namespace mot_mapping

#endif  // MOT_MAPPING_DENSITY_PARTITION_H

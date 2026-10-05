#pragma once

#include <algorithm>
#include <cmath>

namespace dsp_fov {

struct Vector3f {
    float x;
    float y;
    float z;
};

inline Vector3f worldToSensor(const Vector3f &world, float qw, float qx,
                              float qy, float qz) {
    const float norm_sq = qw * qw + qx * qx + qy * qy + qz * qz;
    if (norm_sq <= 1.0e-12f) {
        return world;
    }

    // R(q)^T * world. q describes sensor-to-world rotation.
    const float inv_norm_sq = 1.0f / norm_sq;
    return {
        ((qw * qw + qx * qx - qy * qy - qz * qz) * world.x
         + 2.0f * (qx * qy + qw * qz) * world.y
         + 2.0f * (qx * qz - qw * qy) * world.z) * inv_norm_sq,
        (2.0f * (qx * qy - qw * qz) * world.x
         + (qw * qw - qx * qx + qy * qy - qz * qz) * world.y
         + 2.0f * (qy * qz + qw * qx) * world.z) * inv_norm_sq,
        (2.0f * (qx * qz + qw * qy) * world.x
         + 2.0f * (qy * qz - qw * qx) * world.y
         + (qw * qw - qx * qx - qy * qy + qz * qz) * world.z) * inv_norm_sq};
}

inline bool fullLidarIndex(const Vector3f &sensor, int angle_resolution_deg,
                           int vertical_fov_deg, int &horizontal_index,
                           int &vertical_index) {
    const float horizontal_range = std::hypot(sensor.x, sensor.y);
    if (horizontal_range <= 1.0e-6f && std::fabs(sensor.z) <= 1.0e-6f) {
        return false;
    }

    constexpr float kPi = 3.14159265358979323846f;
    const float elevation = std::atan2(sensor.z, horizontal_range);
    const float half_vertical = 0.5f * vertical_fov_deg * kPi / 180.0f;
    constexpr float kEdgeTolerance = 1.0e-6f;
    if (elevation < -half_vertical - kEdgeTolerance ||
        elevation > half_vertical + kEdgeTolerance) {
        return false;
    }

    const int horizontal_bins = 360 / angle_resolution_deg;
    const int vertical_bins = vertical_fov_deg / angle_resolution_deg;
    float azimuth = std::atan2(sensor.y, sensor.x);
    if (azimuth >= kPi) {
        azimuth -= 2.0f * kPi;
    }
    horizontal_index = static_cast<int>(
        std::floor((azimuth + kPi) * 180.0f / kPi / angle_resolution_deg));
    horizontal_index = std::max(0, std::min(horizontal_bins - 1, horizontal_index));

    vertical_index = static_cast<int>(std::floor(
        (elevation + half_vertical) * 180.0f / kPi / angle_resolution_deg));
    vertical_index = std::max(0, std::min(vertical_bins - 1, vertical_index));
    return true;
}

inline int neighborIndices(int index, int horizontal_bins, int vertical_bins,
                           bool wrap_horizontal, int *neighbors) {
    const int original_h = index / vertical_bins;
    const int original_v = index % vertical_bins;
    int count = 0;
    for (int dh = -1; dh <= 1; ++dh) {
        int h = original_h + dh;
        if (wrap_horizontal) {
            h = (h + horizontal_bins) % horizontal_bins;
        } else if (h < 0 || h >= horizontal_bins) {
            continue;
        }
        for (int dv = -1; dv <= 1; ++dv) {
            const int v = original_v + dv;
            if (v >= 0 && v < vertical_bins) {
                neighbors[count++] = h * vertical_bins + v;
            }
        }
    }
    return count;
}

inline bool clusterGeometryWithinLimits(float max_z, float span_xy,
                                        float voxel_resolution,
                                        bool full_360_lidar_fov) {
    constexpr float kMaximumZ = 2.60f;
    constexpr float kMaximumSpanXY = 1.15f;
    const float max_z_allowance = full_360_lidar_fov ? voxel_resolution : 0.0f;
    const float span_allowance = full_360_lidar_fov
            ? std::sqrt(2.0f) * voxel_resolution
            : 0.0f;
    return max_z <= kMaximumZ + max_z_allowance &&
           span_xy <= kMaximumSpanXY + span_allowance;
}

}  // namespace dsp_fov

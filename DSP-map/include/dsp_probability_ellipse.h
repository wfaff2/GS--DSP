#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace dsp_probability {

constexpr std::int64_t kInvalidObstacleId = -1;

// This package is built with -ffast-math, under which compiler assumptions can
// invalidate std::isfinite checks. Inspect the IEEE-754 exponent bits instead
// so malformed particle data is still rejected in optimized builds.
inline bool isFiniteScalar(double value) {
    static_assert(sizeof(double) == sizeof(std::uint64_t),
                  "Finite-value check requires a 64-bit double");
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT64_C(0x7ff0000000000000)) !=
           UINT64_C(0x7ff0000000000000);
}

template <typename Derived>
inline bool matrixIsFinite(const Eigen::MatrixBase<Derived> &matrix) {
    for(Eigen::Index i = 0; i < matrix.size(); ++i) {
        if(!isFiniteScalar(static_cast<double>(matrix.derived().coeff(i)))) return false;
    }
    return true;
}

class PersistentIdAllocator {
public:
    explicit PersistentIdAllocator(std::int64_t first_id = 0) : next_id_(first_id) {}

    std::int64_t inheritOrAllocate(std::int64_t previous_id) {
        if(previous_id >= 0) {
            if(previous_id >= next_id_) next_id_ = previous_id + 1;
            return previous_id;
        }
        return next_id_++;
    }

    std::int64_t nextId() const { return next_id_; }

private:
    std::int64_t next_id_;
};

struct WeightedMoments {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double weight = 0.0;
    Eigen::Vector3d first_moment = Eigen::Vector3d::Zero();
    Eigen::Matrix2d second_moment_xy = Eigen::Matrix2d::Zero();
    std::size_t sample_count = 0;

    bool add(double x, double y, double z, double sample_weight) {
        if(!isFiniteScalar(x) || !isFiniteScalar(y) || !isFiniteScalar(z) ||
           !isFiniteScalar(sample_weight) || sample_weight <= 0.0) {
            return false;
        }
        weight += sample_weight;
        first_moment += sample_weight * Eigen::Vector3d(x, y, z);
        const Eigen::Vector2d xy(x, y);
        second_moment_xy += sample_weight * xy * xy.transpose();
        ++sample_count;
        return true;
    }
};

struct ProbabilityEllipseConfig {
    double probability_mass = 0.95;
    double covariance_regularization = 1e-4;
    double minimum_weight = 1e-3;
    std::size_t minimum_samples = 3;
};

struct ProbabilityEllipse {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    double semi_major = 0.0;
    double semi_minor = 0.0;
    double yaw = 0.0;
    double confidence = 0.0;
    double total_weight = 0.0;
    std::size_t sample_count = 0;
    bool valid = false;
};

inline double normalizeEllipseYaw(double yaw) {
    const double pi = std::acos(-1.0);
    while(yaw >= 0.5 * pi) yaw -= pi;
    while(yaw < -0.5 * pi) yaw += pi;
    return yaw;
}

// Constructs a Gaussian moment approximation of the discrete, weighted DSP
// particle distribution. This does not assert that the native DSP distribution
// is Gaussian. The covariance includes object spatial extent together with
// measurement, motion-model, and particle/resampling spread. It deliberately
// contains no vehicle radius, controller uncertainty, or safety inflation.
inline ProbabilityEllipse constructProbabilityEllipse(
        const WeightedMoments &moments,
        const ProbabilityEllipseConfig &config) {
    ProbabilityEllipse result;
    result.total_weight = moments.weight;
    result.sample_count = moments.sample_count;
    result.confidence = config.probability_mass;

    if(!isFiniteScalar(moments.weight) || moments.weight < config.minimum_weight ||
       moments.sample_count < config.minimum_samples ||
       !isFiniteScalar(config.probability_mass) || config.probability_mass <= 0.0 ||
       config.probability_mass >= 1.0 ||
       !isFiniteScalar(config.covariance_regularization) ||
       config.covariance_regularization <= 0.0 ||
       !isFiniteScalar(config.minimum_weight) || config.minimum_weight < 0.0) {
        return result;
    }

    result.center = moments.first_moment / moments.weight;
    if(!matrixIsFinite(result.center)) return result;

    const Eigen::Vector2d mean_xy = result.center.head<2>();
    result.covariance = moments.second_moment_xy / moments.weight
                        - mean_xy * mean_xy.transpose();
    result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
    result.covariance.diagonal().array() += config.covariance_regularization;
    if(!matrixIsFinite(result.covariance)) return result;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(result.covariance);
    if(solver.info() != Eigen::Success || !matrixIsFinite(solver.eigenvalues()) ||
       !matrixIsFinite(solver.eigenvectors())) {
        return result;
    }

    const double lambda_minor = std::max(solver.eigenvalues()(0),
                                         config.covariance_regularization);
    const double lambda_major = std::max(solver.eigenvalues()(1), lambda_minor);
    Eigen::Vector2d clamped_eigenvalues;
    clamped_eigenvalues << lambda_minor, lambda_major;
    result.covariance = solver.eigenvectors() * clamped_eigenvalues.asDiagonal()
                        * solver.eigenvectors().transpose();
    result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
    const double chi_square_scale = -2.0 * std::log1p(-config.probability_mass);
    result.semi_major = std::sqrt(chi_square_scale * lambda_major);
    result.semi_minor = std::sqrt(chi_square_scale * lambda_minor);
    const Eigen::Vector2d major_axis = solver.eigenvectors().col(1);
    result.yaw = normalizeEllipseYaw(std::atan2(major_axis.y(), major_axis.x()));
    result.valid = isFiniteScalar(result.semi_major) &&
                   isFiniteScalar(result.semi_minor) &&
                   isFiniteScalar(result.yaw) && result.semi_major >= result.semi_minor &&
                   result.semi_minor > 0.0;
    return result;
}

}  // namespace dsp_probability

#pragma once

#include <acado_optimal_control.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace coni_mpc {

// Owns [px, py, pz, P_occ] OnlineData blocks in the pooling output order.
// n_obs is a generation-time dimension; all coefficients come from the caller.
// Smooth switching is evaluated at the optimized position inside ACADO.
class VoxelBarrierModel {
 public:
  VoxelBarrierModel(std::size_t n_obs, double r_cut, double sigma,
                    double d_safe_robust, double distance_epsilon,
                    double partition_floor, double k_steep)
      : m_n_obs(validateCount(n_obs)), m_r_cut(r_cut), m_sigma(sigma),
        m_d_safe_robust(d_safe_robust), m_distance_epsilon(distance_epsilon),
        m_partition_floor(partition_floor), m_k_steep(k_steep),
        m_voxels("pooled_voxels", static_cast<unsigned>(4 * m_n_obs), 1) {
    if (!std::isfinite(r_cut) || r_cut <= 0.0 ||
        !std::isfinite(sigma) || sigma <= 0.0 ||
        !std::isfinite(d_safe_robust) || d_safe_robust < 0.0 ||
        !std::isfinite(distance_epsilon) || distance_epsilon <= 0.0 ||
        !std::isfinite(partition_floor) || partition_floor <= 0.0 ||
        !std::isfinite(distance_epsilon * distance_epsilon) ||
        distance_epsilon * distance_epsilon == 0.0 ||
        partition_floor >= std::exp(-d_safe_robust / sigma) ||
        !std::isfinite(k_steep) || k_steep <= 0.0 ||
        !std::isfinite(2.0 * k_steep * r_cut) ||
        2.0 * k_steep * r_cut >= std::log(std::numeric_limits<double>::max())) {
      throw std::invalid_argument("Invalid voxel barrier parameters: require "
          "positive finite radii/sigma/regularizers, nonnegative safety "
          "distance, floor < exp(-d_safe_robust/sigma), and "
          "0 < 2*k_steep*r_cut < log(DBL_MAX)");
    }
  }

  ACADO::Expression assemble(ACADO::OCP& ocp, const ACADO::Expression& x,
                            const ACADO::Expression& y,
                            const ACADO::Expression& z) const {
    using ACADO::Expression;
    Expression S = 0.0;
    for (std::size_t i = 0; i < m_n_obs; ++i) {
      const unsigned offset = static_cast<unsigned>(4 * i);
      const Expression dx = x - m_voxels(offset);
      const Expression dy = y - m_voxels(offset + 1);
      const Expression dz = z - m_voxels(offset + 2);
      // Avoid sqrt's AD singularity at a voxel center, even for P_occ=0.
      const Expression d = sqrt(dx * dx + dy * dy + dz * dz +
          m_distance_epsilon * m_distance_epsilon);
      const Expression q = d / m_r_cut;
      const Expression w_poly = pow(1.0 - q, 4) * (4.0 * q + 1.0);
      // 在预测位置上计算 tanh 平滑开关，近处激活、远处衰减。
      // 本地 ACADO 无符号 tanh；以下等价于 0.5+0.5*tanh(z)，避免相减消差。
      const Expression z_switch = m_k_steep * (m_r_cut - d);
      const Expression exp_twice_z = exp(2.0 * z_switch);
      const Expression smooth_switch = exp_twice_z / (1.0 + exp_twice_z);
      const Expression w = w_poly * smooth_switch;
      S = S + w * m_voxels(offset + 3) * exp(-d / m_sigma);
    }
    // An additive floor keeps the empty field finite and reduces d_eval.
    const Expression d_eval = -m_sigma * log(S + m_partition_floor);
    ocp.subjectTo(d_eval - m_d_safe_robust >= 0.0);
    return d_eval;
  }

  unsigned onlineDataSize() const { return static_cast<unsigned>(4 * m_n_obs); }

 private:
  static std::size_t validateCount(std::size_t count) {
    if (count == 0 || count > std::numeric_limits<unsigned>::max() / 4) {
      throw std::invalid_argument("n_obs must fit the ACADO OnlineData dimension");
    }
    return count;
  }
  std::size_t m_n_obs;
  double m_r_cut;
  double m_sigma;
  double m_d_safe_robust;
  double m_distance_epsilon;
  double m_partition_floor;
  double m_k_steep;
  ACADO::OnlineData m_voxels;
};

}  // namespace coni_mpc

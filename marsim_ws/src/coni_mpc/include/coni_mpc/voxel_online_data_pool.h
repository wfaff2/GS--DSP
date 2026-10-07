#pragma once

#include "coni_mpc/field_hocbf.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace coni_mpc {

class VoxelOnlineDataPool {
 public:
  VoxelOnlineDataPool(double pool_leaf_size, std::size_t n_obs,
                      double padding_distance)
      : m_pool_leaf_size(pool_leaf_size),
        m_n_obs(n_obs),
        m_padding_distance(padding_distance) {
    if (!std::isfinite(m_pool_leaf_size) || !(m_pool_leaf_size > 0.0)) {
      throw std::invalid_argument("pool_leaf_size must be finite and positive");
    }
    if (m_n_obs == 0 ||
        m_n_obs > std::numeric_limits<std::size_t>::max() / 4) {
      throw std::invalid_argument("n_obs must be positive and fit the output");
    }
    if (!std::isfinite(m_padding_distance) ||
        !(m_padding_distance > 0.0)) {
      throw std::invalid_argument(
          "padding_distance must be finite and positive");
    }
  }

  std::vector<double> build(
      const Eigen::Vector3d& pos,
      const field_hocbf::KinematicPoints& raw) const {
    if (!pos.allFinite()) {
      throw std::invalid_argument("pool reference position must be finite");
    }

    const Eigen::Vector3d padding = makePaddingPoint(pos);
    std::vector<double> output(4 * m_n_obs, 0.0);
    for (std::size_t i = 0; i < m_n_obs; ++i) {
      const std::size_t base = 4 * i;
      output[base] = padding.x();
      output[base + 1] = padding.y();
      output[base + 2] = padding.z();
    }

    // HASH POOLING: full 3-D key equality keeps hash collisions distinct.
    std::unordered_map<GridKey, std::size_t, GridKeyHash> grid;
    grid.reserve(raw.size());
    AlignedCells cells;
    cells.reserve(raw.size());

    for (const field_hocbf::KinematicPoint& point : raw) {
      if (!point.position.allFinite() || !std::isfinite(point.occupancy) ||
          !(point.occupancy > 0.0)) {
        continue;
      }

      GridKey key;
      if (!gridKey(point.position, &key)) continue;

      const auto found = grid.find(key);
      std::size_t cell_index = 0;
      if (found == grid.end()) {
        if (cells.size() == std::numeric_limits<std::size_t>::max()) continue;
        cell_index = cells.size();
        Cell cell;
        cell.key = key;
        cells.push_back(cell);
        grid.emplace(key, cell_index);
      } else {
        cell_index = found->second;
      }

      Cell& cell = cells[cell_index];
      const Eigen::Vector3d new_sum = cell.sum.position + point.position;
      if (!new_sum.allFinite() ||
          cell.count == std::numeric_limits<std::size_t>::max()) {
        continue;
      }
      cell.sum.position = new_sum;
      cell.sum.occupancy =
          std::min(1.0, cell.sum.occupancy + std::min(1.0, point.occupancy));
      ++cell.count;
    }

    std::vector<RankedCell> ranked;
    ranked.reserve(cells.size());
    for (std::size_t i = 0; i < cells.size(); ++i) {
      Cell& cell = cells[i];
      if (cell.count == 0) continue;
      cell.sum.position /= static_cast<double>(cell.count);
      if (!cell.sum.position.allFinite()) continue;
      ranked.push_back(
          RankedCell{i, distance(cell.sum.position, pos), cell.key});
    }

    std::sort(ranked.begin(), ranked.end(),
              [](const RankedCell& lhs, const RankedCell& rhs) {
                if (lhs.distance < rhs.distance) return true;
                if (rhs.distance < lhs.distance) return false;
                return lhs.key < rhs.key;
              });

    const std::size_t selected = std::min(m_n_obs, ranked.size());
    for (std::size_t i = 0; i < selected; ++i) {
      const Cell& cell = cells[ranked[i].index];
      const std::size_t base = 4 * i;
      output[base] = cell.sum.position.x();
      output[base + 1] = cell.sum.position.y();
      output[base + 2] = cell.sum.position.z();
      // 保留真实占据率；距离开关由 ACADO 在预测位置上计算。
      output[base + 3] = cell.sum.occupancy;
    }
    return output;
  }

 private:
  struct GridKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const GridKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }

    bool operator<(const GridKey& other) const {
      if (x != other.x) return x < other.x;
      if (y != other.y) return y < other.y;
      return z < other.z;
    }
  };

  struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const {
      std::size_t seed = std::hash<std::int64_t>{}(key.x);
      combine(&seed, std::hash<std::int64_t>{}(key.y));
      combine(&seed, std::hash<std::int64_t>{}(key.z));
      return seed;
    }

   private:
    static void combine(std::size_t* seed, std::size_t value) {
      // Unsigned size_t arithmetic is defined modulo 2^N.
      *seed ^= value + static_cast<std::size_t>(0x9e3779b9U) + (*seed << 6U) +
               (*seed >> 2U);
    }
  };

  struct Cell {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    field_hocbf::KinematicPoint sum;
    std::size_t count = 0;
    GridKey key;
  };
  using AlignedCells = std::vector<Cell, Eigen::aligned_allocator<Cell>>;

  struct RankedCell {
    std::size_t index;
    long double distance;
    GridKey key;
  };

  bool gridKey(const Eigen::Vector3d& point, GridKey* key) const {
    return gridCoordinate(point.x(), &key->x) &&
           gridCoordinate(point.y(), &key->y) &&
           gridCoordinate(point.z(), &key->z);
  }

  bool gridCoordinate(double coordinate, std::int64_t* result) const {
    const long double scaled =
        std::floor(static_cast<long double>(coordinate) /
                   static_cast<long double>(m_pool_leaf_size));
    const long double minimum =
        static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    const long double maximum =
        static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (!std::isfinite(scaled) || scaled < minimum || scaled > maximum) {
      return false;
    }
    *result = static_cast<std::int64_t>(scaled);
    return true;
  }

  static long double distance(const Eigen::Vector3d& lhs,
                              const Eigen::Vector3d& rhs) {
    const long double dx = static_cast<long double>(lhs.x()) - rhs.x();
    const long double dy = static_cast<long double>(lhs.y()) - rhs.y();
    const long double dz = static_cast<long double>(lhs.z()) - rhs.z();
    return std::hypot(std::hypot(dx, dy), dz);
  }

  Eigen::Vector3d makePaddingPoint(const Eigen::Vector3d& pos) const {
    Eigen::Vector3d padding = pos;
    padding.x() += m_padding_distance;
    if (std::isfinite(padding.x()) && distance(padding, pos) > 0.0L) {
      return padding;
    }

    padding = pos;
    padding.x() -= m_padding_distance;
    if (std::isfinite(padding.x()) && distance(padding, pos) > 0.0L) {
      return padding;
    }

    padding = pos;
    padding.x() = std::nextafter(pos.x(), 0.0);
    if (std::isfinite(padding.x()) && distance(padding, pos) > 0.0L) {
      return padding;
    }
    throw std::overflow_error(
        "cannot represent a finite distinct padding point");
  }

  double m_pool_leaf_size;
  std::size_t m_n_obs;
  double m_padding_distance;
};

}  // namespace coni_mpc

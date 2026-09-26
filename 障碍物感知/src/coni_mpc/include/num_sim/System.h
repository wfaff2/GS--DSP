/*    coni_mpc
 *    CoNi-MPC: Cooperative Non-inertial Frame Based Model Predictive Control
 *    Copyright (C) 2023 Baozhe Zhang, 
 *    Fast Lab, Huzhou Institute of Zhejiang University
 *  
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace num_sim
{


template <size_t state_dim, size_t control_dim>
class System
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = state_dim;
  static const size_t CONTROL_DIM = control_dim;

  using State_t = Eigen::Matrix<double, STATE_DIM, 1>;
  using Control_t = Eigen::Matrix<double, CONTROL_DIM, 1>;
  using StateArray_t = typename std::vector<State_t>;
  using TimeArray_t = typename std::vector<double>;

  // default constructor
  System(const std::string &name) :
      name_(name), 
      control_action_(Control_t::Zero())
  {
  }

  // copy constructor
  System(const System &rhs) : 
      name_(rhs.name_),
      control_action_(rhs.getControlAction())
  {
  }

  // copy assignment operator
  System& operator=(const System &rhs)
  {
    if (this != &rhs) {
      const Control_t rhs_control = rhs.getControlAction();
      std::lock_guard<std::mutex> lock(control_mtx_);
      name_ = rhs.name_;
      control_action_ = rhs_control;
    }
    return *this;
  }

  virtual ~System() {}

  virtual void updateControlledDynamics(const State_t &state, 
      const double &t, 
      const Control_t &control, 
      State_t &derivative) = 0;
  
  virtual void updateDynamics(const State_t &state, 
      const double &t, 
      State_t &derivative) = 0;

  std::string getName() const { return name_; }
  Control_t getControlAction() const {
    std::lock_guard<std::mutex> lock(control_mtx_);
    return control_action_;
  }
  void setControlAction(const Control_t &control) {
    std::lock_guard<std::mutex> lock(control_mtx_);
    control_action_ = control;
  }

 protected:
  std::string name_;
  mutable std::mutex control_mtx_;
  Control_t control_action_;
};

// This is a simple model of the quadrotor system
class QuadrotorSystem final : public System<10, 4>
{
 public: 

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = System<10, 4>;

  // constructor
  QuadrotorSystem(const std::string &name,
                  double tau_v_xy = 0.2,
                  double tau_v_z = 0.2) :
      Base(name),
      tau_v_xy_(tau_v_xy),
      tau_v_z_(tau_v_z)
  {
  }

  // copy constructor
  QuadrotorSystem(const QuadrotorSystem &rhs) : 
      Base(rhs),
      tau_v_xy_(rhs.tau_v_xy_),
      tau_v_z_(rhs.tau_v_z_)
  {
  }

  void setVelocityTrackingConstants(double tau_v_xy, double tau_v_z)
  {
    tau_v_xy_ = tau_v_xy;
    tau_v_z_ = tau_v_z;
  }

  double getTauVXY() const { return tau_v_xy_; }
  double getTauVZ() const { return tau_v_z_; }

  // p, v, q
  void updateControlledDynamics(const State_t &state, 
      const double &t, 
      const Control_t &control, 
      State_t &derivative) override
  {
    (void)t;
    const double yaw_rate = control(3);
    const double tau_v_xy = std::max(1e-6, tau_v_xy_);
    const double tau_v_z = std::max(1e-6, tau_v_z_);
    derivative(0) = state(3); // vx
    derivative(1) = state(4); // vy
    derivative(2) = state(5); // vz
    derivative(3) = (control(0) - state(3)) / tau_v_xy;
    derivative(4) = (control(1) - state(4)) / tau_v_xy;
    derivative(5) = (control(2) - state(5)) / tau_v_z;
    // Use a simple quaternion integration with a yaw-only body rate (0, 0, yaw_rate).
    Eigen::Quaterniond q(state(6), state(7), state(8), state(9));
    q.normalize();
    derivative(6) = -0.5 * q.z() * yaw_rate;
    derivative(7) = 0.5 * q.y() * yaw_rate;
    derivative(8) = -0.5 * q.x() * yaw_rate;
    derivative(9) = 0.5 * q.w() * yaw_rate;
  }

  void updateDynamics(const State_t &state, 
      const double &t, 
      State_t &derivative) override
  {
    updateControlledDynamics(state, t, control_action_, derivative);
  }

 private:
  double tau_v_xy_;
  double tau_v_z_;
};


} // namespace num_sim

#include "acado_mpc/mpc_wrapper.h"
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <iomanip>
#include <memory>
#include <random>

namespace acado_mpc {
extern "C" void acado_evaluatePathConstraints(const real_t*, real_t*);
}

namespace {
using namespace acado_mpc;
using Wrapper = MpcWrapper<double>;
using State = Eigen::Matrix<double, kStateSize, 1>;
using States = Eigen::Matrix<double, kStateSize, kSamples + 1>;
using Inputs = Eigen::Matrix<double, kInputSize, kSamples + 1>;

Wrapper::WorldVelocityFrameProfile frame(const Eigen::Matrix3d& rotation,
                                        const Eigen::Vector3d& velocity) {
  Wrapper::WorldVelocityFrameProfile result;
  for (int k = 0; k <= kSamples; ++k) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) result(3 * i + j, k) = rotation(i, j);
    result.col(k).tail<3>() = velocity;
  }
  return result;
}

std::unique_ptr<Wrapper> configured() {
  ros::Time::init();
  auto wrapper = std::make_unique<Wrapper>();
  Eigen::Matrix<double, kCostSize, kCostSize> Q =
      Eigen::Matrix<double, kCostSize, kCostSize>::Identity();
  Eigen::Matrix<double, kInputSize, kInputSize> R =
      Eigen::Matrix<double, kInputSize, kInputSize>::Identity();
  EXPECT_TRUE(wrapper->setCosts(Q, R));
  EXPECT_TRUE(wrapper->setLimits(5., 3., 3.14, 10., 4., 0.));
  EXPECT_TRUE(wrapper->setVelocityTrackingTimeConstants(.2, .2));
  EXPECT_TRUE(wrapper->setObstacles({}, {}, 5., 5., false));
  EXPECT_TRUE(wrapper->setFieldHocbf(Wrapper::FieldHocbfProfile::Zero(), false));
  EXPECT_TRUE(wrapper->setNonInertialData(Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  return wrapper;
}

TEST(MpcPhysicalLimits, GeneratedRowsMatchIndependentThreeDimensionalMapping) {
  std::mt19937 random(20261021);
  std::uniform_real_distribution<double> draw(-2., 2.);
  for (int sample = 0; sample < 100; ++sample) {
    Eigen::Matrix<double, kStateSize + kInputSize + kOdSize, 1> in;
    in.setZero();
    auto x = in.head<kStateSize>();
    auto u = in.segment<kInputSize>(kStateSize);
    auto od = in.tail<kOdSize>();
    for (int i = 0; i < 7; ++i) { x(i) = draw(random); u(i) = draw(random); }
    Eigen::Vector3d omega(draw(random), draw(random), draw(random));
    Eigen::Vector3d vg(draw(random), draw(random), draw(random));
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(draw(random), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(draw(random), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(draw(random), Eigen::Vector3d::UnitX())).toRotationMatrix();
    od.segment<3>(kOdOmegaNonOffset) = omega;
    od.segment<3>(kOdBetaNonOffset).setConstant(7.);
    od.segment<3>(kOdCarAccOffset).setConstant(9.);
    od(kOdInvTauVxyIndex) = 5.; od(kOdInvTauVzIndex) = 4.;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) od(kOdWorldRotationOffset + 3 * i + j) = rotation(i, j);
    od.segment<3>(kOdWorldFrameVelocityOffset) = vg;
    // Finite inactive obstacle geometry keeps generated derivative evaluation defined.
    for (int obstacle = 0; obstacle < 3; ++obstacle) {
      const int base = obstacle * kOdObstacleStride;
      od.segment<3>(base + 3).setOnes();
      od(base + 6) = od(base + 10) = od(base + 14) = 1.;
    }
    Eigen::Matrix<double, kPathConstraintSize * (1 + kStateSize + kInputSize), 1> out;
    acado_evaluatePathConstraints(in.data(), out.data());
    const Eigen::Vector3d physical = rotation *
        Eigen::Vector3d(5., 5., 4.).cwiseProduct(u.head<3>() - x.segment<3>(3));
    const Eigen::Vector3d world = vg + rotation * (u.head<3>() + omega.cross(x.head<3>()));
    EXPECT_NEAR(out(0), physical.head<2>().squaredNorm(), 1e-9);
    EXPECT_NEAR(out(1), physical.z(), 1e-11);
    EXPECT_NEAR(out(2), u(3) + omega.z(), 1e-11);
    EXPECT_NEAR(out(kWorldVelocityConstraintOffset), world.head<2>().squaredNorm(), 1e-10);
    EXPECT_NEAR(out(kWorldVelocityConstraintOffset + 1), world.z(), 1e-11);
  }
}

TEST(MpcPhysicalLimits, RejectsInvalidWorldFrames) {
  auto wrapper = configured();
  auto profile = frame(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  EXPECT_TRUE(wrapper->setWorldVelocityFrameProfile(profile));
  profile(0, 7) = 2.;
  EXPECT_FALSE(wrapper->setWorldVelocityFrameProfile(profile));
  profile(0, 7) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(wrapper->setWorldVelocityFrameProfile(profile));
}

TEST(MpcPhysicalLimits, ColdSolveHonorsTranslatedWorldEnvelopeAndPhysicalAcceleration) {
  auto wrapper = configured();
  const Eigen::Vector3d vg(4., 0., 0.);
  EXPECT_TRUE(wrapper->setWorldVelocityFrameProfile(frame(Eigen::Matrix3d::Identity(), vg)));
  // Platform acceleration must not be subtracted from the physical actuator bound.
  EXPECT_TRUE(wrapper->setNonInertialData(Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(2., 0., 0.)));
  State initial = State::Zero(); initial(2) = 1.;
  States refs = initial.replicate(1, kSamples + 1);
  refs.row(0).setConstant(50.);
  Inputs desired = Inputs::Zero(); desired.row(0).setConstant(5.);
  EXPECT_TRUE(wrapper->setTrajectory(refs, desired));
  ASSERT_TRUE(wrapper->solve(initial));
  Eigen::Matrix<double, kInputSize, kSamples> controls;
  States predicted;
  wrapper->getInputs(controls); wrapper->getStates(predicted);
  for (int k = 0; k < kSamples; ++k) {
    const Eigen::Vector3d world = vg + controls.col(k).head<3>();
    EXPECT_LE(world.head<2>().norm(), 5. + 1e-6) << "stage=" << k;
    EXPECT_LE(std::abs(world.z()), 3. + 1e-6);
    const Eigen::Vector3d acceleration = 5. * (controls.col(k).head<3>() - predicted.col(k).segment<3>(3));
    EXPECT_LE(acceleration.head<2>().norm(), 10. + 1e-6);
    EXPECT_LE(std::abs(acceleration.z()), 4. + 1e-6);
  }
}

TEST(MpcPhysicalLimits, StageFramesAndSeparateInstancesDoNotLeak) {
  auto translated = configured();
  auto inertial = configured();
  const Eigen::Vector3d omega(0., 0., .15);
  EXPECT_TRUE(translated->setNonInertialData(omega,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  auto profile = frame(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., 0., 0.));
  for (int k = 0; k <= kSamples; ++k) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.03*k, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) profile(3*i+j, k) = rotation(i,j);
    profile(9, k) = 4. - .02*k;
  }
  EXPECT_TRUE(translated->setWorldVelocityFrameProfile(profile));
  State initial = State::Zero(); initial(2) = 1.;
  States refs = initial.replicate(1, kSamples + 1); refs.row(0).setConstant(30.);
  Inputs desired = Inputs::Zero(); desired.row(0).setConstant(5.);
  ASSERT_TRUE(translated->setTrajectory(refs, desired));
  ASSERT_TRUE(inertial->setTrajectory(refs, desired));
  const bool translated_ok = translated->solve(initial);
  ASSERT_TRUE(inertial->solve(initial));
  Eigen::Matrix<double, kInputSize, kSamples> controls;
  States predicted;
  translated->getInputs(controls);
  translated->getStates(predicted);
  if (!translated_ok) {
    for (int k = 0; k < kSamples; ++k) {
      Eigen::Matrix3d rotation;
      for (int i=0; i<3; ++i) for(int j=0; j<3; ++j) rotation(i,j)=profile(3*i+j,k);
      const Eigen::Vector3d world = profile.col(k).tail<3>() + rotation *
          (controls.col(k).head<3>() + omega.cross(predicted.col(k).head<3>()));
      if (world.head<2>().norm() > 5.+1e-7) ADD_FAILURE() << "stage=" << k << " world speed=" << std::setprecision(17) << world.head<2>().norm();
    }
  }
  ASSERT_TRUE(translated_ok);
  for (int k = 0; k < kSamples; ++k) {
    Eigen::Matrix3d rotation;
    for (int i=0; i<3; ++i) for(int j=0; j<3; ++j) rotation(i,j)=profile(3*i+j,k);
    const Eigen::Vector3d world = profile.col(k).tail<3>() + rotation *
        (controls.col(k).head<3>() + omega.cross(predicted.col(k).head<3>()));
    EXPECT_LE(world.head<2>().norm(), 5. + 1e-6) << "stage=" << k;
  }
  const double translated_first = controls(0,0);
  inertial->getInputs(controls);
  EXPECT_GT(controls(0,0), translated_first + .1);
}

TEST(MpcPhysicalLimits, InfeasibleWorldEnvelopeDoesNotReplaceAcceptedInputAnchor) {
  auto wrapper = configured();
  const auto previous = wrapper->getPreviousInput();
  EXPECT_TRUE(wrapper->setWorldVelocityFrameProfile(frame(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(20., 0., 0.))));
  State initial = State::Zero(); initial(2) = 1.;
  EXPECT_FALSE(wrapper->solve(initial));
  EXPECT_TRUE(wrapper->getPreviousInput().isApprox(previous));
}

TEST(MpcPhysicalLimits, WorldFeasibleRelativeVelocityMayExceedFormerComponentLimits) {
  auto wrapper = configured();
  const Eigen::Vector3d vg(12., -11., 8.);
  ASSERT_TRUE(wrapper->setWorldVelocityFrameProfile(frame(Eigen::Matrix3d::Identity(), vg)));
  State initial = State::Zero(); initial(2) = 1.; initial.segment<3>(3) = -vg;
  States refs = initial.replicate(1, kSamples + 1);
  Inputs desired = Inputs::Zero();
  for (int k = 0; k <= kSamples; ++k) {
    refs.col(k).head<3>() = initial.head<3>() - vg * (.1*k);
    desired.col(k).head<3>() = -vg;
  }
  ASSERT_TRUE(wrapper->setTrajectory(refs, desired));
  ASSERT_TRUE(wrapper->solve(initial));
  Eigen::Matrix<double, kInputSize, kSamples> controls;
  wrapper->getInputs(controls);
  EXPECT_LT(controls(0,0), -5.);
  EXPECT_GT(controls(1,0), 5.);
  EXPECT_LT(controls(2,0), -3.);
  for (int k=0; k<kSamples; ++k) {
    const Eigen::Vector3d world = vg + controls.col(k).head<3>();
    EXPECT_LE(world.head<2>().norm(), 5.+1e-6);
    EXPECT_LE(std::abs(world.z()), 3.+1e-6);
  }
}

TEST(MpcPhysicalLimits, RelativeYawMayCancelPlatformRotationBeyondFormerYawBound) {
  auto wrapper = configured();
  const Eigen::Vector3d omega(0., 0., 4.);
  ASSERT_TRUE(wrapper->setNonInertialData(omega,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  auto frames = frame(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  for (int k=0; k<=kSamples; ++k) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.4*k, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for (int i=0; i<3; ++i) for(int j=0; j<3; ++j) frames(3*i+j,k)=rotation(i,j);
  }
  ASSERT_TRUE(wrapper->setWorldVelocityFrameProfile(frames));
  State initial=State::Zero(); initial(2)=1.;
  States refs=initial.replicate(1,kSamples+1); Inputs desired=Inputs::Zero();
  for(int k=0;k<=kSamples;++k) { refs(6,k)=-.4*k; desired(3,k)=-4.; }
  ASSERT_TRUE(wrapper->setTrajectory(refs,desired));
  ASSERT_TRUE(wrapper->solve(initial));
  Eigen::Matrix<double,kInputSize,kSamples> controls;wrapper->getInputs(controls);
  EXPECT_LT(controls(3,0),-3.14);
  for(int k=0;k<kSamples;++k) EXPECT_LE(std::abs(controls(3,k)+4.),3.14+1e-6);
}

TEST(MpcPhysicalLimits, WorldYawUpperAndLowerBoundsIncludePlatformRate) {
  for (double sign : {-1.,1.}) {
    auto wrapper=configured();
    ASSERT_TRUE(wrapper->setLimits(5.,3.,1.,10.,4.,0.));
    ASSERT_TRUE(wrapper->setNonInertialData(Eigen::Vector3d(0.,0.,sign*.8),
        Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()));
    State initial=State::Zero(); initial(2)=1.;
    States refs=initial.replicate(1,kSamples+1);Inputs desired=Inputs::Zero();
    for(int k=0;k<=kSamples;++k) {refs(6,k)=sign*.3*k;desired(3,k)=sign*3.;}
    ASSERT_TRUE(wrapper->setTrajectory(refs,desired));
    ASSERT_TRUE(wrapper->solve(initial));
    Eigen::Matrix<double,kInputSize,kSamples> controls;wrapper->getInputs(controls);
    for(int k=0;k<kSamples;++k) EXPECT_LE(std::abs(controls(3,k)+sign*.8),1.+1e-6);
    EXPECT_GT(sign*(controls(3,0)+sign*.8),.99);
  }
}

TEST(MpcPhysicalLimits, HorizontalAccelerationIsACircleRatherThanAnAxisAlignedBox) {
  auto wrapper=configured();
  State initial=State::Zero();initial(2)=1.;
  States refs=initial.replicate(1,kSamples+1);refs.row(0).setConstant(50.);refs.row(1).setConstant(50.);
  Inputs desired=Inputs::Zero();desired.row(0).setConstant(5.);desired.row(1).setConstant(5.);
  ASSERT_TRUE(wrapper->setTrajectory(refs,desired));
  ASSERT_TRUE(wrapper->solve(initial));
  Eigen::Matrix<double,kInputSize,kSamples> controls;States predicted;
  wrapper->getInputs(controls);wrapper->getStates(predicted);
  for(int k=0;k<kSamples;++k) {
    const Eigen::Vector3d acceleration=5.*(controls.col(k).head<3>()-predicted.col(k).segment<3>(3));
    EXPECT_LE(acceleration.head<2>().norm(),10.+1e-6);
    EXPECT_LE(std::abs(acceleration.z()),4.+1e-6);
  }
  EXPECT_GT(controls(0,0),1.);
  EXPECT_GT(controls(1,0),1.);
  EXPECT_LE(controls.col(0).head<2>().norm(),2.+1e-6);
}

TEST(MpcPhysicalLimits, TiltedFrameColdSolveChecksWorldAccelerationAndVelocity) {
  auto wrapper=configured();
  const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitY())).toRotationMatrix();
  ASSERT_TRUE(wrapper->setWorldVelocityFrameProfile(frame(rotation,Eigen::Vector3d::Zero())));
  State initial=State::Zero();initial(2)=1.;
  States refs=initial.replicate(1,kSamples+1);
  Inputs desired=Inputs::Zero();
  const Eigen::Vector3d acceleration_non=rotation.transpose()*Eigen::Vector3d(1.,2.,.5);
  // Seed a feasible exact first-order rollout, so the frame test is distinct
  // from the deliberately infeasible cold nominal used by the stress test.
  for(int k=0;k<=kSamples;++k) {
    desired.col(k).head<3>()=refs.col(k).segment<3>(3)+.2*acceleration_non;
    if(k<kSamples) {
      const Eigen::Vector3d v=refs.col(k).segment<3>(3);
      const Eigen::Vector3d u=desired.col(k).head<3>();
      refs.col(k+1).segment<3>(3)=u+(v-u)*std::exp(-.5);
      refs.col(k+1).head<3>()=refs.col(k).head<3>()+.1*u+.2*(v-u)*(1.-std::exp(-.5));
    }
  }
  ASSERT_TRUE(wrapper->setTrajectory(refs,desired));
  ASSERT_TRUE(wrapper->solve(initial));
  Eigen::Matrix<double,kInputSize,kSamples> controls;States predicted;
  wrapper->getInputs(controls);wrapper->getStates(predicted);
  for(int k=0;k<kSamples;++k) {
    const Eigen::Vector3d acceleration=rotation*(5.*(controls.col(k).head<3>()-predicted.col(k).segment<3>(3)));
    const Eigen::Vector3d velocity=rotation*controls.col(k).head<3>();
    EXPECT_LE(acceleration.head<2>().norm(),10.+1e-6);
    EXPECT_LE(std::abs(acceleration.z()),4.+1e-6);
    EXPECT_LE(velocity.head<2>().norm(),5.+1e-6);
    EXPECT_LE(std::abs(velocity.z()),3.+1e-6);
  }
}

TEST(MpcPhysicalLimits, TiltedFrameStressNeverAcceptsUnconvergedPhysicalEnvelope) {
  auto wrapper=configured();
  const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitY())).toRotationMatrix();
  ASSERT_TRUE(wrapper->setWorldVelocityFrameProfile(frame(rotation,Eigen::Vector3d::Zero())));
  State initial=State::Zero();initial(2)=1.;
  States refs=initial.replicate(1,kSamples+1);refs.topRows<3>().setConstant(30.);
  Inputs desired=Inputs::Zero();desired.topRows<3>().setConstant(5.);
  ASSERT_TRUE(wrapper->setTrajectory(refs,desired));
  const auto anchor=wrapper->getPreviousInput();
  const bool accepted=wrapper->solve(initial);
  Eigen::Matrix<double,kInputSize,kSamples> controls;States predicted;
  wrapper->getInputs(controls);wrapper->getStates(predicted);
  double residual=0.;
  for(int k=0;k<kSamples;++k) {
    const Eigen::Vector3d a=rotation*(5.*(controls.col(k).head<3>()-predicted.col(k).segment<3>(3)));
    const Eigen::Vector3d v=rotation*controls.col(k).head<3>();
    residual=std::max({residual,a.head<2>().norm()-10.,std::abs(a.z())-4.,
                       v.head<2>().norm()-5.,std::abs(v.z())-3.});
  }
  if(accepted) EXPECT_LE(residual,1e-7);
  else {
    EXPECT_TRUE(wrapper->getPreviousInput().isApprox(anchor));
    EXPECT_GT(residual,1e-7);
  }
  std::cout << "tilted_stress_accepted=" << accepted
            << " max_physical_residual=" << std::setprecision(17) << residual << '\n';
}
}  // namespace

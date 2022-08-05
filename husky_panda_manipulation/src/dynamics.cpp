/*!
 * @file     dynamics_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "husky_panda_manipulation/dynamics.h"
#include <ros/package.h>

namespace manipulation {

PandaRaisimDynamics::PandaRaisimDynamics(const DynamicsParams& params)
    : params_(params) {
  initialize_world(params_.robot_description, params_.object_description);
  initialize_pd();
  set_collision();
  
  t_ = 0.0;
  ee_force_applied_ = false;
};

void PandaRaisimDynamics::initialize_world(
    const std::string& robot_description,
    const std::string& object_description) {
  dt_ = params_.dt;
  sim_.setTimeStep(params_.dt);
  sim_.setERP(0., 0.);

  gravity_.e() << 0.0, 0.0, -9.81;
  sim_.setGravity(gravity_);

  // set friction properties
  sim_.setMaterialPairProp("steel", "steel", 0.01, 0.15, 0.001);

  robot_description_ = robot_description;
  
  husky_panda_ = sim_.addArticulatedSystem(robot_description_, "/");

  tau_ext_ = Eigen::VectorXd::Zero(husky_panda_->getDOF());
  J_contact_.setZero(3, husky_panda_->getDOF());

  /// create raisim objects
  object_description_ = object_description;
  object_ = sim_.addArticulatedSystem(object_description_, "/");

  // odometry
  odometry_.init();

  // robot dof
  robot_dof_ = BASE_ARM_GRIPPER_DIM;
  input_dimension_ = TORQUE_DIMENSION;
  state_dimension_ = STATE_DIMENSION;
  // q_dot =! v (because floating base has coordinate of (translation (x, y, z) and orientation (w, x, y, z)))
  // Floating base coordinate : TORQUE_DIMENSION + FLOATING_BASE_DIMENSION + 1 = 20
  coordinate_dimension_ = husky_panda_->getGeneralizedCoordinateDim();
  // Floating base velocity : TORQUE_DIMENSION + FLOATING_BASE_DIMENSION = 19
  velocity_dimension_ = husky_panda_->getDOF(); 
  x_ = mppi::observation_t::Zero(STATE_DIMENSION);

  reset(params_.initial_state, t_);
}

void PandaRaisimDynamics::initialize_pd() {
  /// husky_panda
  cmd_.setZero(coordinate_dimension_);
  cmdv_.setZero(velocity_dimension_);
  joint_p_.setZero(coordinate_dimension_);
  joint_v_.setZero(velocity_dimension_);
  joint_p_gain_.setZero(velocity_dimension_);
  joint_d_gain_.setZero(velocity_dimension_);
  joint_p_desired_.setZero(coordinate_dimension_);
  joint_v_desired_.setZero(velocity_dimension_);

  // clang-format off
  joint_p_gain_.segment<BASE_JOINT_DIMENSION>(FLOATING_BASE_DIMENSION) = params_.gains.base_gains.Kp;
  joint_d_gain_.segment<BASE_JOINT_DIMENSION>(FLOATING_BASE_DIMENSION) = params_.gains.base_gains.Kd;
  joint_p_gain_.segment(BASE_JOINT_DIMENSION + FLOATING_BASE_DIMENSION, ARM_DIMENSION) = params_.gains.arm_gains.Kp;
  joint_d_gain_.segment(BASE_JOINT_DIMENSION + FLOATING_BASE_DIMENSION, ARM_DIMENSION) = params_.gains.arm_gains.Kd;
  joint_p_gain_.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kp;
  joint_d_gain_.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kd;
  // clang-format on

  husky_panda_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  husky_panda_->setPdGains(joint_p_gain_, joint_d_gain_);
  husky_panda_->setGeneralizedForce(Eigen::VectorXd::Zero(husky_panda_->getDOF()));

  object_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  object_->setPdGains(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
  object_->setGeneralizedForce({0.0});

  // odometry
  int velocity_rolling_window_size = 2;
  odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

  // Regardless of how we got the separation and radius, use them
  // to set the odometry parameters
  wheel_separation_multiplier_ = 1.875 ;
  left_wheel_radius_multiplier_ = 1.0;
  right_wheel_radius_multiplier_ = 1.0;
  wheel_separation_ = 0.571;
  wheel_radius_ = 0.1651;

  const double ws  = wheel_separation_multiplier_   * wheel_separation_;
  const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
  const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
  odometry_.setWheelParams(ws, lwr, rwr);
}

void PandaRaisimDynamics::set_collision() {
  std::vector<int> pandaBodyIdxs;
  for (const auto& bodyName : husky_panda_->getBodyNames())
    pandaBodyIdxs.push_back(husky_panda_->getBodyIdx(bodyName));
  for (const auto body_idx1 : pandaBodyIdxs)
    for (const auto body_idx2 : pandaBodyIdxs)
      husky_panda_->ignoreCollisionBetween(body_idx1, body_idx2);
}

void PandaRaisimDynamics::set_control(const mppi::input_t& u) {
  // keep the gripper in the current position
  cmd_.tail<HuskyPandaDim::GRIPPER_DIMENSION>()
      << x_.head<BASE_ARM_GRIPPER_DIM>().tail<GRIPPER_DIMENSION>();

  // cmdv_(0) = u(0) * std::cos(x_(2)) - u(1) * std::sin(x_(2));
  // cmdv_(1) = u(0) * std::sin(x_(2)) + u(1) * std::cos(x_(2));
  // cmdv_(2) = u(2);

  double left_pos  = 0.0;
  double right_pos = 0.0;
  for (size_t i = 0; i < 2; ++i)
  {
    const double lp = u(i); // front_left_wheel, front_right_wheel
    const double rp = u(i + 2); // rear_left_wheel, rear_right_wheel
    if (std::isnan(lp) || std::isnan(rp))
      return;

    left_pos  += lp;
    right_pos += rp;
  }
  left_pos  /= 2.0;
  right_pos /= 2.0;

  // Estimate linear and angular velocity using joint information
  odometry_.update(left_pos, right_pos, dt_);

  cmdv_.segment(FLOATING_BASE_DIMENSION, BASE_JOINT_DIMENSION) = u.head(BASE_JOINT_DIMENSION);

  cmdv_.segment<ARM_DIMENSION>(BASE_JOINT_DIMENSION + FLOATING_BASE_DIMENSION) = u.segment<ARM_DIMENSION>(BASE_JOINT_DIMENSION);

  cmdv_.tail<HuskyPandaDim::GRIPPER_DIMENSION>().setZero();
  husky_panda_->setPdTarget(cmd_, cmdv_);
  husky_panda_->setGeneralizedForce(husky_panda_->getNonlinearities(gravity_));

  // gravity compensated object
  object_->setGeneralizedForce(object_->getNonlinearities(gravity_));
}

void PandaRaisimDynamics::advance() {
  // get contact state
  double in_contact = -1;
  for (const auto& contact : object_->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      in_contact = 1;
      break;
    }
  }

  // get external torque
  get_external_torque(tau_ext_);

  // step simulation
  sim_.integrate();
  t_ += sim_.getTimeStep();

  husky_panda_->getState(joint_p_, joint_v_);
  object_->getState(object_p_, object_v_);

  // mppi::observation_t base_p_(3), base_v_(3);
  // base_p_(0) = odometry_.getX();
  // base_p_(1) = odometry_.getY();
  // base_p_(2) = odometry_.getHeading();
  // base_v_(0) = odometry_.getLinear();
  // base_v_(1) = 0.0;
  // base_v_(2) = odometry_.getAngular();

  raisim::Vec<3> base_pos, base_vel;
  raisim::Mat<3, 3> base_ori;
  husky_panda_->getBasePosition(base_pos);
  husky_panda_->getBaseOrientation(base_ori);
  base_vel(0) = joint_v_.head(BASE_ARM_GRIPPER_DIM + FLOATING_BASE_DIMENSION)(0);
  base_vel(1) = joint_v_.head(BASE_ARM_GRIPPER_DIM + FLOATING_BASE_DIMENSION)(1);
  base_vel(2) = joint_v_.head(BASE_ARM_GRIPPER_DIM + FLOATING_BASE_DIMENSION)(5);

  x_.head(BASE_DIMENSION) = base_pos.e();// base state (x,y,theta) from odometry
  x_.segment<ARM_DIMENSION>(BASE_DIMENSION) = joint_p_.segment<ARM_DIMENSION>(BASE_DIMENSION + FLOATING_BASE_DIMENSION + 1);
  x_.segment<GRIPPER_DIMENSION>(BASE_DIMENSION + ARM_DIMENSION) = joint_p_.segment<GRIPPER_DIMENSION>(BASE_DIMENSION + ARM_DIMENSION + FLOATING_BASE_DIMENSION + 1);
  x_.segment<BASE_DIMENSION>(BASE_ARM_GRIPPER_DIM) = base_vel.e();// base state (x_dot,y_dot,theta_dot) from odometry
  x_.segment<ARM_DIMENSION>(BASE_ARM_GRIPPER_DIM + BASE_DIMENSION) = joint_v_.segment<ARM_DIMENSION>(BASE_ARM_GRIPPER_DIM + BASE_DIMENSION + FLOATING_BASE_DIMENSION);
  x_.segment<GRIPPER_DIMENSION>(BASE_ARM_GRIPPER_DIM + BASE_DIMENSION + ARM_DIMENSION) = joint_v_.segment<GRIPPER_DIMENSION>(BASE_ARM_GRIPPER_DIM + BASE_DIMENSION + ARM_DIMENSION + FLOATING_BASE_DIMENSION);
  x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0) = object_p_(0);
  x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(1) = object_v_(0);
  x_(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) = in_contact;
  x_.tail<TORQUE_DIMENSION>() = tau_ext_;
}

mppi::observation_t PandaRaisimDynamics::step(const mppi::input_t& u,
                                              const double dt) {
  set_control(u);
  advance();
  return x_;
}

void PandaRaisimDynamics::reset(const mppi::observation_t& x, const double t) {
  // internal eigen state
  t_ = t;
  x_ = x;

  Eigen::VectorXd floating_based_x(coordinate_dimension_);
  floating_based_x.setZero();
  floating_based_x(0) = x_(0); // x
  floating_based_x(1) = x_(1); // y
  floating_based_x(3) = std::cos(0.5 * x_(2)); // w
  floating_based_x(6) = std::sin(0.5 * x_(2)); // z
  floating_based_x.segment(FLOATING_BASE_DIMENSION + 1, ARM_GRIPPER_DIM) = x_.segment(BASE_DIMENSION, ARM_GRIPPER_DIM);

  // reset arm
  husky_panda_->setState(floating_based_x,
                   x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM));
  object_->setState(x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM),
                    x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION));
}

mppi::input_t PandaRaisimDynamics::get_zero_input(
    const mppi::observation_t& x) {
  return mppi::input_t::Zero(get_input_dimension());
}

void PandaRaisimDynamics::get_end_effector_pose(
    Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
  size_t frame_id = husky_panda_->getFrameIdxByName("panda_grasp_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  husky_panda_->getFramePosition(frame_id, pos);
  husky_panda_->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

void PandaRaisimDynamics::get_handle_pose(Eigen::Vector3d& position,
                                          Eigen::Quaterniond& orientation) {
  size_t frame_id = object_->getFrameIdxByName(params_.object_handle_joint);
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  object_->getFramePosition(frame_id, pos);
  object_->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

std::vector<force_t> PandaRaisimDynamics::get_contact_forces() {
  std::vector<force_t> forces;
  for (const auto contact : husky_panda_->getContacts()) {
    if (contact.skip())
      continue;  /// if the contact is internal, one contact point is set to
                 /// 'skip'
    if (contact.isSelfCollision()) continue;
    force_t force;
    force.force = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() /
                  sim_.getTimeStep();
    force.position = contact.getPosition().e();
    forces.push_back(force);
  }
  return forces;
}

void PandaRaisimDynamics::get_external_torque(Eigen::VectorXd& tau) {
  tau.setZero((int)husky_panda_->getDOF());
  for (const auto contact : husky_panda_->getContacts()) {
    J_contact_.setZero();
    if (!contact.skip() && !contact.isSelfCollision()) {
      husky_panda_->getDenseJacobian(contact.getlocalBodyIndex(),
                               contact.getPosition(), J_contact_);

      // clang-format off
      // the base jacobian references the world frame, need to rotate
      // to the base reference frame
      J_contact_.topLeftCorner<3, 3>() << std::cos(x_(2)), std::sin(x_(2)), 0,
                                          -std::sin(x_(2)), std::cos(x_(2)), 0,
                                          0, 0, 1;
      // transform contact to force --> to force in world frame --> to reaction torques
      tau -= J_contact_.transpose() * contact.getContactFrame().e().transpose() * contact.getImpulse().e() / sim_.getTimeStep();
      // clang-format on
    }
  }

  if (ee_force_applied_){
    J_contact_.setZero();
    husky_panda_->getDenseFrameJacobian("panda_grasp_joint", J_contact_);

    // clang-format off
    J_contact_.topLeftCorner<3, 3>() << std::cos(x_(2)), -std::sin(x_(2)), 0,
                                        std::sin(x_(2)), std::cos(x_(2)), 0,
                                        0, 0, 1;
    // clang-format on
    tau += J_contact_.transpose() * husky_panda_->getExternalForce()[0].e();

  }
}

void PandaRaisimDynamics::get_external_wrench(Eigen::VectorXd& wrench) {
  wrench.setZero(6);
  size_t frame_id = husky_panda_->getFrameIdxByName("panda_grasp_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  husky_panda_->getFramePosition(frame_id, pos);
  husky_panda_->getFrameOrientation(frame_id, rot);

  for (const auto contact : husky_panda_->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      // ee_frame <-- world_frame <-- force <-- impulse
      Eigen::Vector3d force_ee_frame =
          -rot.e().transpose() * contact.getContactFrame().e().transpose() *
          contact.getImpulse().e() / sim_.getTimeStep();
      Eigen::Vector3d relative_position =
          rot.e().transpose() * (contact.getPosition().e() - pos.e());
      wrench.head<3>() += force_ee_frame;
      wrench.tail<3>() = relative_position.cross(force_ee_frame);
    }
  }
  if (ee_force_applied_) {
    wrench.head<3>() += husky_panda_->getExternalForce()[0].e();
  }
}

void PandaRaisimDynamics::get_reference_link_pose(Eigen::Vector3d& position,
                             Eigen::Quaterniond& orientation){
  size_t frame_id = husky_panda_->getFrameIdxByName("reference_link_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  husky_panda_->getFramePosition(frame_id, pos);
  husky_panda_->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

void PandaRaisimDynamics::get_ee_jacobian(Eigen::MatrixXd& J){
  J.setZero(6, (int)husky_panda_->getDOF());
  Eigen::MatrixXd J_linear;
  J_linear.setZero(3, 12);
  Eigen::MatrixXd J_angular;
  J_angular.setZero(3, 12);

  husky_panda_->getDenseFrameJacobian("panda_grasp_joint", J_linear);
  husky_panda_->getDenseFrameRotationalJacobian("panda_grasp_joint", J_angular);
  J.topRows(3) = J_linear;
  J.bottomRows(3) = J_angular;
  // clang-format off
  J.topLeftCorner<3, 3>() << std::cos(x_(2)), -std::sin(x_(2)), 0,
                             std::sin(x_(2)), std::cos(x_(2)), 0,
                             0, 0, 1;
  // clang-format on
}

void PandaRaisimDynamics::set_external_ee_force(const Eigen::Vector3d& f) {
  ee_force_applied_ = (f.norm() > 1e-4);
  auto& frame = husky_panda_->getFrameByName("panda_grasp_joint");
  husky_panda_->setExternalForce(frame.parentId, raisim::ArticulatedSystem::Frame::WORLD_FRAME, f, raisim::ArticulatedSystem::Frame::BODY_FRAME, raisim::Vec<3>());
}

double PandaRaisimDynamics::get_object_displacement() const {
  return x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0);
}

void PandaRaisimDynamics::fix_object() {
  object_->getState(object_p_, object_v_);
  std::vector<raisim::Vec<2>> object_limits;
  raisim::Vec<2> limit;
  limit[0] = object_p_[0] - 0.001;
  limit[1] = object_p_[0] + 0.001;
  object_limits.push_back(limit);
  object_->setJointLimits(object_limits);
}

void PandaRaisimDynamics::release_object() {
  std::vector<raisim::Vec<2>> object_limits;
  raisim::Vec<2> limit;
  limit[0] = 0.0;
  limit[1] = M_PI_2;
  object_limits.push_back(limit);
  object_->setJointLimits(object_limits);
}

}  // namespace manipulation

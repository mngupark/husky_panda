#include "husky_panda_manipulation/dynamics.h"

using namespace husky_panda_control;
namespace husky_panda_control
{

  HuskyPandaRaisimDynamics::HuskyPandaRaisimDynamics(const std::string &robot_description, const std::string &obstacle_description, bool holonomic)
      : robot_description_(robot_description), obstacle_description_(obstacle_description), holonomic_(holonomic)
  {
    // init model
    x_ = observation_t::Zero(HuskyPandaMobileDim::STATE_DIMENSION);
    initialize_world(robot_description, obstacle_description);
    initialize_pd();
    set_collision();

    t_ = 0.0;
  }

  void HuskyPandaRaisimDynamics::initialize_params()
  {
  }

  void HuskyPandaRaisimDynamics::initialize_world(const std::string &robot_description, const std::string &obstacle_description)
  {
    dt_ = 0.01;
    sim_.setTimeStep(0.01);
    sim_.setERP(0., 0.);

    gravity_.e() << 0.0, 0.0, -9.81;
    sim_.setGravity(gravity_);

    // set friction properties
    sim_.setMaterialPairProp("steel", "steel", 0.01, 0.15, 0.001);

    // add ground
    auto ground = sim_.addGround(0.0, "gnd");

    robot_description_ = robot_description;
    husky_panda_ = sim_.addArticulatedSystem(robot_description_);
    husky_panda_->setName("Husky & Panda");

    /// create raisim obstacle
    obstacle_description_ = obstacle_description;
    obstacle_ = sim_.addArticulatedSystem(obstacle_description);
    obstacle_->setName("Obstacle");

    // create reference object
    ref_ = sim_.addSphere(0.05, 0.05);
    ref_->setName("Reference");
    ref_->setAppearance("Yellow");

    // robot dof
    robot_dof_ = VIRTUAL_BASE_ARM_GRIPPER_DIMENSION;
    state_dimension_ = STATE_DIMENSION;
    x_ = husky_panda_control::observation_t::Zero(STATE_DIMENSION);
    if (holonomic_)
    {
      // prismatic x 2, revolute x 1
      // sim_state_dimension_ = state_dimension_;
      // input_dimension_ = INPUT_DIMENSION + 1; // xdot, ydot, yawdot, qdot_arm, qdot_gripper
      // sim_input_dimension_ = input_dimension_;
      //
      // sim_x_ = husky_panda_control::observation_t::Zero(STATE_DIMENSION);
      sim_state_dimension_ = SIM_STATE_DIMENSION;
      input_dimension_ = INPUT_DIMENSION + 1; // xdot, ydot, yawdot, qdot_arm, qdot_gripper
      sim_input_dimension_ = input_dimension_;
      
      sim_x_ = husky_panda_control::observation_t::Zero(SIM_STATE_DIMENSION);

      wheel_radius_multiplier_ = 1.0;
      wheel_separation_multiplier_ = 1.0;

      wheel_separation_x_ = 0.638;
      wheel_separation_y_ = 0.551;

      wheels_k_ = (wheel_separation_x_ + wheel_separation_y_) / 2.0;

      wheel_radius_ = 0.0759 * wheel_radius_multiplier_;

      Eigen::VectorXd gc(husky_panda_->getGeneralizedCoordinateDim()), gv(husky_panda_->getDOF());
      gc.setZero();
      gv.setZero();
      gc.segment<GENERALIZED_COORDINATE + BASE_JOINT_ARM_GRIPPER_DIMENSION>(0) << 
      0, 0, 0.3, 1, 0, 0, 0, 0, 0, 0, 0, 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.04, 0.04;

      husky_panda_->setGeneralizedCoordinate(gc);
      husky_panda_->setGeneralizedVelocity(gv);
    }
    else
    {
      sim_state_dimension_ = SIM_STATE_DIMENSION;
      input_dimension_ = INPUT_DIMENSION;
      sim_input_dimension_ = SIM_INPUT_DIMENSION;
      
      sim_x_ = husky_panda_control::observation_t::Zero(SIM_STATE_DIMENSION);

      wheel_radius_multiplier_ = 1.0;
      wheel_separation_multiplier_ = 1.875;

      wheel_radius_ = 0.1651 * wheel_radius_multiplier_;
      wheel_separation_ = 0.5708 * wheel_separation_multiplier_;
      constrained_matrix_.setZero(BASE_JOINT_DIMENSION, VIRTUAL_BASE_DIMENSION);
      constrained_matrix_.block<BASE_JOINT_DIMENSION, 1>(0, 0) = Eigen::VectorXd::Ones(BASE_JOINT_DIMENSION) / wheel_radius_;
      constrained_matrix_(0, 2) = wheel_separation_ / (2.0 * wheel_radius_);
      constrained_matrix_(1, 2) = -wheel_separation_ / (2.0 * wheel_radius_);
      constrained_matrix_(2, 2) = wheel_separation_ / (2.0 * wheel_radius_);
      constrained_matrix_(3, 2) = -wheel_separation_ / (2.0 * wheel_radius_);

      Eigen::VectorXd gc(husky_panda_->getGeneralizedCoordinateDim()), gv(husky_panda_->getDOF());
      gc.setZero();
      gv.setZero();
      gc.segment<GENERALIZED_COORDINATE + BASE_JOINT_ARM_GRIPPER_DIMENSION>(0) << 
      0, 0, 0.3, 1, 0, 0, 0, 0, 0, 0, 0, 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.04, 0.04;

      husky_panda_->setGeneralizedCoordinate(gc);
      husky_panda_->setGeneralizedVelocity(gv);
    }
    return;
  }

  void HuskyPandaRaisimDynamics::initialize_pd()
  {
    if (holonomic_)
    {
      // prismatic x 2, revolute x 1
      // joint_p_.setZero(STATE_DIMENSION);
      // joint_v_.setZero(INPUT_DIMENSION + 1);
      // cmd_.setZero(STATE_DIMENSION);
      // cmdv_.setZero(INPUT_DIMENSION + 1);
      // joint_p_gain_.setZero(INPUT_DIMENSION + 1);
      // joint_d_gain_.setZero(INPUT_DIMENSION + 1);
      //
      // for (size_t i = 0; i < VIRTUAL_BASE_DIMENSION; i++)
      // {
      //   joint_p_gain_(i) = 0.0;
      //   joint_d_gain_(i) = 1000.0;
      // }
      // for (size_t i = VIRTUAL_BASE_DIMENSION; i < VIRTUAL_BASE_DIMENSION + ARM_DIMENSION; i++)
      // {
      //   joint_p_gain_(i) = 0.0;
      //   joint_d_gain_(i) = 100.0;
      // }
      // for (size_t i = VIRTUAL_BASE_DIMENSION + ARM_DIMENSION; i < VIRTUAL_BASE_ARM_GRIPPER_DIMENSION; i++)
      // {
      //   joint_p_gain_(i) = 100.0;
      //   joint_d_gain_(i) = 50.0;
      // }
      joint_p_.setZero(SIM_STATE_DIMENSION);
      joint_v_.setZero(SIM_INPUT_DIMENSION);
      cmd_.setZero(SIM_STATE_DIMENSION);
      cmdv_.setZero(SIM_INPUT_DIMENSION);
      joint_p_gain_.setZero(SIM_INPUT_DIMENSION);
      joint_d_gain_.setZero(SIM_INPUT_DIMENSION);

      for (size_t i = GENERALIZED_VELOCITY; i < GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION; i++)
      {
        joint_p_gain_(i) = 0.0;
        joint_d_gain_(i) = 100.0;
      }
      for (size_t i = GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION; i < GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION + ARM_DIMENSION; i++)
      {
        joint_p_gain_(i) = 0.0;
        joint_d_gain_(i) = 100.0;
      }
      for (size_t i = GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION + ARM_DIMENSION; i < SIM_INPUT_DIMENSION; i++)
      {
        joint_p_gain_(i) = 100.0;
        joint_d_gain_(i) = 50.0;
      }
    }
    else
    {
      joint_p_.setZero(SIM_STATE_DIMENSION);
      joint_v_.setZero(SIM_INPUT_DIMENSION);
      cmd_.setZero(SIM_STATE_DIMENSION);
      cmdv_.setZero(SIM_INPUT_DIMENSION);
      joint_p_gain_.setZero(SIM_INPUT_DIMENSION);
      joint_d_gain_.setZero(SIM_INPUT_DIMENSION);

      for (size_t i = GENERALIZED_VELOCITY; i < GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION; i++)
      {
        joint_p_gain_(i) = 0.0;
        joint_d_gain_(i) = 100.0;
      }
      for (size_t i = GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION; i < GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION + ARM_DIMENSION; i++)
      {
        joint_p_gain_(i) = 0.0;
        joint_d_gain_(i) = 100.0;
      }
      for (size_t i = GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION + ARM_DIMENSION; i < SIM_INPUT_DIMENSION; i++)
      {
        joint_p_gain_(i) = 100.0;
        joint_d_gain_(i) = 50.0;
      }
    }
    obstacle_p_.setZero(VIRTUAL_BASE_DIMENSION);
    obstacle_v_.setZero(VIRTUAL_BASE_DIMENSION);
    cmd_obstacle_.setZero(VIRTUAL_BASE_DIMENSION);
    ref_p_.setZero();
    cmd_ref_.setZero();

    husky_panda_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    husky_panda_->setPdGains(joint_p_gain_, joint_d_gain_);
    husky_panda_->setGeneralizedForce(Eigen::VectorXd::Zero(husky_panda_->getDOF()));

    // initialize obstacle
    Eigen::Vector3d obstacle_initial_position;
    obstacle_initial_position.setZero();
    obstacle_initial_position(0) = 1.0;
    obstacle_initial_position(2) = 1.0;
    cmd_obstacle_ = obstacle_initial_position;
    obstacle_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    obstacle_->setPdGains(Eigen::VectorXd::Zero(VIRTUAL_BASE_DIMENSION), Eigen::VectorXd::Zero(VIRTUAL_BASE_DIMENSION));
    obstacle_->setGeneralizedForce(obstacle_initial_position);

    // initialize reference object
    Eigen::Vector3d ref_initial_position;
    ref_initial_position.setZero();
    ref_initial_position(0) = 1.0;
    ref_initial_position(2) = 1.0;
    cmd_ref_ = ref_initial_position;
    ref_->setPosition(ref_initial_position);
    return;
  }

  void HuskyPandaRaisimDynamics::set_collision()
  {
    std::vector<int> husky_panda_body_idxs;
    for (const auto &body_name : husky_panda_->getBodyNames())
      husky_panda_body_idxs.push_back(husky_panda_->getBodyIdx(body_name));
    for (const auto body_idx1 : husky_panda_body_idxs)
      for (const auto body_idx2 : husky_panda_body_idxs)
        husky_panda_->ignoreCollisionBetween(body_idx1, body_idx2);
    return;
  }

  void HuskyPandaRaisimDynamics::advance()
  {
    // step simulation
    sim_.integrate();
    t_ += sim_.getTimeStep();

    // get state of each object
    husky_panda_->getState(joint_p_, joint_v_);
    obstacle_->getState(obstacle_p_, obstacle_v_);
    ref_->getPosition(ref_p_);
    
    // set state from state of simulation
    if (holonomic_)
    {
      // prismatic x 2, revolute x 1
      // x_.head(3) = joint_p_.head(3);
      // x_.segment(VIRTUAL_BASE_DIMENSION, ARM_GRIPPER_DIMENSION) = joint_p_.segment(VIRTUAL_BASE_DIMENSION, ARM_GRIPPER_DIMENSION);
      Eigen::Vector3d base_position;
      Eigen::Quaterniond base_orientation;
      get_base_pose(base_position, base_orientation);
      Eigen::Quaterniond temp_quat(joint_p_(3), joint_p_(4), joint_p_(5), joint_p_(6));
      x_(0) = base_position(0);
      x_(1) = base_position(1);
      x_(2) = base_orientation.normalized().toRotationMatrix().eulerAngles(0, 1, 2)(2);
      x_.segment(VIRTUAL_BASE_DIMENSION, ARM_GRIPPER_DIMENSION) = joint_p_.segment(GENERALIZED_COORDINATE + BASE_JOINT_DIMENSION, ARM_GRIPPER_DIMENSION);
    }
    else
    {
      Eigen::Vector3d base_position;
      Eigen::Quaterniond base_orientation;
      get_base_pose(base_position, base_orientation);
      Eigen::Quaterniond temp_quat(joint_p_(3), joint_p_(4), joint_p_(5), joint_p_(6));
      x_(0) = base_position(0);
      x_(1) = base_position(1);
      x_(2) = base_orientation.normalized().toRotationMatrix().eulerAngles(0, 1, 2)(2);
      x_.segment(VIRTUAL_BASE_DIMENSION, ARM_GRIPPER_DIMENSION) = joint_p_.segment(GENERALIZED_COORDINATE + BASE_JOINT_DIMENSION, ARM_GRIPPER_DIMENSION);
    }
    return;
  }
  
  void HuskyPandaRaisimDynamics::set_control(const husky_panda_control::input_t& u)
  {
    // keep the gripper in the current position
    cmd_.tail<GRIPPER_DIMENSION>()
      << x_.tail<GRIPPER_DIMENSION>();

    // set command to simulator
    if (holonomic_)
    {
      // prismatic x 2, revolute x 1
      // cmdv_(0) = u(0) * std::cos(x_(2)) - u(1) * std::sin(x_(2));
      // cmdv_(1) = u(0) * std::sin(x_(2)) + u(1) * std::cos(x_(2));
      // cmdv_(2) = u(2);
      // cmdv_.segment<ARM_DIMENSION>(VIRTUAL_BASE_DIMENSION) = u.segment<ARM_DIMENSION>(VIRTUAL_BASE_DIMENSION);
      //
      // cmdv_.tail<GRIPPER_DIMENSION>().setZero();
      //
      // raisim::VecDyn h = husky_panda_->getNonlinearities(gravity_);
      // husky_panda_->setGeneralizedForce(h);

      double cmd_lin_x = u(0);//u(0) * std::cos(x_(2)) - u(1) * std::sin(x_(2));
      double cmd_lin_y = u(1);//u(0) * std::sin(x_(2)) + u(1) * std::cos(x_(2));
      double cmd_ang = u(2);
      cmdv_(GENERALIZED_VELOCITY + 0) = 1.0 / wheel_radius_ * (cmd_lin_x - cmd_lin_y - wheels_k_ * cmd_ang);
      cmdv_(GENERALIZED_VELOCITY + 1) = 1.0 / wheel_radius_ * (cmd_lin_x + cmd_lin_y - wheels_k_ * cmd_ang);
      cmdv_(GENERALIZED_VELOCITY + 2) = 1.0 / wheel_radius_ * (cmd_lin_x - cmd_lin_y + wheels_k_ * cmd_ang);
      cmdv_(GENERALIZED_VELOCITY + 3) = 1.0 / wheel_radius_ * (cmd_lin_x + cmd_lin_y + wheels_k_ * cmd_ang);
      cmdv_.head<GENERALIZED_VELOCITY>().setZero();
      // cmdv_.segment(GENERALIZED_VELOCITY, BASE_JOINT_DIMENSION) = constrained_matrix_ * u.head(VIRTUAL_BASE_DIMENSION);
      cmdv_.segment(GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION, ARM_DIMENSION) = u.segment(BASE_INPUT_DIMENSION, ARM_DIMENSION);
      cmdv_.tail<GRIPPER_DIMENSION>().setZero();

      raisim::VecDyn h = husky_panda_->getNonlinearities(gravity_);
      // h.e().head(7)(2) = 0.0; // base z-axis force
      h.e().head(GENERALIZED_COORDINATE + BASE_JOINT_DIMENSION).setZero(); // base non linear force
      husky_panda_->setGeneralizedForce(h);
    }
    else
    {
      // u(0) = u(0) * std::cos(x_(2)) - u(1) * std::sin(x_(2));
      // u(2) = u(2);
      // non-holonomic differential drive constraints
      cmdv_(GENERALIZED_VELOCITY + 0) = (u(0) - u(1) * wheel_separation_ / 2.0) / wheel_radius_;
      cmdv_(GENERALIZED_VELOCITY + 1) = (u(0) + u(1) * wheel_separation_ / 2.0) / wheel_radius_;
      cmdv_(GENERALIZED_VELOCITY + 2) = (u(0) - u(1) * wheel_separation_ / 2.0) / wheel_radius_;
      cmdv_(GENERALIZED_VELOCITY + 3) = (u(0) + u(1) * wheel_separation_ / 2.0) / wheel_radius_;
      cmdv_.head<GENERALIZED_VELOCITY>().setZero();
      // cmdv_.segment(GENERALIZED_VELOCITY, BASE_JOINT_DIMENSION) = constrained_matrix_ * u.head(VIRTUAL_BASE_DIMENSION);
      cmdv_.segment(GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION, ARM_DIMENSION) = u.segment(BASE_INPUT_DIMENSION, ARM_DIMENSION);
      cmdv_.tail<GRIPPER_DIMENSION>().setZero();

      raisim::VecDyn h = husky_panda_->getNonlinearities(gravity_);
      // h.e().head(7)(2) = 0.0; // base z-axis force
      h.e().head(GENERALIZED_COORDINATE + BASE_JOINT_DIMENSION).setZero(); // base non linear force
      husky_panda_->setGeneralizedForce(h);
    }

    husky_panda_->setPdTarget(cmd_, cmdv_);

    obstacle_->setGeneralizedCoordinate(cmd_obstacle_);

    // gravity compensation for obstacle which has a mass
    raisim::VecDyn h_obs = obstacle_->getNonlinearities(gravity_);
    obstacle_->setGeneralizedForce(h_obs);

    // set reference position
    ref_->setPosition(cmd_ref_);
    ref_->setLinearVelocity({0, 0, 0});
    ref_->setAngularVelocity({0, 0, 0});
  }

  husky_panda_control::observation_t HuskyPandaRaisimDynamics::step(const husky_panda_control::input_t &u,
                                                                    const double dt)
  {
    set_control(u);
    advance();
    return x_;
  }

  const husky_panda_control::observation_t HuskyPandaRaisimDynamics::get_state() const { return x_; }

  void HuskyPandaRaisimDynamics::reset(const husky_panda_control::observation_t &x, const double t)
  {
    x_ = x;
    t_ = t;

    if (holonomic_)
    {
      // prismatic x 2, revolute x 1
      // husky_panda_->setState(x_.head<VIRTUAL_BASE_ARM_GRIPPER_DIMENSION>(),
      //              Eigen::VectorXd::Zero(VIRTUAL_BASE_ARM_GRIPPER_DIMENSION));
      Eigen::VectorXd genco(husky_panda_->getGeneralizedCoordinateDim()), genve(husky_panda_->getDOF());
      genco.setZero();
      genve.setZero();
      genco(0) = x(0); // x
      genco(1) = x(1); // y
      genco(2) = 0.2;  // z
      Eigen::Matrix3d R;
      R << std::cos(x(2)), -std::sin(x(2)), 0,
          std::sin(x(2)), std::cos(x(2)), 0,
          0, 0, 1;
      Eigen::Quaterniond q(R);
      genco(3) = q.w();
      genco(4) = q.x();
      genco(5) = q.y();
      genco(6) = q.z();
      genco.segment(GENERALIZED_COORDINATE, BASE_JOINT_DIMENSION).setZero();
      genco.segment(GENERALIZED_COORDINATE + BASE_JOINT_DIMENSION, ARM_GRIPPER_DIMENSION) = x.segment(VIRTUAL_BASE_DIMENSION, ARM_GRIPPER_DIMENSION);
      husky_panda_->setState(genco, genve);
    }
    else
    {
      Eigen::VectorXd genco(husky_panda_->getGeneralizedCoordinateDim()), genve(husky_panda_->getDOF());
      genco.setZero();
      genve.setZero();
      genco(0) = x(0); // x
      genco(1) = x(1); // y
      genco(2) = 0.2;  // z
      Eigen::Matrix3d R;
      R << std::cos(x(2)), -std::sin(x(2)), 0,
          std::sin(x(2)), std::cos(x(2)), 0,
          0, 0, 1;
      Eigen::Quaterniond q(R);
      genco(3) = q.w();
      genco(4) = q.x();
      genco(5) = q.y();
      genco(6) = q.z();
      genco.segment(GENERALIZED_COORDINATE, BASE_JOINT_DIMENSION).setZero();
      genco.segment(GENERALIZED_COORDINATE + BASE_JOINT_DIMENSION, ARM_GRIPPER_DIMENSION) = x.segment(VIRTUAL_BASE_DIMENSION, ARM_GRIPPER_DIMENSION);
      husky_panda_->setState(genco, genve);
    }

    Eigen::VectorXd obstacle_pose(obstacle_->getDOF()), obstacle_velocity(obstacle_->getDOF());
    obstacle_pose.setZero();
    obstacle_pose(0) = 1.0;
    obstacle_pose(2) = 1.0;
    obstacle_velocity.setZero();
    obstacle_->setState(obstacle_pose, obstacle_velocity);
    return;
  }

  husky_panda_control::input_t HuskyPandaRaisimDynamics::get_zero_input(const observation_t &x)
  {
    return husky_panda_control::input_t::Zero(get_input_dimension());
  }

  void HuskyPandaRaisimDynamics::get_end_effector_pose(Eigen::Vector3d& ee_position, Eigen::Quaterniond& ee_orientation)
  {
    size_t frame_id = husky_panda_->getFrameIdxByName("panda_grasp_joint");
    raisim::Vec<3> pos;
    raisim::Mat<3, 3> rot;
    husky_panda_->getFramePosition(frame_id, pos);
    husky_panda_->getFrameOrientation(frame_id, rot);
    ee_position = pos.e();
    ee_orientation = Eigen::Quaterniond(rot.e()).normalized();
    return;
  }

  void HuskyPandaRaisimDynamics::get_base_pose(Eigen::Vector3d& base_position, Eigen::Quaterniond& base_orientation)
  {
    raisim::Vec<3> pos;
    raisim::Mat<3, 3> rot;
    husky_panda_->getBodyPosition(0, pos);
    husky_panda_->getBodyOrientation(0, rot);
    base_position = pos.e();
    base_orientation = Eigen::Quaterniond(rot.e()).normalized();
    return;
  }

} // namespace panda_mobile

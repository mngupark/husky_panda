//
// Created by giuseppe on 01.03.21.
//

#pragma once
// pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <rbdl/Model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace husky_panda_rbdl {

struct Pose {
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;

  Pose() = default;
  Pose(Eigen::Vector3d t, Eigen::Quaterniond r) : translation(t), rotation(r){};
};

Pose operator*(const Pose&, const Pose&);
Eigen::Matrix<double, 6, 1> diff(const Pose&, const Pose&);

class RobotModel {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  RobotModel() = default;
  ~RobotModel();

  RobotModel(const RobotModel& rhs);
  /**
   *
   * @param robot_description
   * @return
   */
  bool init_from_xml(const std::string& robot_description);

  /**
   *
   * @param q
   */
  void update_state(const Eigen::VectorXd& q);

  /**
   *
   * @param q
   * @param qd
   */
  void update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd);

  /**
   *
   * @param from_frame
   * @param to_frame
   * @param error
   */
  void get_error(const std::string& from_frame, const std::string& to_frame,
                 Vector6d& error) const;
  /**
   *
   * @param frame
   * @param rot
   * @param trans
   * @param error
   */
  void get_error(const std::string& frame, const Eigen::Quaterniond& rot,
                 const Eigen::Vector3d& trans, Vector6d& error) const;

  /**
   *
   * @param from_frame
   * @param to_frame
   * @param offset
   */
  void get_offset(const std::string& from_frame, const std::string& to_frame,
                  Eigen::Vector3d& offset);

  /**
   *
   * @param frame
   * @param q
   */
  Pose get_pose(const std::string& frame, const Eigen::VectorXd& q) const;

  void print_info() const;

 private:
  // pinocchio
  // pinocchio::Model* model_;
  // pinocchio::Data* data_;

  RigidBodyDynamics::Model* model_;
  Eigen::VectorXd q_;
  Eigen::Matrix3d global_rotate_;
};
}  // namespace husky_panda_rbdl

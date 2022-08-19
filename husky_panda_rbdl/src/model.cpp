// pinocchio
#include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/model.hpp>
// #include <pinocchio/multibody/data.hpp>
// #include <pinocchio/parsers/urdf.hpp>

#include "husky_panda_rbdl/model.h"

// pinocchio
using namespace pinocchio;

namespace husky_panda_rbdl {

Eigen::Matrix<double, 6, 1> diff(const Pose& p1, const Pose& p2) {
  // pinocchio
  return log6(SE3(p1.rotation, p1.translation)
                  .actInv(SE3(p2.rotation, p2.translation)))
      .toVector();
}

Pose operator*(const Pose& p1, const Pose& p2) {
  Pose res;
  // pinocchio
  SE3 temp(
      SE3(p1.rotation, p1.translation).act(SE3(p2.rotation, p2.translation)));
  res.translation = temp.translation();
  res.rotation = temp.rotation();
  return res;
}

RobotModel::~RobotModel() {
  delete model_;
  // pinocchio
  // delete data_;
};

// deep copy
RobotModel::RobotModel(const RobotModel& rhs) {
  // pinocchio
  model_ = new RigidBodyDynamics::Model(*rhs.model_);
  // model_ = new pinocchio::Model(*rhs.model_);
  // data_ = new pinocchio::Data(*rhs.data_);
}

void RobotModel::print_info() const {
  std::stringstream ss;
  // pinocchio
  // ss << "Robot model has " << model_->nq << " dofs. joint dofs : " << model_->nv << std::endl;
  std::cout << ss.str();
}

bool RobotModel::init_from_xml(const std::string& robot_description) {
  // pinocchio
  // try {
  //   model_ = new Model();
  //   pinocchio::urdf::buildModelFromXML(robot_description, *model_);
  //   data_ = new Data(*model_);
  // } catch (std::runtime_error& exc) {
  //   std::cout << exc.what();
  //   return false;
  // } catch (...) {
  //   std::cout << "Unknown exception caught while building model." << std::endl;
  //   return false;
  // }
  // return true;
  bool floating_base = false;
  bool verbose = false;
  model_ = new RigidBodyDynamics::Model();
  // RigidBodyDynamics::Addons::URDFReadFromFile("/home/keunwoo/husky_panda_ws/src/husky_panda/husky_panda_manipulation/data/husky_panda_planar.urdf", model_, floating_base, verbose);
  RigidBodyDynamics::Addons::URDFReadFromString(robot_description.c_str(), model_, floating_base, verbose);
  q_.setZero(model_->dof_count);
  global_rotate_.setZero();
  global_rotate_ << std::cos(M_PI), -std::sin(M_PI), 0,
                    std::sin(M_PI),  std::cos(M_PI), 0,
                    0,               0,              1;
  // std::cout << "Model DoFs: " << model_->dof_count << '\n';
  return true;
}

void RobotModel::update_state(const Eigen::VectorXd& q) {
  // pinocchio
  // forwardKinematics(*model_, *data_, q);
  // updateFramePlacements(*model_, *data_);
  RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q, NULL, NULL);
  q_ = q;
  return;
}

void RobotModel::update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd) {
  // pinocchio
  // forwardKinematics(*model_, *data_, q, qd);
  // updateFramePlacements(*model_, *data_);
  RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q, &qd, NULL);
  q_ = q;
  return;
}

void RobotModel::get_error(const std::string& from_frame,
                           const std::string& to_frame, Vector6d& error) const {
  // pinocchio
  // error = log6(data_->oMf[model_->getFrameId(to_frame)].actInv(
  //                  data_->oMf[model_->getFrameId(from_frame)]))
  //             .toVector();
  // size_t from_idx = model_->GetBodyId(from_frame.c_str());
  // size_t to_idx = model_->GetBodyId(to_frame.c_str());
  // RigidBodyDynamics::Math::SpatialTransform tf = model_->GetJointFrame(from_idx);
  // tf.inverse()
}

void RobotModel::get_offset(const std::string& from_frame,
                            const std::string& to_frame,
                            Eigen::Vector3d& offset) {
  // pinocchio
  // offset = data_->oMf[model_->getFrameId(to_frame)].translation() -
  //          data_->oMf[model_->getFrameId(from_frame)].translation();
}

void RobotModel::get_error(const std::string& frame,
                           const Eigen::Quaterniond& rot,
                           const Eigen::Vector3d& trans,
                           Vector6d& error) const {
  // pinocchio
  // error = log6(data_->oMf[model_->getFrameId(frame)].actInv(SE3(rot, trans)))
  //             .toVector();
}

Pose RobotModel::get_pose(const std::string& frame, const Eigen::VectorXd& q) const {
  Pose pose;
  // pinocchio
  // pose.translation = data_->oMf[model_->getFrameId(frame)].translation();
  // pose.rotation = data_->oMf[model_->getFrameId(frame)].rotation();
  pose.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model_, q, model_->GetBodyId(frame.c_str()), Eigen::Vector3d::Zero(3), false);
  pose.rotation = Eigen::Quaterniond(RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q, model_->GetBodyId(frame.c_str()), false));
  return pose;
}

}  // namespace husky_panda_rbdl
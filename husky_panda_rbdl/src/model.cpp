// pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

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
  // rbdl
  // delete model_;
  // pinocchio
  delete model_;
  delete data_;
};

// deep copy
RobotModel::RobotModel(const RobotModel& rhs) {
  // rbdl
  // model_ = new RigidBodyDynamics::Model(*rhs.model_);
  // pinocchio
  model_ = new pinocchio::Model(*rhs.model_);
  data_ = new pinocchio::Data(*rhs.data_);
}

void RobotModel::print_info() const {
  std::stringstream ss;
  // pinocchio
  ss << "Robot model has " << model_->nq << " dofs. joint dofs : " << model_->nv << std::endl;
  std::cout << ss.str();
}

bool RobotModel::init_from_xml(const std::string& robot_description) {
  // pinocchio
  try {
    model_ = new Model();
    pinocchio::urdf::buildModelFromXML(robot_description, *model_);
    data_ = new Data(*model_);
  } catch (std::runtime_error& exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception caught while building model." << std::endl;
    return false;
  }
  frame_id_.resize(12);
  frame_id_[0]  = "x_slider";
  frame_id_[1]  = "y_slider";
  frame_id_[2]  = "pivot";
  frame_id_[3]  = "panda_link1";
  frame_id_[4]  = "panda_link2";
  frame_id_[5]  = "panda_link3";
  frame_id_[6]  = "panda_link4";
  frame_id_[7]  = "panda_link5";
  frame_id_[8]  = "panda_link6";
  frame_id_[9]  = "panda_link7";
  frame_id_[10] = "panda_leftfinger";
  frame_id_[11] = "panda_rightfinger";

  // rbdl
  // bool floating_base = false;
  // bool verbose = false;
  // model_ = new RigidBodyDynamics::Model();
  // RigidBodyDynamics::Addons::URDFReadFromString(robot_description.c_str(), model_, floating_base, verbose);
  // std::cout << "model dof: " << model_->dof_count << "\n";
  // body_id_.resize(model_->dof_count);
  // body_id_[0]  = (model_->GetBodyId("x_slider"));
  // body_id_[1]  = (model_->GetBodyId("y_slider"));
  // body_id_[2]  = (model_->GetBodyId("pivot"));
  // body_id_[3]  = (model_->GetBodyId("panda_link1"));
  // body_id_[4]  = (model_->GetBodyId("panda_link2"));
  // body_id_[5]  = (model_->GetBodyId("panda_link3"));
  // body_id_[6]  = (model_->GetBodyId("panda_link4"));
  // body_id_[7]  = (model_->GetBodyId("panda_link5"));
  // body_id_[8]  = (model_->GetBodyId("panda_link6"));
  // body_id_[9]  = (model_->GetBodyId("panda_link7"));
  // body_id_[10] = (model_->GetBodyId("panda_leftfinger"));
  // body_id_[11] = (model_->GetBodyId("panda_rightfinger"));
  
  // q_.setZero(model_->dof_count);
  // global_rotate_.setZero();
  // global_rotate_ << std::cos(M_PI), -std::sin(M_PI), 0,
  //                   std::sin(M_PI),  std::cos(M_PI), 0,
  //                   0,               0,              1;
  
  return true;
}

void RobotModel::update_state(const Eigen::VectorXd& q) {
  // pinocchio
  forwardKinematics(*model_, *data_, q);
  updateFramePlacements(*model_, *data_);
  // rbdl
  // RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q, NULL, NULL);
  // q_ = q;
  return;
}

void RobotModel::update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd) {
  // pinocchio
  forwardKinematics(*model_, *data_, q, qd);
  updateFramePlacements(*model_, *data_);
  // rbdl
  // RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q, &qd, NULL);
  // q_ = q;
  return;
}

void RobotModel::get_error(const std::string& from_frame,
                           const std::string& to_frame, Vector6d& error) const {
  // pinocchio
  error = log6(data_->oMf[model_->getFrameId(to_frame)].actInv(
                   data_->oMf[model_->getFrameId(from_frame)]))
              .toVector();
  // rbdl
  // size_t from_idx = model_->GetBodyId(from_frame.c_str());
  // size_t to_idx = model_->GetBodyId(to_frame.c_str());
  // RigidBodyDynamics::Math::SpatialTransform tf = model_->GetJointFrame(from_idx);
  // tf.inverse()
}

void RobotModel::get_offset(const std::string& from_frame,
                            const std::string& to_frame,
                            Eigen::Vector3d& offset) {
  // pinocchio
  offset = data_->oMf[model_->getFrameId(to_frame)].translation() -
           data_->oMf[model_->getFrameId(from_frame)].translation();
}

void RobotModel::get_offset(const std::string& from_frame,
                            const std::string& to_frame,
                            Eigen::Vector3d& offset,
                            const Eigen::VectorXd& q) {
  // pinocchio
  offset = data_->oMf[model_->getFrameId(to_frame)].translation() -
           data_->oMf[model_->getFrameId(from_frame)].translation();
  // rbdl
  // offset = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model_, q, model_->GetBodyId(from_frame.c_str()), Eigen::Vector3d::Zero(3), false) -
  //          RigidBodyDynamics::CalcBaseToBodyCoordinates(*model_, q, model_->GetBodyId(to_frame.c_str()), Eigen::Vector3d::Zero(3), false);
  return;
}

void RobotModel::get_error(const std::string& frame,
                           const Eigen::Quaterniond& rot,
                           const Eigen::Vector3d& trans,
                           Vector6d& error) const {
  // pinocchio
  error = log6(data_->oMf[model_->getFrameId(frame)].actInv(SE3(rot, trans)))
              .toVector();
}

Pose RobotModel::get_pose(const std::string& frame, const Eigen::VectorXd& q) const {
  Pose pose;
  // pinocchio
  pose.translation = data_->oMf[model_->getFrameId(frame)].translation();
  pose.rotation = data_->oMf[model_->getFrameId(frame)].rotation();
  // rbdl
  // pose.translation = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model_, q, model_->GetBodyId(frame.c_str()), Eigen::Vector3d::Zero(3), false);
  // pose.rotation = Eigen::Quaterniond(RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q, model_->GetBodyId(frame.c_str()), false));
  return pose;
}

Pose RobotModel::get_pose(const size_t idx, const Eigen::VectorXd& q) const {
  Pose pose;
  // rbdl
  // pose.translation = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model_, q, body_id_[idx], Eigen::Vector3d::Zero(3), false);
  // pose.rotation = Eigen::Quaterniond(RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q, body_id_[idx], false));
  return pose;
}


}  // namespace husky_panda_rbdl
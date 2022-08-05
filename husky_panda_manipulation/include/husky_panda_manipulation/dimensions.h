/*!
 * @file     dimensions.h
 * @author   Mingyu Park
 * @date     04.08.2022
 * @version  1.0
 * @brief    Dimensions for state and input of the robot
 */
#pragma once

enum HuskyPandaDim : char {
  GRIPPER_DIMENSION = 2,        // position of the gripper fingers (2)
  ARM_DIMENSION = 7,            // arm only joints
  BASE_DIMENSION = 3,           // x, y, theta
  BASE_JOINT_DIMENSION = 4,     // front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel,
  FLOATING_BASE_DIMENSION = 6,  // for floating base, additional DoFs are required
  OBJECT_DIMENSION = 1,
  CONTACT_STATE = 1,
  TORQUE_DIMENSION = BASE_JOINT_DIMENSION + ARM_DIMENSION + GRIPPER_DIMENSION,

  ARM_GRIPPER_DIM = ARM_DIMENSION + GRIPPER_DIMENSION,
  BASE_ARM_GRIPPER_DIM = BASE_DIMENSION + ARM_GRIPPER_DIM,

  /*
   * @brief state and input dimensions
   * @arg STATE_DIMENSION : [x_base y_base theta_base q_arm]^T
   * @arg INPUT_DIMENSION : [q_fl q_fr q_rl q_rr q_arm]^T (non-holonomic mobile robot)
   */
  STATE_DIMENSION = BASE_ARM_GRIPPER_DIM * 2 + OBJECT_DIMENSION * 2 + CONTACT_STATE + TORQUE_DIMENSION,
  INPUT_DIMENSION = BASE_JOINT_DIMENSION + ARM_GRIPPER_DIM - 1, // mimic-joint for gripper

  REFERENCE_POSE_DIMENSION = 7,
  REFERENCE_OBSTACLE = 3,
  REFERENCE_OBJECT_DIMENSION = 1,
  REFERENCE_FLAG = 1,  // trigger some costs components

  REFERENCE_DIMENSION = REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE +
                        REFERENCE_OBJECT_DIMENSION + REFERENCE_FLAG
};

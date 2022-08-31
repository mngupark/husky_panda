#pragma once
#ifndef __DIMENSIONS_H__
#define __DIMENSIONS_H__

enum HuskyPandaMobileDim
{
      REFERENCE_DIMENSION = 10,                       // x_t, x_q, obstacle_t

      GENERALIZED_COORDINATE = 7,                     // translation vector(x,y,z) and orientation quaternion(w,x,y,z) of simulation
      GENERALIZED_VELOCITY = 6,                       // linear velocity vector(x,y,z) and angular velocity vector(roll,pitch,yaw) of simulation
      VIRTUAL_BASE_DIMENSION = 3,                     // translation x and y, orientation yaw(z-axis) for mobile base
      BASE_JOINT_DIMENSION = 4,                       // base wheel joint dimension
      ARM_DIMENSION = 7,                              // arm joint dimension
      GRIPPER_DIMENSION = 2,                          // gripper joint dimension
      
      ARM_GRIPPER_DIMENSION = ARM_DIMENSION + GRIPPER_DIMENSION,
      VIRTUAL_BASE_ARM_GRIPPER_DIMENSION = VIRTUAL_BASE_DIMENSION + ARM_GRIPPER_DIMENSION,
      BASE_JOINT_ARM_GRIPPER_DIMENSION = BASE_JOINT_DIMENSION + ARM_GRIPPER_DIMENSION,

      STATE_DIMENSION = VIRTUAL_BASE_ARM_GRIPPER_DIMENSION, // x, y, yaw, q_arm, q_gripper
      INPUT_DIMENSION = VIRTUAL_BASE_ARM_GRIPPER_DIMENSION - 1, // xdot, yawdot, qdot_arm, qdot_gripper
      SIM_STATE_DIMENSION = GENERALIZED_COORDINATE + BASE_JOINT_ARM_GRIPPER_DIMENSION, // genco + wheel + arm + gripper
      SIM_INPUT_DIMENSION = GENERALIZED_VELOCITY + BASE_JOINT_ARM_GRIPPER_DIMENSION, // genve + wheel + arm + gripper
      BASE_INPUT_DIMENSION = VIRTUAL_BASE_DIMENSION - 1 // xdot, yawdot
};

#endif // !__DIMENSIONS_H__
// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "odometry_flattener/odometry_flattener_node.h"
namespace nvblox {

OdometryFlattenerNode::OdometryFlattenerNode(const rclcpp::NodeOptions & options)
    : Node("odometry_flattener_node", options) {
  RCLCPP_INFO(get_logger(), "Creating a OdometryFlattenerNode().");

  // Parameters
  input_parent_frame_id_ = declare_parameter<std::string>(
      "input_parent_frame_id", input_parent_frame_id_);
  input_child_frame_id_ = declare_parameter<std::string>(
      "input_child_frame_id", input_child_frame_id_);
  output_parent_frame_id_ = declare_parameter<std::string>(
      "output_parent_frame_id", output_parent_frame_id_);
  output_child_frame_id_ = declare_parameter<std::string>(
      "output_child_frame_id", output_child_frame_id_);
  invert_output_transform_ = declare_parameter<bool>(
      "invert_output_transform", invert_output_transform_);

  // Declare covariance parameters
  position_variance_ = declare_parameter<double>("position_variance", 0.05);  // Variance in position (m^2)
  orientation_variance_ = declare_parameter<double>("orientation_variance", 0.02);  // Variance in orientation (rad^2)
  linear_velocity_variance_ = declare_parameter<double>("linear_velocity_variance", 0.05);  // Variance in linear velocity (m^2/s^2)
  angular_velocity_variance_ = declare_parameter<double>("angular_velocity_variance", 0.02);  // Variance in angular velocity (rad^2/s^2)

  // Subscribe to tf
  constexpr size_t qos_history_depth = 10;
  tf2_message_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", qos_history_depth,
      std::bind(&OdometryFlattenerNode::tfMessageCallback, this,
                std::placeholders::_1));

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscribe to odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "visual_slam/tracking/odometry", qos_history_depth,
      std::bind(&OdometryFlattenerNode::odometryCallback, this,
                std::placeholders::_1));

  // Initialize the odometry publisher
  flattened_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "flattened_odom", qos_history_depth);
}

void OdometryFlattenerNode::tfMessageCallback(
    tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {

  // Search for the right transform in the list
  geometry_msgs::msg::TransformStamped transform_msg;
  bool transform_found = false;
  bool input_transform_inverted = false;
  for (const auto& transform : msg->transforms) {
    if (transform.child_frame_id == input_child_frame_id_ &&
        transform.header.frame_id == input_parent_frame_id_) {
      transform_msg = transform;
      transform_found = true;
      input_transform_inverted = false;
      break;
    }
    if (transform.child_frame_id == input_parent_frame_id_ &&
        transform.header.frame_id == input_child_frame_id_) {
      transform_msg = transform;
      transform_found = true;
      input_transform_inverted = true;
      break;
    }
  }
  if (!transform_found) {
    return;
  }

  // To Eigen
  Eigen::Isometry3d T_parent_child = tf2::transformToEigen(transform_msg);

  if (input_transform_inverted) {
    T_parent_child = T_parent_child.inverse();
  }

  // Flatten
  // We achieve this by projecting the vector part of the quaternion to the z
  // axis (by zeroing the other components) and then renormalizing.
  const auto q = Eigen::Quaterniond(T_parent_child.rotation());
  const auto q_flattened =
      Eigen::Quaterniond(q.w(), 0.0, 0.0, q.z()).normalized();
  const auto t_flattened = Eigen::Vector3d(
      T_parent_child.translation().x(), T_parent_child.translation().y(), 0.0);
  Eigen::Isometry3d T_flattened = Eigen::Isometry3d::Identity();
  T_flattened.prerotate(q_flattened);
  T_flattened.pretranslate(t_flattened);

  // Invert if requested 
  if (invert_output_transform_) {
    T_flattened = T_flattened.inverse();
  }

  // Re-broadcast
  geometry_msgs::msg::TransformStamped msg_flattened =
      tf2::eigenToTransform(T_flattened);
  msg_flattened.header.stamp = transform_msg.header.stamp;
  if (invert_output_transform_) {
    msg_flattened.child_frame_id = output_parent_frame_id_;
    msg_flattened.header.frame_id = output_child_frame_id_;
  } else {
    msg_flattened.child_frame_id = output_child_frame_id_;
    msg_flattened.header.frame_id = output_parent_frame_id_;
  }
  tf_broadcaster_->sendTransform(msg_flattened);
}

void OdometryFlattenerNode::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Extract the pose
  const geometry_msgs::msg::Pose& pose_msg = msg->pose.pose;

  // Convert to Eigen Isometry3d
  Eigen::Isometry3d T;
  tf2::fromMsg(pose_msg, T);

  // Flatten the transform
  const auto q = Eigen::Quaterniond(T.rotation());
  const auto q_flattened =
    Eigen::Quaterniond(q.w(), 0.0, 0.0, q.z()).normalized();
  const auto t_flattened = Eigen::Vector3d(
    T.translation().x(), T.translation().y(), 0.0);
  Eigen::Isometry3d T_flattened = Eigen::Isometry3d::Identity();
  T_flattened.prerotate(q_flattened);
  T_flattened.pretranslate(t_flattened);

  // Convert back to Pose message
  geometry_msgs::msg::Pose flattened_pose_msg = tf2::toMsg(T_flattened);

  // Flatten the linear velocity
  geometry_msgs::msg::Vector3 flattened_linear_velocity = msg->twist.twist.linear;
  flattened_linear_velocity.z = 0.0;

  // Flatten the angular velocity
  geometry_msgs::msg::Vector3 flattened_angular_velocity = msg->twist.twist.angular;
  flattened_angular_velocity.x = 0.0;
  flattened_angular_velocity.y = 0.0;

  // Create new odometry message
  nav_msgs::msg::Odometry flattened_odom_msg = *msg;  // Copy original message
  flattened_odom_msg.pose.pose = flattened_pose_msg;
  flattened_odom_msg.twist.twist.linear = flattened_linear_velocity;
  flattened_odom_msg.twist.twist.angular = flattened_angular_velocity;

  // Set child_frame_id and frame_id
  flattened_odom_msg.child_frame_id = output_child_frame_id_;
  flattened_odom_msg.header.frame_id = output_parent_frame_id_;

  // Update the covariance matrices with realistic values
  setCovarianceMatrices(flattened_odom_msg);

  // Publish the flattened odometry
  flattened_odom_pub_->publish(flattened_odom_msg);
}

void OdometryFlattenerNode::setCovarianceMatrices(nav_msgs::msg::Odometry& odom_msg) {
  // Initialize pose covariance matrix with zeros
  std::array<double, 36> pose_covariance = {0.0};

  // Set variances for X and Y positions
  pose_covariance[0] = position_variance_;  // Variance in X
  pose_covariance[7] = position_variance_;  // Variance in Y

  // Set large variance for Z position (not used)
  pose_covariance[14] = 1e6;  // Variance in Z

  // Set large variances for Roll and Pitch (not used)
  pose_covariance[21] = 1e6;  // Variance in Roll
  pose_covariance[28] = 1e6;  // Variance in Pitch

  // Set variance for Yaw
  pose_covariance[35] = orientation_variance_;  // Variance in Yaw

  // Assign the updated pose covariance to the odometry message
  odom_msg.pose.covariance = pose_covariance;

  // Initialize twist covariance matrix with zeros
  std::array<double, 36> twist_covariance = {0.0};

  // Set variance for linear velocity X and Y
  twist_covariance[0] = linear_velocity_variance_;  // Variance in linear velocity X
  twist_covariance[7] = linear_velocity_variance_;  // Variance in linear velocity Y

  // Set large variance for linear velocity Z (not used)
  twist_covariance[14] = 1e6;  // Variance in linear velocity Z

  // Set large variances for angular velocities Roll and Pitch (not used)
  twist_covariance[21] = 1e6;  // Variance in angular velocity X
  twist_covariance[28] = 1e6;  // Variance in angular velocity Y

  // Set variance for angular velocity Yaw
  twist_covariance[35] = angular_velocity_variance_;  // Variance in angular velocity Z

  // Assign the updated twist covariance to the odometry message
  odom_msg.twist.covariance = twist_covariance;
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::OdometryFlattenerNode)

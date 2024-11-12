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
  bool input_transform_inverted = false;

  if (msg->header.frame_id == input_parent_frame_id_ &&
      msg->child_frame_id == input_child_frame_id_) {
    input_transform_inverted = false;
  } else if (msg->header.frame_id == input_child_frame_id_ &&
             msg->child_frame_id == input_parent_frame_id_) {
    input_transform_inverted = true;
  } else {
    // Not the frames we are interested in
    return;
  }

  // Extract the pose
  const geometry_msgs::msg::Pose& pose_msg = msg->pose.pose;

  // Convert to Eigen Isometry3d
  Eigen::Isometry3d T;
  tf2::fromMsg(pose_msg, T);

  if (input_transform_inverted) {
    T = T.inverse();
  }

  // Flatten the transform
  const auto q = Eigen::Quaterniond(T.rotation());
  const auto q_flattened =
      Eigen::Quaterniond(q.w(), 0.0, 0.0, q.z()).normalized();
  const auto t_flattened = Eigen::Vector3d(
      T.translation().x(), T.translation().y(), 0.0);
  Eigen::Isometry3d T_flattened = Eigen::Isometry3d::Identity();
  T_flattened.prerotate(q_flattened);
  T_flattened.pretranslate(t_flattened);

  // Invert if requested
  if (invert_output_transform_) {
    T_flattened = T_flattened.inverse();
  }

  // Convert back to Pose message
  geometry_msgs::msg::Pose flattened_pose_msg = tf2::toMsg(T_flattened);

  // Create new odometry message
  nav_msgs::msg::Odometry flattened_odom_msg = *msg;  // Copy original message
  flattened_odom_msg.pose.pose = flattened_pose_msg;

  // Set child_frame_id and frame_id
  if (invert_output_transform_) {
    flattened_odom_msg.child_frame_id = output_parent_frame_id_;
    flattened_odom_msg.header.frame_id = output_child_frame_id_;
  } else {
    flattened_odom_msg.child_frame_id = output_child_frame_id_;
    flattened_odom_msg.header.frame_id = output_parent_frame_id_;
  }

  // Publish the flattened odometry
  flattened_odom_pub_->publish(flattened_odom_msg);
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::OdometryFlattenerNode)

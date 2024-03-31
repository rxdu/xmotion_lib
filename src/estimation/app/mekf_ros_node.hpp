/*
 * @file mekf_ros_node.hpp
 * @date 3/31/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MEKF_ROS_NODE_HPP
#define XMOTION_MEKF_ROS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "interface/driver/imu_interface.hpp"

namespace xmotion {
class MekfRosNode : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

 public:
  MekfRosNode(std::shared_ptr<ImuInterface> imu);

  bool Initialize(std::string dev_name, uint32_t baud_rate = 0);

 private:
  void ImuCallback(const ImuData &data);

  std::shared_ptr<ImuInterface> imu_;
  TimePoint last_update_time_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};
}  // namespace xmotion

#endif  // XMOTION_MEKF_ROS_NODE_HPP

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <array>
#include <cstddef>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
// ===== 任务6: 里程计（新增） =====
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
// ===== 任务6: 里程计（新增结束） =====

namespace hero_chassis_controller {

class HeroChassisController: public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;  

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void cmdVelCallback(const geometry_msgs::Twist& msg); 

  std::array<hardware_interface::JointHandle, 4> joints_;  //每个轮子对应一个 JointHandle
  std::array<control_toolbox::Pid, 4> pids_;  //每个轮子都有一个 PID 控制器
  std::array<double, 4> target_velocity_{};  //目标角速度
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;  //保证回调线程和实时控制线程之间的数据安全
  double wheel_radius_{0.07625};
  double wheel_base_{0.4};
  double wheel_track_{0.4};
  double max_effort_{5.0};
  // ===== 任务8: 底盘加速度限制（新增） =====
  double max_linear_acceleration_{0.0};   // m/s^2，<=0 表示不限制
  double max_angular_acceleration_{0.0};  // rad/s^2，<=0 表示不限制
  double vx_cmd_{0.0};
  double vy_cmd_{0.0};
  double wz_cmd_{0.0};
  // ===== 任务8: 底盘加速度限制（新增结束） =====
  ros::Subscriber cmd_vel_sub_;
  // ===== 任务6: 里程计（新增） =====
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_th_{0.0};
  ros::Time last_odom_time_;
  tf::TransformBroadcaster odom_broadcaster_;
  realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
  // ===== 任务6: 里程计（新增结束） =====
};

}  // namespace hero_chassis_coddntroller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

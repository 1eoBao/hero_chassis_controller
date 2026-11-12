#include "hero_chassis_controller/hero_chassis_controller.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <pluginlib/class_list_macros.hpp>
// ===== 任务6: 里程计（新增） =====
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// ===== 任务6: 里程计（新增结束） =====

namespace hero_chassis_controller {

namespace {
constexpr std::size_t kWheelNum = 4;
const char* const kWheelNames[kWheelNum] = {
    "left_front_wheel_joint", "right_front_wheel_joint",
    "left_back_wheel_joint", "right_back_wheel_joint"};
const char* const kPidNamespaces[kWheelNum] = {
    "left_front_wheel", "right_front_wheel",
    "left_back_wheel", "right_back_wheel"};
// ===== 任务8: 底盘加速度限制（新增） =====
double slewLimit(double current, double target, double max_delta) {
  if (max_delta <= 0.0) {
    return target;  // <=0 表示不限制
  }
  const double diff = target - current;
  if (std::fabs(diff) <= max_delta) {
    return target;
  }
  return current + (diff > 0.0 ? max_delta : -max_delta);
}
// ===== 任务8: 底盘加速度限制（新增结束） =====
}  // namespace

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  controller_nh.param<double>("wheel_base", wheel_base_, 0.4);
  controller_nh.param<double>("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param<double>("wheel_track", wheel_track_, 0.4);
  controller_nh.param<double>("max_effort", max_effort_, 5.0);
  // ===== 任务8: 底盘加速度限制（新增） =====
  controller_nh.param<double>("max_linear_acceleration", max_linear_acceleration_, 0.0);
  controller_nh.param<double>("max_angular_acceleration", max_angular_acceleration_, 0.0);
  // ===== 任务8: 底盘加速度限制（新增结束） =====

  for (std::size_t i = 0; i < kWheelNum; ++i) {
    joints_[i] = effort_joint_interface->getHandle(kWheelNames[i]);

    ros::NodeHandle pid_nh(controller_nh, kPidNamespaces[i]);
    if (!pids_[i].init(pid_nh)) {
      ROS_ERROR("Failed to initialize PID for joint %s", kWheelNames[i]);
      return false;
    }
  }

  cmd_vel_sub_ = root_nh.subscribe(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);  // 输入目标速度集

  // ===== 任务6: 里程计（新增） =====
  odom_pub_.init(root_nh, "/odom", 50);
  // ===== 任务6: 里程计（新增结束） =====

  return true;
}

void HeroChassisController::starting(const ros::Time& time) {
  for (auto& pid : pids_) {
    pid.reset();
  }
  target_velocity_.fill(0.0);
  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
  // ===== 任务8: 底盘加速度限制（新增） =====
  vx_cmd_ = 0.0;
  vy_cmd_ = 0.0;
  wz_cmd_ = 0.0;
  // ===== 任务8: 底盘加速度限制（新增结束） =====
  // ===== 任务6: 里程计（新增） =====
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_th_ = 0.0;
  last_odom_time_ = time;
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  const geometry_msgs::Twist cmd = *cmd_vel_buffer_.readFromRT();

  const double k = (wheel_base_ + wheel_track_) / 2.0;
  const double r = wheel_radius_;
  // ===== 任务8: 底盘加速度限制（新增） =====
  const double dt = period.toSec();
  vx_cmd_ = slewLimit(vx_cmd_, cmd.linear.x, max_linear_acceleration_ * dt);
  vy_cmd_ = slewLimit(vy_cmd_, cmd.linear.y, max_linear_acceleration_ * dt);
  wz_cmd_ = slewLimit(wz_cmd_, cmd.angular.z, max_angular_acceleration_ * dt);
  const double vx = vx_cmd_;
  const double vy = vy_cmd_;
  const double wz = wz_cmd_;
  // ===== 任务8: 底盘加速度限制（新增结束） =====

  target_velocity_[0] = (vx - vy - k * wz) / r;  // left_front
  target_velocity_[1] = (vx + vy + k * wz) / r;  // right_front
  target_velocity_[2] = (vx + vy - k * wz) / r;  // left_back
  target_velocity_[3] = (vx - vy + k * wz) / r;  // right_back

  for (std::size_t i = 0; i < kWheelNum; ++i) {  // 核心
    const double error = target_velocity_[i] - joints_[i].getVelocity();
    double effort = pids_[i].computeCommand(error, period);
    effort = std::max(-max_effort_, std::min(max_effort_, effort));
    joints_[i].setCommand(effort);
  }

  // ===== 任务6: 里程计（新增） =====
  // 正运动学：由四个轮子的实际角速度求底盘在 base_link 下的速度
  const double w0 = joints_[0].getVelocity();
  const double w1 = joints_[1].getVelocity();
  const double w2 = joints_[2].getVelocity();
  const double w3 = joints_[3].getVelocity();

  const double base_vx = r * (w0 + w1 + w2 + w3) / 4.0;
  const double base_vy = r * (-w0 + w1 + w2 - w3) / 4.0;
  const double base_wz = r * (-w0 + w1 - w2 + w3) / (4.0 * k);

  // 积分得到 odom 坐标系下的位姿
  odom_x_ += (base_vx * std::cos(odom_th_) - base_vy * std::sin(odom_th_)) * dt;
  odom_y_ += (base_vx * std::sin(odom_th_) + base_vy * std::cos(odom_th_)) * dt;
  odom_th_ += base_wz * dt;

  // 限频约 50Hz 发布 /odom 与 tf(odom -> base_link)
  if ((time - last_odom_time_).toSec() >= 0.02) {
    last_odom_time_ = time;
    const geometry_msgs::Quaternion q =
        tf::createQuaternionMsgFromYaw(odom_th_);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = q;
    odom_broadcaster_.sendTransform(tf_msg);

    if (odom_pub_.trylock()) {
      nav_msgs::Odometry& odom = odom_pub_.msg_;
      odom.header.stamp = time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = odom_x_;
      odom.pose.pose.position.y = odom_y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = q;
      odom.twist.twist.linear.x = base_vx;
      odom.twist.twist.linear.y = base_vy;
      odom.twist.twist.angular.z = base_wz;
      odom_pub_.unlockAndPublish();
    }
  }
  // ===== 任务6: 里程计 =====
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  cmd_vel_buffer_.writeFromNonRT(msg);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

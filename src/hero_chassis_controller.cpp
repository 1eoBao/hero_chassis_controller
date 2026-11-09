#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*controller_nh*/) {
  static const char* const kJointNames[4] = {
      "left_front_wheel_joint", "right_front_wheel_joint",
      "left_back_wheel_joint", "right_back_wheel_joint"};

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    joints_[i] = effort_joint_interface->getHandle(kJointNames[i]);
  }
  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {}

void HeroChassisController::update(const ros::Time& /*time*/,
                                   const ros::Duration& /*period*/) {
  for (auto& joint : joints_) {
    joint.setCommand(0.0);
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

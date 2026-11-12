# hero_chassis_controller

## 简介

`hero_chassis_controller` 是一个用于 RoboMaster 英雄机器人麦克纳姆轮底盘的 ROS 控制器。
它通过 `ros_control` 以 pluginlib 插件的形式加载到 Gazebo 仿真中，实现底盘的闭环速度控制、
逆运动学、里程计以及底盘加速度限制。

## 主要功能

- 基于 `/cmd_vel` 的麦克纳姆轮逆运动学，计算四个轮子的目标转速
- 每个轮子一个 `control_toolbox::Pid` 速度环，输出力矩控制
- 由四个轮子的实际转速做正运动学，积分得到里程计
- 发布 `nav_msgs/Odometry` 到 `/odom`，并广播 `odom -> base_link` 的 tf
- 可在配置文件中设定底盘线加速度、角加速度上限

## 依赖

- ROS Noetic
- `ros_control`、`controller_interface`、`hardware_interface`、`pluginlib`
- `control_toolbox`、`realtime_tools`、`dynamic_reconfigure`
- `geometry_msgs`、`nav_msgs`、`tf`
- `rm_description_for_task`（仿真模型）

## 编译

```bash
cd ~/ROSPrograms/catkin_ws
catkin build hero_chassis_controller
source devel/setup.bash
```

## 运行

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
```

发送速度指令：

```bash
# 前进 0.2 m/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"

# 停止
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"
```

## 接口

### 订阅

- `/cmd_vel` (`geometry_msgs/Twist`)：底盘目标速度，位于 `base_link` 坐标系

### 发布

- `/odom` (`nav_msgs/Odometry`)：里程计
- `/tf`：`odom` 到 `base_link` 的坐标变换

## 主要参数

参数位于 `/controller/hero_chassis_controller` 命名空间下，见 `config/controllers.yaml`。

| 参数 | 说明 |
| --- | --- |
| `wheel_radius` | 轮子半径 (m) |
| `wheel_base` | 前后轮距 (m) |
| `wheel_track` | 左右轮距 (m) |
| `max_effort` | 轮子最大力矩 (N·m) |
| `max_linear_acceleration` | 线加速度上限 (m/s²)，`<=0` 表示不限制 |
| `max_angular_acceleration` | 角加速度上限 (rad/s²)，`<=0` 表示不限制 |

每个轮子（`left_front_wheel`、`right_front_wheel`、`left_back_wheel`、`right_back_wheel`）
都有独立的 PID 参数：`p`、`i`、`d`、`i_clamp`、`antiwindup`。
p参数已经调好

## 参考资料
https://deepwiki.com/ros-controls/ros_control/5.1-creating-a-custom-controller
https://github.com/ros-controls/ros_controllers/blob/noetic-devel/effort_controllers/src/joint_velocity_controller.cpp


## License

本项目基于 [BSD 3-Clause License](LICENSE) 发布。

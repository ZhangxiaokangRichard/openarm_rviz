#ifndef FUEL_GUN_GRASP_GRASP_CONTROLLER_H
#define FUEL_GUN_GRASP_GRASP_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/KinematicsPose.h>

namespace fuel_gun_grasp
{

// ==============================================================================
// 抓取任务状态枚举
// 状态机从 STATE_INIT 出发，按顺序推进到 STATE_DONE
// ==============================================================================
enum GraspState
{
  STATE_INIT = 0,        // 初始化：等待控制器就绪
  STATE_HOME,            // 回零位：运动到安全的初始关节角
  STATE_PRE_GRASP,       // 预备位：移动到加油枪正上方
  STATE_OPEN_GRIPPER,    // 打开夹爪：为抓取做准备
  STATE_APPROACH,        // 接近：竖直下降到加油枪抓取位
  STATE_CLOSE_GRIPPER,   // 闭合夹爪：夹住加油枪
  STATE_LIFT,            // 抬升：抬起加油枪离开放置台
  STATE_MOVE_TARGET,     // 转移：携带加油枪运动到安装位上方
  STATE_PLACE,           // 放置：竖直下降到安装终点
  STATE_OPEN_RELEASE,    // 释放夹爪：松开加油枪完成安装
  STATE_DONE             // 完成：任务结束
};

// ==============================================================================
// GraspController 类
// 封装与控制器 ROS 服务的交互逻辑，以及任务状态机
// ==============================================================================
class GraspController
{
public:
  explicit GraspController(ros::NodeHandle& nh);

  // 运行主循环（阻塞）
  void run();

private:
  // ---------- 参数加载 ----------
  void loadParams();

  // ---------- 服务调用封装 ----------

  // 关节空间运动（目标关节角列表）
  bool moveJointSpace(const std::vector<double>& positions, double path_time);

  // 任务空间全位姿运动（位置 + 姿态）
  bool moveTaskSpace(const geometry_msgs::Pose& target_pose, double path_time);

  // 任务空间仅位置运动（保持当前姿态不变）
  bool moveTaskSpacePositionOnly(const geometry_msgs::Pose& target_pose, double path_time);

  // 夹爪控制
  bool controlGripper(double value, double path_time);

  // ---------- 状态机推进 ----------
  void advanceState();

  // ---------- 辅助函数 ----------

  // 将 rosparam 中的位姿数据读入 geometry_msgs::Pose
  geometry_msgs::Pose readPoseParam(const std::string& ns);

  // ---------- ROS 句柄 ----------
  ros::NodeHandle& nh_;

  // ---------- 服务客户端 ----------
  ros::ServiceClient client_joint_space_;        // /goal_joint_space_path
  ros::ServiceClient client_task_space_;         // /goal_task_space_path
  ros::ServiceClient client_task_pos_only_;      // /goal_task_space_path_position_only
  ros::ServiceClient client_tool_control_;       // /goal_tool_control

  // ---------- 状态机 ----------
  GraspState current_state_;   // 当前状态

  // ---------- 位姿参数（抓取目标） ----------
  geometry_msgs::Pose gun_pose_;      // 加油枪起点位姿
  geometry_msgs::Pose install_pose_;  // 安装终点位姿

  // ---------- 数值参数 ----------
  double pre_grasp_height_offset_;   // 预备位 Z 偏移 (m)
  double pre_place_height_offset_;   // 放置预备位 Z 偏移 (m)
  double lift_height_offset_;        // 抬升高度 (m)

  double gripper_open_value_;        // 夹爪打开值
  double gripper_grasp_value_;       // 夹爪抓取值

  double path_time_home_;            // 回零运动时间
  double path_time_transit_;         // 转移运动时间
  double path_time_approach_;        // 接近运动时间
  double path_time_lift_;            // 抬升运动时间
  double path_time_gripper_;         // 夹爪运动时间

  double wait_after_open_;           // 夹爪打开后等待时间
  double wait_after_grasp_;          // 夹爪闭合后等待时间
  double wait_after_lift_;           // 抬升后等待时间
  double wait_after_place_;          // 放置后等待时间

  std::vector<double> home_joint_positions_;  // 零位关节角
};

}  // namespace fuel_gun_grasp

#endif  // FUEL_GUN_GRASP_GRASP_CONTROLLER_H

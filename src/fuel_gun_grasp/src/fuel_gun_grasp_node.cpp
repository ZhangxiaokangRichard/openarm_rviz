/**
 * @file fuel_gun_grasp_node.cpp
 * @brief 加油枪自动抓取与安装功能包主节点
 *
 * 本节点实现一个 10 阶段状态机，驱动 OpenManipulator-X 机械臂执行以下流程：
 *   1. 等待控制器就绪 → 2. 回零位 → 3. 预备位 → 4. 打开夹爪
 *   → 5. 接近抓取 → 6. 夹住加油枪 → 7. 抬升 → 8. 移动到安装位上方
 *   → 9. 放置安装 → 10. 释放夹爪 → 完成
 *
 * 所有位姿参数通过 rosparam 配置（见 config/params.yaml）。
 * 运行前需确保 open_manipulator_controller 节点已经启动。
 */

#include <ros/ros.h>
#include <fuel_gun_grasp/grasp_controller.h>

// ==============================================================================
// GraspController 成员函数实现
// ==============================================================================

namespace fuel_gun_grasp
{

GraspController::GraspController(ros::NodeHandle& nh)
  : nh_(nh), current_state_(STATE_INIT)
{
  // 初始化服务客户端
  // 等待控制器服务上线（最多 10 秒，避免节点启动顺序问题）
  ROS_INFO("[FuelGunGrasp]connecting_service...");

  // 服务名使用绝对路径（加 /），避免私有命名空间 ~ 导致路径被解析为 /fuel_gun_grasp_node/...
  client_joint_space_ = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>(
      "/goal_joint_space_path");
  client_task_space_ = nh_.serviceClient<open_manipulator_msgs::SetKinematicsPose>(
      "/goal_task_space_path");
  client_task_pos_only_ = nh_.serviceClient<open_manipulator_msgs::SetKinematicsPose>(
      "/goal_task_space_path_position_only");
  client_tool_control_ = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>(
      "/goal_tool_control");

  // 加载 rosparam 参数（抓取任务参数）
  loadParams();

  ROS_INFO("[FuelGunGrasp] Initialization complete, ready to execute grasping task.");
}

// ------------------------------------------------------------------------------
// 参数加载
// ------------------------------------------------------------------------------
void GraspController::loadParams()
{
  // 读取加油枪起点位姿
  gun_pose_ = readPoseParam("gun_pose");

  // 读取安装终点位姿
  install_pose_ = readPoseParam("install_pose");

  // 预备位偏移量
  nh_.param("pre_grasp_height_offset", pre_grasp_height_offset_, 0.06);
  nh_.param("pre_place_height_offset", pre_place_height_offset_, 0.06);
  nh_.param("lift_height_offset",      lift_height_offset_,      0.08);

  // 夹爪参数
  nh_.param("gripper_open_value",  gripper_open_value_,  0.010);
  nh_.param("gripper_grasp_value", gripper_grasp_value_, -0.005);

  // 运动时间参数
  nh_.param("path_time_home",     path_time_home_,     2.0);
  nh_.param("path_time_transit",  path_time_transit_,  1.5);
  nh_.param("path_time_approach", path_time_approach_, 1.0);
  nh_.param("path_time_lift",     path_time_lift_,     1.0);
  nh_.param("path_time_gripper",  path_time_gripper_,  0.8);

  // 等待时间参数
  nh_.param("wait_after_open",  wait_after_open_,  1.0);
  nh_.param("wait_after_grasp", wait_after_grasp_, 1.5);
  nh_.param("wait_after_lift",  wait_after_lift_,  0.5);
  nh_.param("wait_after_place", wait_after_place_, 1.0);

  // 零位关节角（4 个关节）
  if (!nh_.getParam("home_joint_positions", home_joint_positions_))
  {
    // 参数未设置时使用默认值：竖直伸展姿态
    home_joint_positions_ = {0.0, -1.0472, 0.3491, 0.6981};
    ROS_WARN("[FuelGunGrasp] home_joint_positions not set, using default values.");
  }

  // 打印加载结果
  ROS_INFO("[FuelGunGrasp] Parameters loaded:");
  ROS_INFO("  Gun start pose: (%.3f, %.3f, %.3f)",
           gun_pose_.position.x, gun_pose_.position.y, gun_pose_.position.z);
  ROS_INFO("  Install end pose:   (%.3f, %.3f, %.3f)",
           install_pose_.position.x, install_pose_.position.y, install_pose_.position.z);
  ROS_INFO("  Pre-grasp height offset: %.3f m, Lift height: %.3f m",
           pre_grasp_height_offset_, lift_height_offset_);
  ROS_INFO("  Gripper: open=%.3f m, grasp=%.3f m",
           gripper_open_value_, gripper_grasp_value_);
}

// ------------------------------------------------------------------------------
// 辅助：从 rosparam 读取位姿（命名空间 ns 下的 position/orientation）
// ------------------------------------------------------------------------------
geometry_msgs::Pose GraspController::readPoseParam(const std::string& ns)
{
  geometry_msgs::Pose pose;

  nh_.param(ns + "/position/x", pose.position.x, 0.20);
  nh_.param(ns + "/position/y", pose.position.y, 0.00);
  nh_.param(ns + "/position/z", pose.position.z, 0.10);

  nh_.param(ns + "/orientation/x", pose.orientation.x, 0.0);
  nh_.param(ns + "/orientation/y", pose.orientation.y, 0.7071);
  nh_.param(ns + "/orientation/z", pose.orientation.z, 0.0);
  nh_.param(ns + "/orientation/w", pose.orientation.w, 0.7071);

  return pose;
}

// ------------------------------------------------------------------------------
// 服务调用：关节空间运动
// positions: 各关节目标角度列表 [joint1, joint2, joint3, joint4]
// path_time: 轨迹插值持续时间 (s)
// ------------------------------------------------------------------------------
bool GraspController::moveJointSpace(const std::vector<double>& positions, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.planning_group = "planning_group";
  srv.request.joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
  srv.request.joint_position.position   = positions;
  srv.request.path_time                 = path_time;

  if (!client_joint_space_.call(srv))
  {
    ROS_ERROR("[FuelGunGrasp] Failed to call goal_joint_space_path service!");
    return false;
  }
  if (!srv.response.is_planned)
  {
    ROS_ERROR("[FuelGunGrasp] goal_joint_space_path planning failed (is_planned=false)!");
    return false;
  }
  return true;
}

// ------------------------------------------------------------------------------
// 服务调用：任务空间全位姿运动（位置 + 姿态）
// target_pose: 目标位姿（geometry_msgs::Pose，使用四元数表示姿态）
// ------------------------------------------------------------------------------
bool GraspController::moveTaskSpace(const geometry_msgs::Pose& target_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group   = "planning_group";
  srv.request.end_effector_name = "gripper";
  srv.request.kinematics_pose.pose = target_pose;
  srv.request.path_time        = path_time;

  if (!client_task_space_.call(srv))
  {
    ROS_ERROR("[FuelGunGrasp] Failed to call goal_task_space_path service!");
    return false;
  }
  if (!srv.response.is_planned)
  {
    ROS_ERROR("[FuelGunGrasp] goal_task_space_path planning failed (is_planned=false)!");
    return false;
  }
  return true;
}

// ------------------------------------------------------------------------------
// 服务调用：任务空间仅位置运动（保持末端姿态不变，只改变位置）
// ------------------------------------------------------------------------------
bool GraspController::moveTaskSpacePositionOnly(const geometry_msgs::Pose& target_pose,
                                                double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group    = "planning_group";
  srv.request.end_effector_name = "gripper";
  srv.request.kinematics_pose.pose = target_pose;
  srv.request.path_time         = path_time;

  if (!client_task_pos_only_.call(srv))
  {
    ROS_ERROR("[FuelGunGrasp] Failed to call goal_task_space_path_position_only service!");
    return false;
  }
  if (!srv.response.is_planned)
  {
    ROS_ERROR("[FuelGunGrasp] goal_task_space_path_position_only planning failed (is_planned=false)!");
    return false;
  }
  return true;
}

// ------------------------------------------------------------------------------
// 服务调用：夹爪控制
// value: 夹爪目标位移 (m)，正值=张开，负值=收紧
// ------------------------------------------------------------------------------
bool GraspController::controlGripper(double value, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.planning_group = "planning_group";
  srv.request.joint_position.joint_name = {"gripper"};
  srv.request.joint_position.position   = {value};
  srv.request.path_time                 = path_time;

  if (!client_tool_control_.call(srv))
  {
    ROS_ERROR("[FuelGunGrasp] Failed to call goal_tool_control service!");
    return false;
  }
  if (!srv.response.is_planned)
  {
    ROS_ERROR("[FuelGunGrasp] goal_tool_control planning failed (is_planned=false)!");
    return false;
  }
  return true;
}

// ------------------------------------------------------------------------------
// 状态机推进核心逻辑
// 每次调用执行当前状态的动作，成功后切换到下一个状态
// ------------------------------------------------------------------------------
void GraspController::advanceState()
{
  switch (current_state_)
  {
    // ------------------------------------------------------------------
    case STATE_INIT:
    {
      // 等待服务上线（阻塞等待最多 10s）
      ROS_INFO("[FuelGunGrasp] [INIT] Waiting for controller services to be available...");
      bool ok = client_joint_space_.waitForExistence(ros::Duration(10.0)) &&
                client_task_space_.waitForExistence(ros::Duration(10.0)) &&
                client_task_pos_only_.waitForExistence(ros::Duration(10.0)) &&
                client_tool_control_.waitForExistence(ros::Duration(10.0));
      if (!ok)
      {
        ROS_ERROR("[FuelGunGrasp] Controller services timeout, please ensure open_manipulator_controller is running!");
        // 保持 INIT 状态，下次循环重试
        return;
      }
      ROS_INFO("[FuelGunGrasp] [INIT] Controller services are ready, starting grasp sequence.");
      current_state_ = STATE_HOME;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_HOME:
    {
      // 运动到安全零位，避免奇异或碰撞
      ROS_INFO("[FuelGunGrasp] [HOME] Moving to home position...");
      if (!moveJointSpace(home_joint_positions_, path_time_home_))
        return;  // 失败则保持当前状态，等待下次重试

      // 等待轨迹执行完毕（path_time + 余量）
      ros::Duration(path_time_home_ + 0.5).sleep();
      ROS_INFO("[FuelGunGrasp] [HOME] Reached home position.");
      current_state_ = STATE_PRE_GRASP;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_PRE_GRASP:
    {
      // 移动到加油枪正上方的预备位（Z + offset）
      // 使用 position_only：4-DoF 机械臂无法同时满足任意位置+完整四元数姿态约束，
      // position_only 仅约束末端位置，由 IK 自由选择合法姿态，避免 IK 无解
      ROS_INFO("[FuelGunGrasp] [PRE_GRASP] Moving to pre-grasp position (above fuel gun)...");
      geometry_msgs::Pose pre_grasp_pose = gun_pose_;
      pre_grasp_pose.position.z += pre_grasp_height_offset_;  // 抬高到枪的正上方

      if (!moveTaskSpacePositionOnly(pre_grasp_pose, path_time_transit_))
        return;

      ros::Duration(path_time_transit_ + 0.5).sleep();
      ROS_INFO("[FuelGunGrasp] [PRE_GRASP] Reached pre-grasp position.");
      current_state_ = STATE_OPEN_GRIPPER;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_OPEN_GRIPPER:
    {
      // 打开夹爪，为抓取做准备
      ROS_INFO("[FuelGunGrasp] [OPEN_GRIPPER] Opening gripper (%.3f m)...", gripper_open_value_);
      if (!controlGripper(gripper_open_value_, path_time_gripper_))
        return;

      ros::Duration(path_time_gripper_ + wait_after_open_).sleep();
      ROS_INFO("[FuelGunGrasp] [OPEN_GRIPPER] Gripper opened.");
      current_state_ = STATE_APPROACH;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_APPROACH:
    {
      // 从预备位竖直下降到加油枪抓取位（仅位置，保持姿态）
      ROS_INFO("[FuelGunGrasp] [APPROACH] Approaching fuel gun...");
      if (!moveTaskSpacePositionOnly(gun_pose_, path_time_approach_))
        return;

      ros::Duration(path_time_approach_ + 0.3).sleep();

      ROS_INFO("[FuelGunGrasp] [APPROACH] Reached grasp position.");
      current_state_ = STATE_CLOSE_GRIPPER;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_CLOSE_GRIPPER:
    {
      // 闭合夹爪，夹住加油枪
      ROS_INFO("[FuelGunGrasp] [CLOSE_GRIPPER] Closing gripper (%.3f m)...", gripper_grasp_value_);
      if (!controlGripper(gripper_grasp_value_, path_time_gripper_))
        return;

      // 等待夹爪稳定夹紧
      ros::Duration(path_time_gripper_ + wait_after_grasp_).sleep();
      ROS_INFO("[FuelGunGrasp] [CLOSE_GRIPPER] Gripper closed, fuel gun grasped.");
      current_state_ = STATE_LIFT;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_LIFT:
    {
      // 带着加油枪竖直抬升，离开放置台
      ROS_INFO("[FuelGunGrasp] [LIFT] Lifting fuel gun (%.3f m)...", lift_height_offset_);
      geometry_msgs::Pose lift_pose = gun_pose_;
      lift_pose.position.z += lift_height_offset_;  // 抬升后的高度

      if (!moveTaskSpacePositionOnly(lift_pose, path_time_lift_))
        return;

      ros::Duration(path_time_lift_ + wait_after_lift_).sleep();
      ROS_INFO("[FuelGunGrasp] [LIFT] Lift completed.");
      current_state_ = STATE_MOVE_TARGET;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_MOVE_TARGET:
    {
      // 携带加油枪运动到安装终点正上方的预备位
      // 同样使用 position_only，避免 4-DoF IK 姿态过约束失败
      ROS_INFO("[FuelGunGrasp] [MOVE_TARGET] Moving to pre-place position (above install point)...");
      geometry_msgs::Pose pre_place_pose = install_pose_;
      pre_place_pose.position.z += pre_place_height_offset_;  // 安装位正上方

      if (!moveTaskSpacePositionOnly(pre_place_pose, path_time_transit_))
        return;

      ros::Duration(path_time_transit_ + 0.5).sleep();
      ROS_INFO("[FuelGunGrasp] [MOVE_TARGET] Reached pre-place position.");
      current_state_ = STATE_PLACE;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_PLACE:
    {
      // 竖直下降，将加油枪插入/对准安装口
      ROS_INFO("[FuelGunGrasp] [PLACE] Descending to install point...");
      if (!moveTaskSpacePositionOnly(install_pose_, path_time_approach_))
        return;

      ros::Duration(path_time_approach_ + wait_after_place_).sleep();
      ROS_INFO("[FuelGunGrasp] [PLACE] Reached install point, fuel gun in position.");
      current_state_ = STATE_OPEN_RELEASE;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_OPEN_RELEASE:
    {
      // 松开夹爪，释放加油枪
      ROS_INFO("[FuelGunGrasp] [OPEN_RELEASE] Releasing gripper...");
      if (!controlGripper(gripper_open_value_, path_time_gripper_))
        return;

      ros::Duration(path_time_gripper_ + wait_after_open_).sleep();
      ROS_INFO("[FuelGunGrasp] [OPEN_RELEASE] Gripper released, fuel gun installed!");
      current_state_ = STATE_DONE;
      break;
    }

    // ------------------------------------------------------------------
    case STATE_DONE:
    {
      ROS_INFO("[FuelGunGrasp] ====================================");
      ROS_INFO("[FuelGunGrasp]  Fuel gun grasp and installation task completed!");
      ROS_INFO("[FuelGunGrasp] ====================================");
      // 任务已完成，不再推进
      break;
    }

    default:
      ROS_WARN("[FuelGunGrasp] Unknown state %d, resetting to initial state.", static_cast<int>(current_state_));
      current_state_ = STATE_INIT;
      break;
  }
}

// ------------------------------------------------------------------------------
// 主循环：在状态机完成前不断调用 advanceState()
// ------------------------------------------------------------------------------
void GraspController::run()
{
  ros::Rate rate(1.0);  // 1 Hz 轮询，状态机每秒尝试推进一次（实际由 sleep 控制节奏）

  while (ros::ok() && current_state_ != STATE_DONE)
  {
    ros::spinOnce();
    advanceState();
    rate.sleep();
  }

  // 最后一次推进 STATE_DONE 的日志输出
  if (current_state_ == STATE_DONE)
    advanceState();
}

}  // namespace fuel_gun_grasp

// ==============================================================================
// main 入口
// ==============================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fuel_gun_grasp_node");
  ros::NodeHandle nh("~");  // 私有命名空间，参数通过 <param> 或 rosparam 加载

  ROS_INFO("======================================");
  ROS_INFO("Fuel Gun Grasping Node Started");
  ROS_INFO("======================================");

  fuel_gun_grasp::GraspController controller(nh);
  controller.run();

  return 0;
}

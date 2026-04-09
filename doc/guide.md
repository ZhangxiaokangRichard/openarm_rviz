# OpenArm 工作空间开发指南

本文档面向在此 ROS Noetic 工作空间中进行 **OpenManipulator-X** 路径规划、RViz 仿真及 GUI 演示开发的工程师。

---

## 目录

1. [工作空间架构概览](#1-工作空间架构概览)
2. [构建与环境初始化](#2-构建与环境初始化)
3. [RViz 可视化仿真](#3-rviz-可视化仿真)
4. [Gazebo 仿真（含控制器）](#4-gazebo-仿真含控制器)
5. [路径规划功能包开发](#5-路径规划功能包开发)
6. [求解机械臂全部可行运动空间](#6-求解机械臂全部可行运动空间)
7. [GUI 与 RViz 联合动作演示](#7-gui-与-rviz-联合动作演示)
8. [关键 API 速查](#8-关键-api-速查)
9. [常见问题](#9-常见问题)

---

## 1. 工作空间架构概览

```
openarm/
├── build/                          # catkin 编译产物（自动生成）
├── devel/                          # catkin devel 空间（setup.bash 等）
├── doc/                            # 文档（本文件所在位置）
├── robotis_manipulator/            # ⚠️ 根目录下的冗余克隆（见第 9 节）
└── src/
    ├── CMakeLists.txt              # catkin 工作空间顶层 CMakeLists（符号链接）
    ├── open_manipulator/           # 主功能包组
    │   ├── open_manipulator/                 # 元包
    │   ├── open_manipulator_controller/      # 核心 ROS 控制节点（17 个服务）
    │   ├── open_manipulator_control_gui/     # Qt5 图形控制界面
    │   ├── open_manipulator_description/     # URDF/xacro + 网格 + RViz 配置
    │   ├── open_manipulator_libs/            # 机器人特定 C++ 库（运动学、关节链、执行器）
    │   └── open_manipulator_teleop/          # 键盘 / 手柄遥操作
    ├── open_manipulator_msgs/      # 自定义消息与服务定义
    ├── open_manipulator_simulations/
    │   ├── open_manipulator_gazebo/          # Gazebo 仿真配置、控制器 YAML、世界文件
    │   └── open_manipulator_simulations/     # 元包
    ├── open_manipulator_dependencies/
    │   └── roboticsgroup_gazebo_plugins/     # Gazebo mimic 关节插件
    └── robotis_manipulator/        # ✅ 实际参与编译的核心运动学/轨迹库
```

### 依赖层次

```
roboticsgroup_gazebo_plugins   ← Gazebo mimic joint 插件
open_manipulator_description   ← URDF/xacro/meshes
robotis_manipulator            ← 数学、运动学框架、轨迹生成（MinimumJerk）
        ↓
open_manipulator_libs          ← 关节链配置、IK 求解器、Dynamixel 执行器、自定义轨迹
        ↓
open_manipulator_controller    ← ROS 节点，暴露 17 个 Service
       ↙              ↘
open_manipulator_teleop    open_manipulator_control_gui
open_manipulator_gazebo    open_manipulator_msgs
```

### 机器人关节参数速查

| 关节 | 类型 | 轴 | 范围 |
|------|------|----|------|
| joint1 | revolute | Z | ±162°（±2.83 rad） |
| joint2 | revolute | Y | -103° ~ +90° |
| joint3 | revolute | Y | -54° ~ +79° |
| joint4 | revolute | Y | -103° ~ +117° |
| gripper | prismatic | Y | -10 mm ~ +19 mm |

末端执行器（end_effector_link）固定在 link5 前方 126 mm 处。

---

## 2. 构建与环境初始化

```bash
cd ~/Desktop/openarm
catkin_make                   # 首次或修改后重新编译
source devel/setup.bash       # 每个终端都需要执行
```

> **建议**：将 `source ~/Desktop/openarm/devel/setup.bash` 加入 `~/.bashrc`，避免每次手动 source。

---

## 3. RViz 可视化仿真

有两种 RViz 使用方式，按需选择。

### 3.1 纯 RViz（关节滑块手动交互，无控制器）

只需可视化模型、手动拖动关节到各姿态时使用：

```bash
roslaunch open_manipulator_description open_manipulator_rviz.launch
```

此 launch 启动 `joint_state_publisher`（带 GUI 滑块）、`robot_state_publisher` 和 RViz。
**不支持路径规划服务调用，仅供手动浏览关节空间。**

### 3.2 RViz 轻量仿真模式（推荐，含完整控制器）

不需要 Gazebo，控制器直接向 RViz 的 `robot_state_publisher` 发布关节状态，支持全部 17 个规划服务和 GUI 控制：

```bash
# 一条命令启动全部（RViz + 控制器）
roslaunch open_manipulator_description open_manipulator_rviz_sim.launch
```

> **原理说明**
>
> 原始控制器在 `use_platform:=false` 时只发布 `std_msgs/Float64` 的 `joint1_position/command` 等话题（供 Gazebo ros_control 使用），并不发布 `/joint_states`，因此 RViz 无法看到运动。
> 本工作空间已修改控制器，新增 `use_rviz` 参数：当 `use_rviz:=true` 时，控制器改为直接发布 `sensor_msgs/JointState`（含全部关节及夹爪 mimic 关节 `gripper_sub`），`robot_state_publisher` 直接订阅并更新 RViz 中的模型，不需要 Gazebo。

**如需同时启动 GUI：**

```bash
roslaunch open_manipulator_description open_manipulator_rviz_sim_gui.launch
```

**新增 launch 文件位置：**

| Launch 文件 | 说明 |
|-------------|------|
| `open_manipulator_description/launch/open_manipulator_rviz_sim.launch` | RViz + 控制器（无 Gazebo） |
| `open_manipulator_description/launch/open_manipulator_rviz_sim_gui.launch` | RViz + 控制器 + Qt5 GUI |

---

## 4. Gazebo 仿真（含控制器）

```bash
# 一步启动 Gazebo 完整仿真（含 Gazebo 世界、URDF spawn、控制器、controller 节点）
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```

Gazebo launch 内部依次完成：
1. 启动空白 Gazebo 世界（`worlds/empty.world`）
2. 通过 xacro 解析 `open_manipulator_robot.urdf.xacro` 并 spawn 模型
3. 加载 `arm_controller.yaml`（joints 1-4，位置控制）、`gripper_controller.yaml`（effort 控制，P=100）
4. 启动 `joint_state_controller`
5. 启动 `open_manipulator_controller`（`use_platform:=false`）
6. 启动 `omx_gripper_sub_publisher`（处理 mimic 关节镜像）

---

## 5. 路径规划功能包开发

### 5.1 创建新功能包

```bash
cd ~/Desktop/openarm/src
catkin_create_pkg my_arm_planner roscpp rospy std_msgs \
    open_manipulator_msgs geometry_msgs
cd my_arm_planner
mkdir launch scripts
```

在 `package.xml` 中确认依赖：

```xml
<depend>open_manipulator_msgs</depend>
<depend>roscpp</depend>
```

### 5.2 控制器提供的规划服务

控制器节点（`/open_manipulator_controller`）提供以下服务，路径规划器通过 ROS 服务调用即可驱动机器人：

| 服务名 | 类型 | 用途 |
|--------|------|------|
| `/goal_joint_space_path` | `SetJointPosition` | 关节空间绝对位置运动 |
| `/goal_task_space_path` | `SetKinematicsPose` | 笛卡尔空间绝对位姿运动 |
| `/goal_joint_space_path_from_present` | `SetJointPosition` | 关节空间相对运动 |
| `/goal_task_space_path_from_present_position_only` | `SetKinematicsPose` | 仅平移相对运动 |
| `/goal_task_space_path_from_present_orientation_only` | `SetKinematicsPose` | 仅旋转相对运动 |
| `/goal_drawing_trajectory` | `SetDrawingTrajectory` | 内置轨迹（Line/Circle/Rhombus/Heart） |
| `/goal_tool_control` | `SetJointPosition` | 夹爪开合 |
| `/get_joint_position` | `GetJointPosition` | 查询当前关节角 |
| `/get_kinematics_pose` | `GetKinematicsPose` | 查询当前末端位姿 |

### 5.3 示例：C++ Service Client 发送关节轨迹

```cpp
#include <ros/ros.h>
#include <open_manipulator_msgs/SetJointPosition.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_planner");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(
        "/goal_joint_space_path");
    ros::service::waitForService("/goal_joint_space_path");

    open_manipulator_msgs::SetJointPosition srv;
    srv.request.planning_group = "planning_group";
    srv.request.joint_position.joint_name = {"joint1","joint2","joint3","joint4"};
    srv.request.joint_position.position    = {0.0, -1.05, 0.35, 0.70};
    srv.request.path_time = 2.0;   // 运动时间（秒），影响轨迹平滑度

    if (client.call(srv) && srv.response.is_planned)
        ROS_INFO("Trajectory planned and executing.");
    return 0;
}
```

### 5.4 示例：Python Service Client 发送末端位姿

```python
#!/usr/bin/env python3
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose

rospy.init_node("task_space_mover")
rospy.wait_for_service("/goal_task_space_path")
client = rospy.ServiceProxy("/goal_task_space_path", SetKinematicsPose)

req = SetKinematicsPoseRequest()
req.planning_group = "planning_group"
req.end_effector_name = "gripper"
req.kinematics_pose.pose.position.x = 0.286
req.kinematics_pose.pose.position.y = 0.0
req.kinematics_pose.pose.position.z = 0.204
req.kinematics_pose.pose.orientation.w = 1.0
req.path_time = 2.0

resp = client(req)
print("Planned:", resp.is_planned)
```

### 5.5 轨迹生成原理

工作空间内置两类轨迹生成器（位于 `robotis_manipulator` 库）：

- **关节空间轨迹** (`JointTrajectory`)：五阶多项式（MinimumJerk），零起始/终止速度与加速度，保证平滑。
- **笛卡尔空间轨迹** (`TaskTrajectory`)：同样五阶多项式，对末端位置和姿态（RPY）各自独立插值。
- **自定义轨迹**：`Line`（梯形速度剖面）、`Circle`、`Rhombus`、`Heart`，通过 `/goal_drawing_trajectory` 触发。

控制器以 **100 Hz**（`control_period = 0.01s`）采样轨迹并发送给执行器。

---

## 6. 求解机械臂全部可行运动空间

### 6.1 工作空间定义

**关节空间（C-space）**的 4 维盒形约束（单位：弧度）：

```
joint1 ∈ [-2.827, +2.827]
joint2 ∈ [-1.796, +1.571]
joint3 ∈ [-0.942, +1.379]
joint4 ∈ [-1.796, +2.042]
```

**任务空间（笛卡尔工作空间）**需通过正运动学采样得到。

### 6.2 正运动学采样（求解可达工作空间）

在你的功能包中编写如下 Python 脚本，通过调用控制器的 FK 或直接使用 `open_manipulator_libs` 中的运动学类进行网格采样：

```python
#!/usr/bin/env python3
"""
workspace_sampler.py
通过蒙特卡洛随机采样 + 正运动学，可视化 OpenManipulator-X 可达工作空间。
运行前确保已 source devel/setup.bash。
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 关节限位（rad）
LIMITS = [
    (-2.827, 2.827),   # joint1
    (-1.796, 1.571),   # joint2
    (-0.942, 1.379),   # joint3
    (-1.796, 2.042),   # joint4
]

# DH 参数（来自 URDF，单位：米）
# 连杆长度：link1_z=0.077, link2=0.130, link3=0.124, link4=0.126（end-effector offset）
def fk(q):
    """简化正运动学，返回末端执行器 (x, y, z)"""
    c, s = np.cos, np.sin
    q1, q2, q3, q4 = q

    # base → link1
    T = np.array([
        [c(q1), -s(q1), 0, 0],
        [s(q1),  c(q1), 0, 0],
        [0,      0,     1, 0.077],
        [0,      0,     0, 1   ]
    ])
    # link1 → link2 (Y轴旋转)
    def ry(a, dx, dz):
        return np.array([
            [ c(a), 0, s(a), dx],
            [ 0,    1, 0,    0 ],
            [-s(a), 0, c(a), dz],
            [ 0,    0, 0,    1 ]
        ])
    T = T @ ry(q2, 0, 0) @ np.array([[1,0,0,0.130],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    T = T @ ry(q3, 0, 0) @ np.array([[1,0,0,0.124],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    T = T @ ry(q4, 0, 0) @ np.array([[1,0,0,0.126],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    return T[:3, 3]

N = 200_000   # 采样点数
points = []
rng = np.random.default_rng(42)

for _ in range(N):
    q = [rng.uniform(lo, hi) for lo, hi in LIMITS]
    points.append(fk(q))

pts = np.array(points)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(pts[::20, 0], pts[::20, 1], pts[::20, 2], s=0.3, alpha=0.3, c=pts[::20, 2], cmap='viridis')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
ax.set_title('OpenManipulator-X 可达工作空间（蒙特卡洛采样）')
plt.tight_layout()
plt.savefig('workspace.png', dpi=150)
plt.show()
```

运行方式：

```bash
source ~/Desktop/openarm/devel/setup.bash
python3 src/my_arm_planner/scripts/workspace_sampler.py
```

### 6.3 通过 RViz 交互式探索工作空间

```bash
# 启动 RViz + joint_state_publisher_gui（带滑块）
roslaunch open_manipulator_description open_manipulator_rviz.launch
```

拖动 GUI 滑块即可实时预览各关节组合下机械臂的姿态，直观感受工作空间边界。

### 6.4 利用 IK 验证可达性

调用控制器 IK 服务验证给定笛卡尔目标是否可达：

```python
from open_manipulator_msgs.srv import GetKinematicsPose, SetKinematicsPose

# 先查询当前末端位姿
get_pose = rospy.ServiceProxy("/get_kinematics_pose", GetKinematicsPose)
# 再尝试规划目标位姿，如 is_planned=False 则位于工作空间外
plan = rospy.ServiceProxy("/goal_task_space_path", SetKinematicsPose)
```

---

## 7. GUI 与 RViz 联合动作演示

### 7.1 完整演示启动顺序

**方案一：Gazebo + GUI（含物理仿真）**

```bash
# 终端 1：启动 Gazebo 完整仿真
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch

# 终端 2（可选，额外 RViz 窗口）
roslaunch open_manipulator_description open_manipulator_rviz.launch

# 终端 3：启动 Qt5 GUI 控制面板
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```

**方案二：纯 RViz 轻量仿真 + GUI（推荐，无需 Gazebo）**

```bash
# 一条命令启动全部（RViz + 控制器 + Qt5 GUI）
roslaunch open_manipulator_description open_manipulator_rviz_sim_gui.launch
```

或分终端启动（便于查看各节点日志）：

```bash
# 终端 1：RViz + 控制器
roslaunch open_manipulator_description open_manipulator_rviz_sim.launch

# 终端 2：Qt5 GUI
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```

### 7.2 GUI 功能说明

Qt5 GUI（`open_manipulator_control_gui`）提供：
- **Timer 启动/停止**：控制状态订阅频率
- **Actuator 使能/失能**：调用 `/set_actuator_state`
- **关节控制标签页**：输入各关节目标角度 + path_time，调用 `/goal_joint_space_path`
- **任务空间标签页**：输入末端 (x, y, z, roll, pitch, yaw) + path_time，调用 `/goal_task_space_path`
- **夹爪控制**：开/关夹爪，调用 `/goal_tool_control`
- **Drawing 轨迹**：一键触发 Line / Circle / Rhombus / Heart，调用 `/goal_drawing_trajectory`
- **状态显示**：实时显示关节角、末端位姿、机器人状态

### 7.3 内置动作演示（Drawing Trajectory）

通过服务调用触发内置演示动作：

```bash
# 圆形轨迹：半径 0.05m，走 1 圈，耗时 4s
rosservice call /goal_drawing_trajectory \
    "end_effector_name: 'gripper'
     drawing_trajectory_name: 'circle'
     param: [0.05, 1.0, 0.0]
     path_time: 4.0"

# 直线轨迹：沿 X 延伸 0.1m，耗时 2s
rosservice call /goal_drawing_trajectory \
    "end_effector_name: 'gripper'
     drawing_trajectory_name: 'line'
     param: [0.1, 0.0, 0.0]
     path_time: 2.0"

# 心形轨迹
rosservice call /goal_drawing_trajectory \
    "end_effector_name: 'gripper'
     drawing_trajectory_name: 'heart'
     param: [0.05, 1.0, 0.0]
     path_time: 6.0"
```

### 7.4 键盘遥操作

```bash
# 需控制器已运行
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```

键盘控制说明（运行后终端内显示帮助）：`W/A/S/D` 控制末端平移，`Z/X` 控制夹爪开合，`Q/E` 改变 path_time。

---

## 8. 关键 API 速查

### 消息类型

```
open_manipulator_msgs/JointPosition
    string[] joint_name
    float64[] position
    float64[] velocity      # 通常不用，留空
    float64[] effort        # 通常不用，留空

open_manipulator_msgs/KinematicsPose
    geometry_msgs/Pose pose     # position (x,y,z) + orientation (quaternion)
    float64 max_velocity_scaling_factor     # 0.0~1.0（可选）
    float64 max_acceleration_scaling_factor # 0.0~1.0（可选）
```

### 服务类型

```
open_manipulator_msgs/SetJointPosition
    Request:  string planning_group, JointPosition joint_position, float64 path_time
    Response: bool is_planned

open_manipulator_msgs/SetKinematicsPose
    Request:  string planning_group, string end_effector_name,
              KinematicsPose kinematics_pose, float64 path_time
    Response: bool is_planned

open_manipulator_msgs/SetDrawingTrajectory
    Request:  string end_effector_name, string drawing_trajectory_name,
              float64[] param, float64 path_time
    Response: bool is_planned
```

### 话题

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/open_manipulator/joint_states` | `sensor_msgs/JointState` | 发布 | 当前关节状态 |
| `/open_manipulator/kinematics_pose` | `open_manipulator_msgs/KinematicsPose` | 发布 | 当前末端位姿 |
| `/open_manipulator/open_manipulator_state` | `open_manipulator_msgs/OpenManipulatorState` | 发布 | 机器人 & 执行器状态 |
| `/joint_states` | `sensor_msgs/JointState` | 发布 | RViz/robot_state_publisher 用 |

---

## 9. 常见问题

### Q1：根目录下的 `robotis_manipulator/` 能否删除？

**可以安全删除。**

该目录（`~/Desktop/openarm/robotis_manipulator/`）是在 `src/` 之外独立克隆的副本，**不在 catkin 工作空间内**，catkin 不会编译它。实际参与编译的是 `src/robotis_manipulator/`。两者内容完全相同（同一仓库同一分支 `noetic`）。

```bash
# 验证两者内容一致（应无输出）
diff -rq --exclude='.git' \
    ~/Desktop/openarm/robotis_manipulator/ \
    ~/Desktop/openarm/src/robotis_manipulator/

# 确认无误后删除根目录冗余副本
rm -rf ~/Desktop/openarm/robotis_manipulator/
```

### Q2：`use_platform:=false` 启动后 RViz 无运动反应

**原因**：官方控制器在 `use_platform:=false` 时进入 Gazebo 模式，只发布 `std_msgs/Float64` 的 `joint1_position/command` 等话题（Gazebo ros_control 专用），不发布 RViz 所需的 `sensor_msgs/JointState`。

**正确做法**：使用本工作空间新增的 `use_rviz` 参数，该参数已在控制器中实现：

```bash
# 方式 1（推荐）：一键启动
roslaunch open_manipulator_description open_manipulator_rviz_sim.launch

# 方式 2：手动传参
roslaunch open_manipulator_controller open_manipulator_controller.launch \
    use_platform:=false use_rviz:=true
```

> `use_platform:=false use_rviz:=true` 时，控制器同时发布 `sensor_msgs/JointState`（含 `gripper_sub` mimic 关节），`robot_state_publisher` 直接订阅，无需 Gazebo。

### Q3：IK 求解失败（`is_planned: false`）

目标位姿超出工作空间。检查：
- 目标距底座约 120~450 mm
- Z 高度不低于 50 mm（避免碰底）
- 增大 `path_time`（≥ 1.0s）有时可帮助求解器收敛

### Q4：Gazebo 中夹爪不动

夹爪使用 effort 控制器（PID P=100），确保 `gripper_controller.yaml` 已正确加载，并检查 `omx_gripper_sub_publisher` 节点是否运行。

### Q5：如何添加 MoveIt 支持？

本工作空间不包含 MoveIt 配置。如需 MoveIt，参考官方 `open_manipulator_with_tb3` 或使用 MoveIt Setup Assistant 为 URDF 生成配置包，并将规划结果通过上述服务接口发送。

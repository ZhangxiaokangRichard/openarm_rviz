# OpenArm — OpenManipulator-X RViz Simulation Workspace

A ROS Noetic catkin workspace for simulating and developing path planning with the **ROBOTIS OpenManipulator-X** (4-DOF + parallel-jaw gripper), using RViz as the visualization backend — no physical hardware or Gazebo required.

Based on the official [ROBOTIS open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) repository, extended with a lightweight RViz-only simulation mode.

---

## 环境要求 / Requirements

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 20.04 |
| ROS | Noetic Ninjemys |
| C++ | C++14 |
| CMake | ≥ 3.0.2 |

### ROS 依赖包

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-ros-base \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-urdf \
    ros-noetic-controller-manager \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-position-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-dynamixel-workbench-toolbox \
    ros-noetic-qt-build \
    qtbase5-dev libqt5-dev \
    python3-catkin-tools \
    libeigen3-dev
```

---

## 编译 / Build

```bash
cd ~/Desktop/openarm          # 或你克隆的路径
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

> 建议将 `source ~/Desktop/openarm/devel/setup.bash` 加入 `~/.bashrc`，避免每次手动执行。

---

## 使用方法 / Usage

### 1. RViz 轻量仿真（推荐，无需硬件和 Gazebo）

```bash
# 启动 RViz + 控制器（完整路径规划回路）
roslaunch open_manipulator_description open_manipulator_rviz_sim.launch

# 启动 RViz + 控制器 + Qt5 GUI（一键演示）
roslaunch open_manipulator_description open_manipulator_rviz_sim_gui.launch
```

GUI 弹出后，点击 **Timer ON** 连接控制器，即可通过界面驱动机械臂运动并在 RViz 中实时看到。

### 2. 仅 RViz（关节滑块手动交互）

```bash
roslaunch open_manipulator_description open_manipulator_rviz.launch
```

### 3. Gazebo 物理仿真

```bash
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```

### 4. 路径规划服务调用示例

控制器暴露 17 个 ROS 服务，路径规划节点通过服务调用驱动机器人：

```bash
# 关节空间运动
rosservice call /goal_joint_space_path \
    "planning_group: 'planning_group'
     joint_position: {joint_name: ['joint1','joint2','joint3','joint4'], position: [0.0,-1.05,0.35,0.70]}
     path_time: 2.0"

# 内置圆形轨迹
rosservice call /goal_drawing_trajectory \
    "end_effector_name: 'gripper'
     drawing_trajectory_name: 'circle'
     param: [0.05, 1.0, 0.0]
     path_time: 4.0"
```

---

## 工程结构 / Project Structure

```
openarm/
├── doc/
│   └── guide.md              # 详细开发指南（路径规划、工作空间分析、API 速查）
└── src/
    ├── open_manipulator/
    │   ├── open_manipulator_controller/   # 核心 ROS 控制节点（已修改，支持 use_rviz 参数）
    │   ├── open_manipulator_control_gui/  # Qt5 图形控制界面
    │   ├── open_manipulator_description/  # URDF/xacro + RViz 配置（含新增 launch 文件）
    │   ├── open_manipulator_libs/         # 机器人运动学库、IK 求解器
    │   └── open_manipulator_teleop/       # 键盘 / 手柄遥操作
    ├── open_manipulator_msgs/             # 自定义 ROS 消息 & 服务
    ├── open_manipulator_simulations/      # Gazebo 仿真配置
    ├── open_manipulator_dependencies/     # roboticsgroup Gazebo 插件
    └── robotis_manipulator/               # 核心运动学/轨迹生成库（MinimumJerk）
```

---

## 核心改动说明 / Key Modifications

相较于官方 open_manipulator 仓库，本项目对控制器做了如下修改：

**`open_manipulator_controller`**：新增 `use_rviz` 参数。

- 官方 `use_platform:=false` 仅向 Gazebo ros_control 发布 `std_msgs/Float64` 关节命令，无法驱动 RViz。
- 本项目新增 `use_rviz:=true` 模式，控制器直接发布 `sensor_msgs/JointState`（含 `gripper_sub` mimic 关节），`robot_state_publisher` 订阅后实时更新 RViz 模型，无需 Gazebo。

新增 launch 文件：
- `open_manipulator_description/launch/open_manipulator_rviz_sim.launch`
- `open_manipulator_description/launch/open_manipulator_rviz_sim_gui.launch`

---

## 文档 / Documentation

详细开发指南（路径规划包开发、运动空间求解、API 速查、常见问题）：

```
doc/guide.md
```

---

## 致谢 / Credits

- [ROBOTIS open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [ROBOTIS robotis_manipulator](https://github.com/ROBOTIS-GIT/robotis_manipulator)
- [roboticsgroup_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins)

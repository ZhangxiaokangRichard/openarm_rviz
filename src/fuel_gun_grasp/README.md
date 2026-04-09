# fuel_gun_grasp — 加油枪自动抓取功能包

基于 OpenManipulator-X 机械臂的加油枪抓取与安装自动化功能包。

---

## 功能概述

本功能包实现一个 **10 阶段状态机**，驱动机械臂完成从拾取加油枪到在指定位置安装的完整流程：

```
初始化 → 回零位 → 预备位（枪上方）→ 打开夹爪 → 接近抓取
       → 夹住加油枪 → 抬升 → 移至安装位上方 → 下降放置 → 释放夹爪 → 完成
```

所有起始位姿、终点位姿、夹爪参数、运动时间均通过 `config/params.yaml` 配置，无需修改源码。

---

## 包结构

```
fuel_gun_grasp/
├── config/
│   └── params.yaml                   # 所有运行参数（含中文注释）
├── include/
│   └── fuel_gun_grasp/
│       └── grasp_controller.h        # 状态机与服务调用封装头文件
├── launch/
│   ├── fuel_gun_grasp.launch         # 仅启动抓取节点（需控制器已运行）
│   └── sim_with_grasp.launch         # 一键启动 RViz 仿真 + 抓取节点
├── src/
│   └── fuel_gun_grasp_node.cpp       # 主节点：状态机实现
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 参数说明

所有参数位于 `config/params.yaml`，通过 `rosparam` 加载到节点私有命名空间。

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `gun_pose/position/{x,y,z}` | `(0.20, 0.00, 0.10)` | 加油枪起点位置 (m) |
| `gun_pose/orientation/{x,y,z,w}` | `(0,0.7071,0,0.7071)` | 抓取姿态（四元数） |
| `install_pose/position/{x,y,z}` | `(0.25, 0.10, 0.15)` | 安装终点位置 (m) |
| `install_pose/orientation/{x,y,z,w}` | `(0,0.7071,0,0.7071)` | 安装姿态（四元数） |
| `pre_grasp_height_offset` | `0.06` | 预备位相对抓取点的 Z 偏移 (m) |
| `pre_place_height_offset` | `0.06` | 安装预备位相对终点的 Z 偏移 (m) |
| `lift_height_offset` | `0.08` | 抓取后抬升高度 (m) |
| `gripper_open_value` | `0.010` | 夹爪打开位移 (m)，\> 0 |
| `gripper_grasp_value` | `-0.005` | 夹爪抓取位移 (m)，< 0 |
| `path_time_home` | `2.0` | 回零位运动时间 (s) |
| `path_time_transit` | `1.5` | 空间转移运动时间 (s) |
| `path_time_approach` | `1.0` | 垂直接近/放置时间 (s) |
| `path_time_lift` | `1.0` | 抬升时间 (s) |
| `path_time_gripper` | `0.8` | 夹爪开合时间 (s) |
| `wait_after_open` | `1.0` | 夹爪打开后额外等待 (s) |
| `wait_after_grasp` | `1.5` | 夹爪闭合后额外等待 (s) |
| `wait_after_lift` | `0.5` | 抬升后额外等待 (s) |
| `wait_after_place` | `1.0` | 放置后额外等待 (s) |
| `home_joint_positions` | `[0, -1.047, 0.349, 0.698]` | 零位关节角 (rad) |

---

## 依赖服务

本节点依赖 `open_manipulator_controller` 提供以下 ROS 服务：

| 服务名 | 类型 | 用途 |
|--------|------|------|
| `/goal_joint_space_path` | `SetJointPosition` | 回零位关节运动 |
| `/goal_task_space_path` | `SetKinematicsPose` | 预备位/转移全位姿运动 |
| `/goal_task_space_path_position_only` | `SetKinematicsPose` | 接近/抬升/放置仅位置运动 |
| `/goal_tool_control` | `SetJointPosition` | 夹爪开合控制 |

---

## 调用方式

### 方式一：单独启动（控制器已运行）

```bash
# 终端 1：启动 RViz 仿真控制器
roslaunch open_manipulator_description open_manipulator_rviz_sim.launch

# 终端 2：启动抓取节点
roslaunch fuel_gun_grasp fuel_gun_grasp.launch
```

### 方式二：一键启动（仿真 + 抓取）

```bash
roslaunch fuel_gun_grasp sim_with_grasp.launch
```

> 一键启动模式会在控制器启动后延迟 5 秒再运行抓取节点，确保服务就绪。

---

## 自定义抓取目标

修改 `config/params.yaml` 中的 `gun_pose` 和 `install_pose`：

```yaml
# 修改加油枪起点位置（单位 m，基坐标系）
gun_pose:
  position:
    x: 0.23    # 向前移动 3 cm
    y: 0.05    # 向左移动 5 cm
    z: 0.08    # 高度调低 2 cm
  orientation:
    x: 0.0
    y: 0.7071
    z: 0.0
    w: 0.7071

# 修改安装终点位置
install_pose:
  position:
    x: 0.28
    y: 0.00
    z: 0.20
```

修改后无需重新编译，直接重启节点即可生效。

---

## 状态机流程图

```
[INIT]
  │ 等待控制器服务上线
  ▼
[HOME]
  │ 关节空间运动到零位
  ▼
[PRE_GRASP]
  │ 任务空间全位姿运动到枪的上方
  ▼
[OPEN_GRIPPER]
  │ 夹爪打开
  ▼
[APPROACH]
  │ 仅位置运动：竖直下降到枪
  ▼
[CLOSE_GRIPPER]
  │ 夹爪闭合抓取
  ▼
[LIFT]
  │ 仅位置运动：竖直抬升
  ▼
[MOVE_TARGET]
  │ 任务空间全位姿运动到安装位上方
  ▼
[PLACE]
  │ 仅位置运动：竖直下降到安装位
  ▼
[OPEN_RELEASE]
  │ 夹爪打开释放
  ▼
[DONE]
  任务完成
```

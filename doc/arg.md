# OpenManipulator-X 运动规划架构深度解析

本文档从 ROS Service 层到底层数学原理，逐层剖析控制器的运动规划机制，并指导加油枪抓取功能包的设计与动作空间分析包的搭建。

---

## 目录

1. [关节位置服务的构成与传递链](#1-关节位置服务的构成与传递链)
2. [末端位姿服务与逆运动学](#2-末端位姿服务与逆运动学)
3. [路径执行：MinimumJerk 算法原理](#3-路径执行minimumjerk-算法原理)
4. [加油枪抓取功能包接口设计](#4-加油枪抓取功能包接口设计)
5. [动作空间分析功能包设计](#5-动作空间分析功能包设计)

---

## 1. 关节位置服务的构成与传递链

### 1.1 服务定义

```
服务名：/goal_joint_space_path
类型：  open_manipulator_msgs/SetJointPosition

Request:
  string   planning_group        # 规划组名，填 "planning_group"
  JointPosition joint_position
    string[] joint_name          # ["joint1","joint2","joint3","joint4"]
    float64[] position           # 目标角度（单位：rad）
    float64[] velocity           # 通常留空（使用轨迹规划）
    float64[] effort             # 通常留空
  float64  path_time             # 运动时长（秒）

Response:
  bool is_planned                # true=已接受规划；false=规划失败
```

### 1.2 控制器处理链（逐级传递路径）

```
ROS Client
  │
  │  call /goal_joint_space_path
  ▼
OpenManipulatorController::goalJointSpacePathCallback()
  │  从 req.joint_position.position 提取目标角度 vector<double>
  │
  ▼
RobotisManipulator::makeJointTrajectory(target_angle, path_time)
  │  1. 读取当前关节状态（trajectory_.getPresentJointWaypoint()）
  │  2. 设置轨迹总时间（trajectory_.setMoveTime(path_time)）
  │  3. 记录轨迹开始时刻（trajectory_.setStartTimeToPresentTime()）
  │  4. 调用 trajectory_.makeJointTrajectory(start, goal)
  │     └─ JointTrajectory::makeJointTrajectory()
  │         └─ 对每个关节，调用 MinimumJerk::calcCoefficient()
  │             计算 6 个多项式系数，存入 minimum_jerk_coefficient_(6×N)
  │
  ▼
timerThread（100 Hz，独立 POSIX 线程）
  │  每 10ms 调用 controller_->process(time)
  │
  ▼
processOpenManipulator(present_time)
  │  1. goal = getJointGoalValueFromTrajectory(present_time)
  │     └─ tick = present_time - start_time
  │        若 tick < path_time：对每关节求 θ(tick) = Σ aₙ tⁿ
  │        若 tick ≥ path_time：返回 goal（保持末态）
  │  2. sendAllJointActuatorValue(goal)
  │     硬件模式：串口指令→Dynamixel 舵机
  │     仿真模式：更新内部 Manipulator 状态
  │  3. receiveAllJointActuatorValue()
  │     硬件模式：读取舵机实际反馈
  │     仿真模式：no-op，内部状态保持计划值
  │  4. solveForwardKinematics()
  │     由当前关节角计算末端位姿（递推，见第 2 节）
  │
  ▼
publishCallback（ROS Timer，100 Hz）
  └─ publishJointStates()
      发布 sensor_msgs/JointState → /joint_states
      robot_state_publisher 订阅后更新 TF → RViz 渲染
```

### 1.3 关键数据结构

```cpp
// 每个关节的状态
struct JointValue {
    double position;     // rad
    double velocity;     // rad/s
    double acceleration; // rad/s²
    double effort;       // Nm
};
// 关节轨迹 = N 个关节各自的 MinimumJerk 多项式系数矩阵 (6×N)
Eigen::MatrixXd minimum_jerk_coefficient_;  // 行: a₀..a₅, 列: joint0..jointN
```

---

## 2. 末端位姿服务与逆运动学

### 2.1 服务定义

```
服务名：/goal_task_space_path
类型：  open_manipulator_msgs/SetKinematicsPose

Request:
  string   planning_group
  string   end_effector_name    # "gripper"
  KinematicsPose kinematics_pose
    geometry_msgs/Pose pose
      Point position            # x, y, z（单位：m）
      Quaternion orientation    # w, x, y, z（单位制）
    float64 max_velocity_scaling_factor     # 可选 0~1
    float64 max_acceleration_scaling_factor # 可选 0~1
  float64  path_time

Response:
  bool is_planned
```

### 2.2 末端位姿的传递链

```
goalTaskSpacePathCallback()
  │  提取目标位置 (x,y,z) 和旋转矩阵（由四元数转换）
  │
  ▼
makeTaskTrajectory(end_effector_name, target_pose, path_time)
  │  1. 获取当前末端位姿（已由上一次 FK 更新）
  │  2. 对 [x,y,z,roll,pitch,yaw] 分别计算 MinimumJerk 系数（各 6 个）
  │     姿态转换：旋转矩阵 → RPY 角后做标量插值
  │
  ▼
timerThread → processOpenManipulator(time)
  │
  ├─ getJointGoalValueFromTrajectory(time)
  │    └─ TaskTrajectory::getTaskWaypoint(tick)
  │         ├─ 计算当前时刻 Cartesian 位置 [x(t), y(t), z(t)]
  │         └─ 计算当前时刻 RPY 角，再转回旋转矩阵 R(t)
  │    └─ 对当前 Cartesian 目标调用 IK 求解器
  │         → 返回本时刻关节目标角 JointWaypoint
  │
  └─ sendAllJointActuatorValue(goal_joints) ...（同关节空间链路）
```

### 2.3 正运动学：链式法则 + Rodrigues 旋转

**坐标系递推（`forwardSolverUsingChainRule`）：**

对第 $i$ 个连杆：

$$
\mathbf{p}_i = \mathbf{p}_{i-1} + \mathbf{R}_{i-1} \cdot \mathbf{r}_{i|i-1}
$$

$$
\mathbf{R}_i = \mathbf{R}_{i-1} \cdot \mathbf{R}^{\text{rel}}_{i|i-1} \cdot \mathbf{R}_{\text{Rodrigues}}(\hat{\mathbf{n}}_i,\, \theta_i)
$$

其中 $\mathbf{r}_{i|i-1}$ 为在父坐标系中的相对位置，$\hat{\mathbf{n}}_i$ 为关节轴，$\theta_i$ 为当前关节角。

**Rodrigues 旋转公式：**

$$
\mathbf{R}(\hat{\mathbf{n}}, \theta) = \mathbf{I} + \sin\theta\,[\hat{\mathbf{n}}]_{\times} + (1 - \cos\theta)\,[\hat{\mathbf{n}}]_{\times}^2
$$

反对称矩阵（skew-symmetric）：
$$
[\hat{\mathbf{n}}]_{\times} = \begin{bmatrix} 0 & -n_z & n_y \\ n_z & 0 & -n_x \\ -n_y & n_x & 0 \end{bmatrix}
$$

OpenManipulator-X 的关节轴：

| 关节 | 旋转轴（局部系） |
|------|-----------------|
| joint1 | Z = (0,0,1) |
| joint2 | Y = (0,1,0) |
| joint3 | Y = (0,1,0) |
| joint4 | Y = (0,1,0) |

### 2.4 Jacobian 矩阵构造

对**旋转关节** $i$，关节轴在世界系中的方向为：

$$
\hat{\mathbf{z}}_i = \mathbf{R}_{\text{parent}(i)} \cdot \hat{\mathbf{n}}_i
$$

Jacobian 第 $i$ 列：

$$
\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{v,i} \\ \mathbf{J}_{\omega,i} \end{bmatrix}
= \begin{bmatrix} \hat{\mathbf{z}}_i \times (\mathbf{p}_e - \mathbf{p}_i) \\ \hat{\mathbf{z}}_i \end{bmatrix}
= \begin{bmatrix} [\hat{\mathbf{z}}_i]_{\times}\,(\mathbf{p}_e - \mathbf{p}_i) \\ \hat{\mathbf{z}}_i \end{bmatrix}
$$

完整 Jacobian（4 个旋转关节，$6 \times 4$ 矩阵）：

$$
\mathbf{J} = \begin{bmatrix}
| & | & | & | \\
\mathbf{J}_1 & \mathbf{J}_2 & \mathbf{J}_3 & \mathbf{J}_4 \\
| & | & | & |
\end{bmatrix}_{6\times4}
$$

### 2.5 逆运动学求解器

工作空间提供 4 种 IK 求解器（在 `open_manipulator_libs/kinematics.cpp` 中实现）：

---

#### 2.5.1 标准雅可比伪逆法（`SolverUsingCRAndJacobian`）

每次迭代（最多 10 次）：

1. FK 更新当前末端位姿
2. 计算误差向量（位置 + 姿态）：

$$
\Delta\mathbf{x} = \begin{bmatrix} \mathbf{p}_{\text{target}} - \mathbf{p}_e \\ \text{AxisAngle}(\mathbf{R}_{\text{target}} \mathbf{R}_e^{-1}) \end{bmatrix}_{6\times1}
$$

3. 求解增量关节角（QR 分解）：

$$
\mathbf{J}\,\Delta\mathbf{q} = \lambda\,\Delta\mathbf{x}, \quad \lambda = 0.7
$$

4. 更新：$\mathbf{q} \leftarrow \mathbf{q} + \Delta\mathbf{q}$

收敛条件：$\|\Delta\mathbf{x}\| < 10^{-6}$

---

#### 2.5.2 奇异鲁棒雅可比法（`SolverUsingCRAndSRJacobian`）

引入**误差加权矩阵**和**自适应阻尼**：

$$
\mathbf{W}_e = \text{diag}\!\left(\underbrace{\tfrac{1}{0.3},\tfrac{1}{0.3},\tfrac{1}{0.3}}_{\text{位置权重}},\,\underbrace{\tfrac{1}{2\pi},\tfrac{1}{2\pi},\tfrac{1}{2\pi}}_{\text{姿态权重}}\right)
$$

误差能量函数：

$$
E_k = \Delta\mathbf{x}^T \mathbf{W}_e \,\Delta\mathbf{x}
$$

自适应阻尼因子（随误差变化，接近奇异时阻尼增大）：

$$
\lambda_k = E_k + 0.002
$$

求解方程（加权最小二乘）：

$$
\underbrace{(\mathbf{J}^T\mathbf{W}_e\mathbf{J} + \lambda_k\,\mathbf{W}_n)}_{\text{SR Jacobian}}\,\Delta\mathbf{q}
= \underbrace{\mathbf{J}^T\mathbf{W}_e\,\Delta\mathbf{x}}_{\text{加权梯度}}
$$

其中 $\mathbf{W}_n = \mathbf{I}$ 为关节空间阻尼单位矩阵。

**回退机制（防止发散）：**

$$
\text{若}\ E_{k+1} \geq E_k \,：\quad \mathbf{q} \leftarrow \mathbf{q} - \gamma\,\Delta\mathbf{q},\quad \gamma = 0.5
$$

收敛条件：$E_k < 10^{-12}$（约 $\|\Delta\mathbf{p}\| < 0.32\,\mu\text{m}$，$\|\Delta R\| < 0.16\,\mu\text{rad}$）

---

#### 2.5.3 OpenManipulator 定制求解器（`SolverCustomizedforOMChain`）—— **默认求解器**

在 SR Jacobian 基础上，**首先对目标姿态施加物理约束**（利用该机械臂的结构特性）：

```cpp
// 1. 强制 yaw = atan2(y_target, x_target)（ joint1 指向目标在 XY 平面的方向）
target_rpy[2] = atan2(p_target_y, p_target_x);

// 2. 保持当前 roll 不变（joint2-4 均绕 Y 轴，无法独立控制 roll）
target_rpy[0] = current_rpy[0];

// 3. pitch 使用请求值
target_rpy[1] = requested_pitch;
```

**物理意义：**

- `joint1` 绕 Z 轴旋转 → 控制末端在 XY 平面的方向角（yaw）
- `joint2-4` 均绕 Y 轴旋转 → 控制末端抬高/伸展（pitch），无法单独控制 roll
- 因此向目标发起运动时，最优做法是先把 yaw 对准目标方向

这一约束使初始状态更接近解，显著提升迭代收敛速度，也避免奇异附近的数值不稳定。

之后再调用与 `SolverUsingCRAndSRJacobian` 相同的 SR 迭代循环。

---

## 3. 路径执行：MinimumJerk 算法原理

### 3.1 定时器线程架构

```
main thread (ROS spin)
    │
    │ startTimerThread()
    ▼
POSIX timer thread (10ms 间隔，CLOCK_MONOTONIC 高精度)
    │
    ├─ process(time) → processOpenManipulator(time)
    │     └─ 100Hz 轨迹采样 + 状态更新
    │
    └─ (ROS Timer, 独立) publishCallback()
          └─ 100Hz 发布 /joint_states
```

### 3.2 MinimumJerk 多项式：完整数学推导

#### 变分问题

**目标**：在给定起止状态约束下，找最平滑轨迹以最小化"jerk"（加加速度）的平方积分：

$$
\min_{\theta(t)} \quad J = \int_0^T \left(\frac{d^3\theta}{dt^3}\right)^2 dt
$$

**边界条件（共 6 个）：**

$$
\theta(0)=\theta_0,\quad \dot\theta(0)=v_0,\quad \ddot\theta(0)=a_0
$$
$$
\theta(T)=\theta_f,\quad \dot\theta(T)=v_f,\quad \ddot\theta(T)=a_f
$$

（在 `makeJointTrajectory` 中，起止速度和加速度通常均为 0。）

#### 最优多项式

使用 Euler-Lagrange 变分法可证明，满足上述 6 个边界条件的最优轨迹是**五阶多项式**：

$$
\theta(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
$$

#### 系数求解矩阵方程（源码直接实现）

低阶系数由初始状态直接确定：

$$
a_0 = \theta_0, \quad a_1 = v_0, \quad a_2 = \tfrac{1}{2} a_0
$$

高阶系数 $(a_3, a_4, a_5)$ 由末端边界条件组成的 $3\times3$ 线性方程组求解：

$$
\underbrace{\begin{bmatrix}
T^3 & T^4 & T^5 \\
3T^2 & 4T^3 & 5T^4 \\
6T & 12T^2 & 20T^3
\end{bmatrix}}_{\mathbf{A}}
\begin{bmatrix} a_3 \\ a_4 \\ a_5 \end{bmatrix}
=
\underbrace{\begin{bmatrix}
\theta_f - \theta_0 - v_0 T - \tfrac{1}{2}a_0 T^2 \\
v_f - v_0 - a_0 T \\
a_f - a_0
\end{bmatrix}}_{\mathbf{b}}
$$

以 `ColPivHouseholderQR` 分解求解（数值稳定）。

#### 零初末速度/加速度时的解析表达

令 $v_0=v_f=a_0=a_f=0$，$\Delta\theta = \theta_f - \theta_0$，代入解：

$$
a_3 = \frac{10\Delta\theta}{T^3},\quad
a_4 = -\frac{15\Delta\theta}{T^4},\quad
a_5 = \frac{6\Delta\theta}{T^5}
$$

归一化时间 $\tau = t/T \in [0,1]$：

$$
\boxed{
\theta(t) = \theta_0 + \Delta\theta \bigl(10\tau^3 - 15\tau^4 + 6\tau^5\bigr)
}
$$

| 时刻 | 位置 | 速度特性 |
|------|------|---------|
| $\tau=0$ | $\theta_0$ | 速度=0 |
| $\tau=0.5$ | $\theta_0 + 0.5\Delta\theta$ | 速度最大 |
| $\tau=1$ | $\theta_f$ | 速度=0 |

速度最大值发生在 $\tau=0.5$：

$$
\dot\theta_{\max} = \frac{15\Delta\theta}{8T}
$$

加速度在 $\tau = 0.5 \pm \sqrt{0.1} \approx 0.184, 0.816$ 处取极值。

### 3.3 任务空间轨迹的姿态插值

任务空间轨迹（`TaskTrajectory::makeTaskTrajectory`）对 **6 个自由度**各自独立应用 MinimumJerk：

| 通道 | 描述 | 处理方式 |
|------|------|---------|
| 0-2 | 末端位置 $x, y, z$ | 直接标量插值 |
| 3-5 | 末端姿态 $r,p,y$ | RPY 角标量插值 |

**姿态插值流程：**

```
目标旋转矩阵 R_target
  → convertRotationMatrixToRPYVector()  →  (roll_f, pitch_f, yaw_f)
    当前旋转矩阵 R_current
  → convertRotationMatrixToRPYVector()  →  (roll_0, pitch_0, yaw_0)

对 roll、pitch、yaw 各自运行 MinimumJerk 插值
  → r(t), p(t), y(t)

在每个 tick：
  R(t) = convertRPYToRotationMatrix(r(t), p(t), y(t))
  然后调用 IK(R(t), p(t)) → joint angles(t)
```

**注意**：RPY 插值（而非四元数 SLERP）在大角度旋转时可能经过奇点（万向锁），对于加油枪安装等大姿态变化任务，建议使用位移量较小的中间航点。

### 3.4 100Hz IK 循环图（任务空间执行）

```
每 10ms:
  tick = now - start_time

  [x(tick), y(tick), z(tick)] ←─ position MinimumJerk
  [r(tick), p(tick), yaw(tick)] ←─ orientation MinimumJerk
  R(tick) = RPY2Rotation(r, p, yaw)

  IK({p_target(tick), R(tick)}) → q_target(tick)
     └─ chainCustomInverseKinematics
         1. 约束 yaw_IK = atan2(y,x), roll_IK = roll_now
         2. SR 加权最小二乘迭代（10次以内）

  sendAllJointActuatorValue(q_target)   ← 驱动关节

  solveForwardKinematics()              ← 更新内部末端位姿状态

  publishJointStates()                  → /joint_states → RViz
```

---

## 4. 加油枪抓取功能包接口设计

### 4.1 任务分解

```
阶段 0：感知   →  检测加油枪位姿
阶段 1：接近   →  末端移动到抓取预备点（grasp_prepose）
阶段 2：精对正 →  精确对准抓取姿态
阶段 3：夹取   →  夹爪闭合 + 确认接触
阶段 4：提升   →  携枪退出
阶段 5：迁移   →  搬运至安装点上方
阶段 6：插入   →  以指定姿态逼近目标孔位（需末端朝向约束）
阶段 7：到位   →  夹爪松开 + 验证安装
```

### 4.2 需要使用的话题与服务

#### 订阅（感知当前状态）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 监听当前关节角（100Hz） |
| `/open_manipulator/kinematics_pose` (gripper) | `open_manipulator_msgs/KinematicsPose` | 实时末端位姿 |
| `/open_manipulator/states` | `open_manipulator_msgs/OpenManipulatorState` | 判断是否处于运动中 |
| `/camera/color/image_raw` 或 `/camera/depth/...` | `sensor_msgs/Image` | 感知相机（外部）|
| `/tf` | TF2 tree | 坐标系变换（相机→基坐标系变换） |

**等待运动完成的标准做法：**

```python
def wait_for_stop():
    while True:
        state = rospy.wait_for_message("/open_manipulator/states",
                                       OpenManipulatorState)
        if state.open_manipulator_moving_state == "STOPPED":
            break
        rospy.sleep(0.05)
```

#### 调用（驱动运动）

| 服务 | 类型 | 用途 |
|------|------|------|
| `/goal_joint_space_path` | `SetJointPosition` | 移动到安全初始姿态（避障） |
| `/goal_task_space_path` | `SetKinematicsPose` | 末端移动到指定位姿（6DOF） |
| `/goal_task_space_path_position_only` | `SetKinematicsPose` | 仅平移，不改变当前姿态 |
| `/goal_task_space_path_from_present_position_only` | `SetKinematicsPose` | 相对当前位置微调 |
| `/goal_task_space_path_from_present_orientation_only` | `SetKinematicsPose` | 在当前位置微调姿态 |
| `/goal_tool_control` | `SetJointPosition` | 夹爪开合 |
| `/get_kinematics_pose` | `GetKinematicsPose` | 主动查询末端位姿 |
| `/get_joint_position` | `GetJointPosition` | 主动查询关节角 |

#### 服务调用示意（Python）

```python
#!/usr/bin/env python3
import rospy
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import OpenManipulatorState
from geometry_msgs.msg import Pose

def call_task_space(x, y, z, qw, qx, qy, qz, path_time):
    cli = rospy.ServiceProxy("/goal_task_space_path", SetKinematicsPose)
    req = SetKinematicsPoseRequest()
    req.end_effector_name = "gripper"
    req.kinematics_pose.pose.position.x = x
    req.kinematics_pose.pose.position.y = y
    req.kinematics_pose.pose.position.z = z
    req.kinematics_pose.pose.orientation.w = qw
    req.kinematics_pose.pose.orientation.x = qx
    req.kinematics_pose.pose.orientation.y = qy
    req.kinematics_pose.pose.orientation.z = qz
    req.path_time = path_time
    return cli(req).is_planned

def gripper_open():
    cli = rospy.ServiceProxy("/goal_tool_control", SetJointPosition)
    req = SetJointPositionRequest()
    req.joint_position.joint_name = ["gripper"]
    req.joint_position.position   = [0.010]   # 开爪
    req.path_time = 1.0
    cli(req)

def gripper_close():
    cli = rospy.ServiceProxy("/goal_tool_control", SetJointPosition)
    req = SetJointPositionRequest()
    req.joint_position.joint_name = ["gripper"]
    req.joint_position.position   = [-0.005]  # 闭爪（根据枪柄直径调整）
    req.path_time = 1.0
    cli(req)
```

### 4.3 状态机设计

```
                  ┌──────────────────────────────────────────┐
                  │           FuelGunGraspStateMachine        │
                  └──────────────────────────────────────────┘
     初始化
      │
      ▼
  IDLE ──── 触发任务 ────►  DETECT_GUN
                              │
                          感知枪位姿（TF/视觉）
                              │
                              ▼
                         APPROACH_PREPOSE
                              │
                          call task_space(prepose, t=3s)
                          wait_for_stop()
                              │
                              ▼
                         ALIGN_GRASP
                              │
                          call task_space(grasp_pose, t=1s)
                          wait_for_stop()
                              │
                              ▼
                         CLOSE_GRIPPER
                              │
                          gripper_close()
                          sleep(1.0s)
                              │
                              ▼
                         LIFT_GUN
                              │
                          call task_space_from_present(Δz=+0.05m, t=1s)
                          wait_for_stop()
                              │
                              ▼
                         TRANSFER_TO_TARGET
                              │
                          call task_space(above_target, t=4s)
                          wait_for_stop()
                              │
                              ▼
                         INSERT_GUN
                              │
                          call task_space(install_pose, t=2s)  ← 指定安装姿态
                          wait_for_stop()
                              │
                              ▼
                         RELEASE_GRIPPER
                              │
                          gripper_open()
                          sleep(0.5s)
                              │
                              ▼
                         RETURN_HOME
                              │
                          call joint_space([0,0,0,0], t=3s)
                              │
                              ▼
                           DONE
```

### 4.4 功能包结构

```
my_fuel_gun_grasp/
├── CMakeLists.txt
├── package.xml                   # 依赖: roscpp, rospy, open_manipulator_msgs,
│                                 #        geometry_msgs, tf2_ros, sensor_msgs
├── launch/
│   └── fuel_gun_grasp.launch     # 启动仿真+本节点
├── config/
│   ├── grasp_poses.yaml          # 各阶段位姿（prepose, grasp_pose, install_pose…）
│   └── robot_params.yaml         # path_time, 夹爪力度阈值等
├── scripts/
│   ├── fuel_gun_grasp_node.py    # 主状态机节点
│   ├── arm_client.py             # 封装所有 service call
│   └── workspace_check.py        # IK 可达性预检
└── src/
    └── grasp_planner.cpp         # 可选：C++ 高性能规划
```

---

## 5. 动作空间分析功能包设计

### 5.1 功能需求

| 分析目标 | 指标 | 可视化形式 |
|---------|------|----------|
| 可达工作空间 | 末端位置可达点云 | 3D 散点图（matplotlib/RViz Marker） |
| 姿态可达性 | 各位置能实现的姿态锥 | 方向码颜色编码点云 |
| 可操作度 | Yoshikawa 可操作度 $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$ | 热图/彩色点云 |
| 轨迹代价 | 路径长度、关节行程、加速度积分 | 轨迹线 + 代价曲线 |
| 奇异性分析 | Jacobian 最小奇异值 | 热图，标注奇异区域 |

### 5.2 功能包结构

```
arm_workspace_analysis/
├── CMakeLists.txt
├── package.xml                   # 依赖: roscpp, rospy, visualization_msgs,
│                                 #       geometry_msgs, sensor_msgs, std_msgs, numpy, matplotlib
├── launch/
│   ├── workspace_analysis.launch         # 工作空间分析（不需要 RViz sim 运行）
│   └── trajectory_analysis.launch        # 轨迹分析（需要控制器在线）
├── config/
│   └── analysis_params.yaml      # 采样数、角度网格密度、可视化参数
├── scripts/
│   ├── workspace_sampler.py      # 蒙特卡洛 FK 采样 → 可达点云
│   ├── manipulability_map.py     # 可操作度热图
│   ├── trajectory_analyzer.py    # 订阅轨迹话题 → 实时代价分析
│   ├── ik_feasibility_checker.py # 批量 IK 可达性测试
│   └── plot_all.py               # 综合绘图入口
├── src/
│   └── workspace_analyzer_node.cpp  # RViz Marker 发布（C++，高性能）
└── rviz/
    └── analysis.rviz             # 预配置 RViz 布局（点云 + Marker + TF）
```

### 5.3 可达工作空间点云（实现示例）

```python
#!/usr/bin/env python3
"""workspace_sampler.py
蒙特卡洛正运动学采样，生成 3D 可达点云并保存为 CSV 和 PNG。
不需要 ROS 运行，纯离线计算。
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# ---------- 机器人参数（单位：m，弧度）----------
JOINTS = [
    ("joint1", -2.827, 2.827,  np.array([0,0,1])),  # Z轴
    ("joint2", -1.796, 1.571,  np.array([0,1,0])),  # Y轴
    ("joint3", -0.942, 1.379,  np.array([0,1,0])),
    ("joint4", -1.796, 2.042,  np.array([0,1,0])),
]
# 连杆偏置（来自 URDF，单位 m）
OFFSETS = [
    np.array([0.012, 0.0, 0.077]),  # world → joint1
    np.array([0.000, 0.0, 0.060]),  # joint1 → joint2
    np.array([0.024, 0.0, 0.128]),  # joint2 → joint3
    np.array([0.124, 0.0, 0.000]),  # joint3 → joint4
    np.array([0.126, 0.0, 0.000]),  # joint4 → end_effector
]

def rodrigues(axis, theta):
    K = np.array([[0,-axis[2],axis[1]], [axis[2],0,-axis[0]], [-axis[1],axis[0],0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)

def fk(q):
    """正运动学：返回末端位置 (x,y,z)"""
    R = np.eye(3)
    p = np.zeros(3)
    for i, (_, _, _, axis) in enumerate(JOINTS):
        p = p + R @ OFFSETS[i]
        R = R @ rodrigues(axis, q[i])
    p = p + R @ OFFSETS[4]  # end-effector offset
    return p

# ---------- 蒙特卡洛采样 ----------
N = 300_000
rng = np.random.default_rng(42)
q_samples = np.column_stack([rng.uniform(lo, hi, N) for _, lo, hi, _ in JOINTS])
pts = np.array([fk(q) for q in q_samples])

# ---------- 可操作度（近似，仅位置）----------
# 用 J 行列式近似：采样附近有限差分
def manip_approx(q, eps=1e-4):
    J = np.zeros((3, 4))
    for i in range(4):
        dq = q.copy(); dq[i] += eps
        J[:, i] = (fk(dq) - fk(q)) / eps
    return np.sqrt(max(np.linalg.det(J @ J.T), 0))

# ---------- 保存结果 ----------
df = pd.DataFrame(pts, columns=["x","y","z"])
df.to_csv("workspace_points.csv", index=False)
print(f"采样完成，可达点数：{len(pts)}")
print(f"X 范围：[{pts[:,0].min():.3f}, {pts[:,0].max():.3f}] m")
print(f"Y 范围：[{pts[:,1].min():.3f}, {pts[:,1].max():.3f}] m")
print(f"Z 范围：[{pts[:,2].min():.3f}, {pts[:,2].max():.3f}] m")

# ---------- 绘图 ----------
fig = plt.figure(figsize=(12, 10))

# XYZ 点云
ax1 = fig.add_subplot(221, projection='3d')
sc = ax1.scatter(pts[::10,0], pts[::10,1], pts[::10,2],
                 s=0.2, c=pts[::10,2], cmap='viridis', alpha=0.3)
ax1.set_title("可达工作空间点云")
ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")
plt.colorbar(sc, ax=ax1, label="Z (m)")

# XZ 截面
ax2 = fig.add_subplot(222)
ax2.hexbin(pts[:,0], pts[:,2], gridsize=80, cmap='hot')
ax2.set_title("XZ 截面密度图"); ax2.set_xlabel("X"); ax2.set_ylabel("Z")

# XY 截面
ax3 = fig.add_subplot(223)
ax3.hexbin(pts[:,0], pts[:,1], gridsize=80, cmap='Blues')
ax3.set_title("XY 截面密度图"); ax3.set_xlabel("X"); ax3.set_ylabel("Y")
ax3.set_aspect('equal')

# Z 分布直方图
ax4 = fig.add_subplot(224)
ax4.hist(pts[:,2], bins=100, color='steelblue', edgecolor='none')
ax4.set_title("Z 轴高度分布"); ax4.set_xlabel("Z (m)"); ax4.set_ylabel("频次")

plt.tight_layout()
plt.savefig("workspace_cloud.png", dpi=150)
plt.show()
```

### 5.4 可操作度分析

**Yoshikawa 可操作度指标：**

$$
w(\mathbf{q}) = \sqrt{\det(\mathbf{J}(\mathbf{q})\,\mathbf{J}(\mathbf{q})^T)}
$$

- $w > 0$：姿态可达，无奇异
- $w = 0$：奇异状态（关节轴对齐，自由度退化）
- $w$ 越大，末端速度/力的可操作性越好，IK 收敛越容易

**最小奇异值指标（更敏感）：**

$$
\sigma_{\min}(\mathbf{J}) = \lambda_{\min}(\mathbf{J}\mathbf{J}^T)^{1/2}
$$

可将 $w$ 或 $\sigma_{\min}$ 作为颜色值映射到工作空间点云。

```python
# manipulability_map.py 核心片段
def yoshikawa(q):
    J = build_jacobian(q)   # 6×4
    val = np.linalg.det(J @ J.T)
    return np.sqrt(max(val, 0))

# 对点云中每个采样点计算可操作度，用颜色编码
w_vals = np.array([yoshikawa(q) for q in q_samples[::50]])
sc = ax.scatter(pts[::50,0], pts[::50,1], pts[::50,2],
                c=w_vals, cmap='RdYlGn', s=1, vmin=0, vmax=w_vals.max())
plt.colorbar(sc, label="Yoshikawa 可操作度 w")
```

### 5.5 轨迹代价分析

代价指标定义：

$$
C_{\text{length}} = \sum_{k} \|\mathbf{p}_{k+1} - \mathbf{p}_k\|_2 \quad\text{（末端路径长度）}
$$

$$
C_{\text{joint}} = \sum_{i=1}^{4}\sum_{k} |\theta_{i,k+1} - \theta_{i,k}| \quad\text{（关节行程总量）}
$$

$$
C_{\text{jerk}} = \sum_{i}\int_0^T\left(\dddot\theta_i\right)^2 dt
= \sum_{i} \left[60a_3^2 T - 360a_3 a_4 T^2 + \cdots\right] \quad\text{（jerk 积分，可由系数解析计算）}
$$

```python
# trajectory_analyzer.py 核心片段
import rospy
from sensor_msgs.msg import JointState

class TrajectoryAnalyzer:
    def __init__(self):
        self.joint_history = []
        self.ee_history = []
        rospy.Subscriber("/joint_states", JointState, self.cb)

    def cb(self, msg):
        self.joint_history.append(
            (msg.header.stamp.to_sec(),
             list(msg.position[:4]))  # joint1-4
        )

    def compute_cost(self):
        times  = [h[0] for h in self.joint_history]
        joints = np.array([h[1] for h in self.joint_history])

        # 关节行程
        joint_travel = np.sum(np.abs(np.diff(joints, axis=0)))

        # 加速度均方根
        dt = np.mean(np.diff(times))
        accs = np.diff(np.diff(joints, axis=0), axis=0) / dt**2
        rms_acc = np.sqrt(np.mean(accs**2))

        return {"joint_travel_rad": joint_travel, "rms_acc": rms_acc}
```

### 5.6 RViz 可视化集成（C++ Marker 节点）

```cpp
// workspace_analyzer_node.cpp（核心片段）
// 发布 visualization_msgs/MarkerArray 至 RViz
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// 1. 发布可达点云（PointCloud2）：直接被 RViz PointCloud2 Display 显示
// 2. 发布可操作度热图（MarkerArray，SPHERE_LIST + 颜色映射）
// 3. 发布轨迹路径（nav_msgs::Path）
// 4. 发布姿态框架（MarkerArray，ARROW + TEXT）

void publishWorkspaceCloud(const ros::Publisher& pub,
                           const std::vector<Eigen::Vector3d>& pts,
                           const std::vector<double>& manip)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (size_t i = 0; i < pts.size(); i++)
    {
        pcl::PointXYZRGB p;
        p.x = pts[i].x(); p.y = pts[i].y(); p.z = pts[i].z();
        // 映射可操作度到颜色
        float w = std::min(1.0, manip[i] / max_manip);
        p.r = (uint8_t)((1-w)*255); p.g = (uint8_t)(w*255); p.b = 0;
        cloud.push_back(p);
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";
    pub.publish(msg);
}
```

### 5.7 分析功能包 CMakeLists.txt 关键依赖

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs
  geometry_msgs visualization_msgs
  open_manipulator_msgs tf2_ros
  pcl_ros
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_python_setup()   # 若使用 Python 节点
```

### 5.8 一键运行工作流

```bash
# Step 1：启动仿真（用于 IK 可达性在线测试）
roslaunch open_manipulator_description open_manipulator_rviz_sim.launch

# Step 2：离线工作空间采样（不需要 ROS）
python3 src/arm_workspace_analysis/scripts/workspace_sampler.py

# Step 3：发布点云到 RViz（在线）
rosrun arm_workspace_analysis workspace_analyzer_node

# Step 4：在线轨迹代价记录（执行运动时同时运行）
rosrun arm_workspace_analysis trajectory_analyzer.py

# Step 5：综合绘图报告
python3 src/arm_workspace_analysis/scripts/plot_all.py
```

---

## 附录：坐标系与符号约定

| 符号 | 含义 |
|------|------|
| $\mathbf{q} \in \mathbb{R}^4$ | 关节角向量 $[\theta_1,\theta_2,\theta_3,\theta_4]^T$ |
| $\mathbf{p}_e \in \mathbb{R}^3$ | 末端位置（world 系） |
| $\mathbf{R}_e \in SO(3)$ | 末端旋转矩阵（world 系） |
| $\mathbf{J} \in \mathbb{R}^{6\times4}$ | 几何 Jacobian |
| $T$ | 轨迹总时间（path_time，秒） |
| $\tau = t/T$ | 归一化时间 $[0,1]$ |
| $w$ | Yoshikawa 可操作度 |
| $\sigma_{\min}$ | Jacobian 最小奇异值 |
| $\mathbf{W}_e$ | 误差加权矩阵（位置/姿态权重比 $\approx$ 0.3m / $2\pi$rad） |
| $\lambda$ | SR 自适应阻尼因子 |

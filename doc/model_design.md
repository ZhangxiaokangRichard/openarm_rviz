# 场景模型自定义 与 机械臂基座位置设计指南

> 文档版本：v1.0  
> 适用功能包：`fuel_gun_grasp`  
> 前置阅读：`doc/guide.md`、`doc/arg.md`

---

## 目录

1. [概述](#1-概述)
2. [场景模型自定义（充电站 / 汽车）](#2-场景模型自定义充电站--汽车)
   - 2.1 [当前实现说明](#21-当前实现说明)
   - 2.2 [方案 A：Mesh Marker 方案（推荐）](#22-方案-a-mesh-marker-方案推荐)
   - 2.3 [方案 B：URDF + robot_state_publisher 方案](#23-方案-b-urdf--robot_state_publisher-方案)
   - 2.4 [3D 模型文件准备](#24-3d-模型文件准备)
   - 2.5 [坐标对齐注意事项](#25-坐标对齐注意事项)
3. [机械臂基座位置与朝向自定义](#3-机械臂基座位置与朝向自定义)
   - 3.1 [URDF 结构分析](#31-urdf-结构分析)
   - 3.2 [方案 X：直接修改 world_fixed joint（最简单）](#32-方案-x直接修改-world_fixed-joint最简单)
   - 3.3 [方案 Y：xacro 参数化（推荐）](#33-方案-y-xacro-参数化推荐)
   - 3.4 [常见基座位置场景示例](#34-常见基座位置场景示例)
4. [两者联动：统一坐标系设计](#4-两者联动统一坐标系设计)

---

## 1. 概述

本工程的 RViz 仿真场景由两部分可视元素组成：

| 元素 | 来源 | 配置入口 |
|---|---|---|
| 机械臂模型 | `open_manipulator_description/urdf/` | `open_manipulator_robot.urdf.xacro` |
| 充电站 / 汽车场景模型 | `fuel_gun_grasp_node` 发布的 MarkerArray | `config/params.yaml` |

目前场景模型使用**几何原语（box / cylinder）**作为占位可视化。本文档介绍如何将其替换为真实 3D 模型，以及如何调整机械臂在世界坐标系中的位置与朝向。

---

## 2. 场景模型自定义（充电站 / 汽车）

### 2.1 当前实现说明

`fuel_gun_grasp_node` 节点在启动时调用 `publishSceneObjects()`，向 topic  
`/fuel_gun_grasp_node/scene_markers`（latched）发布 `visualization_msgs/MarkerArray`，  
包含 7 个 Marker（5 个几何体 + 2 个目标点标记）。

```
MarkerArray
├── charging_station/0  CUBE    蓝灰色主机箱
├── charging_station/1  CUBE    顶部面板
├── charging_station/2  CYLINDER 橙色插槽
├── car/0               CUBE    红色底盘
├── car/1               CUBE    深红驾驶室
├── car/2               CYLINDER 黄色加油口
├── targets/0           SPHERE  绿色 — gun_pose（抓取目标）
└── targets/1           SPHERE  紫色 — install_pose（安装目标）
```

替换为自定义 3D 模型有两条路径，下面分别说明。

---

### 2.2 方案 A：Mesh Marker 方案（推荐）

**原理**：将部分 `CUBE`/`CYLINDER` Marker 的 `type` 改为  
`visualization_msgs::Marker::MESH_RESOURCE`，指向本功能包内的 STL / DAE 文件。  
无需额外节点，位置仍由 `params.yaml` 控制，是最轻量的升级方式。

#### 目录结构（改造后）

```
fuel_gun_grasp/
├── meshes/
│   ├── charging_station.stl   ← 充电站完整模型
│   ├── car_body.stl           ← 汽车车身
│   └── car_fuel_port.stl      ← 加油口特写（可选）
├── urdf/
│   ├── charging_station.urdf.xacro   ← 已有，可同步更新
│   └── car.urdf.xacro                ← 已有，可同步更新
└── config/params.yaml
```

#### 代码修改要点

在 `fuel_gun_grasp_node.cpp` 的 `publishSceneObjects()` 中，  
将原来的 `makeBoxMarker(...)` 替换为以下模板：

```cpp
// 用法示例：替换充电站主机箱为 STL 模型
visualization_msgs::Marker m;
m.header.frame_id = "world";
m.header.stamp    = ros::Time(0);           // 0 = 使用最新 TF，避免过期
m.ns     = "charging_station";
m.id     = 0;
m.type   = visualization_msgs::Marker::MESH_RESOURCE;
m.action = visualization_msgs::Marker::ADD;

// 模型文件路径（必须以 package:// 开头）
m.mesh_resource = "package://fuel_gun_grasp/meshes/charging_station.stl";
m.mesh_use_embedded_materials = true;       // 使用 DAE 内嵌材质；STL 忽略此项

// 位置：与 params.yaml 中 charging_station.pose 对应
m.pose.position.x    = cs_pose_.x;
m.pose.position.y    = cs_pose_.y;
m.pose.position.z    = cs_pose_.z;
m.pose.orientation.w = 1.0;                // 无旋转（按需设置）

// scale：STL 单位通常为 mm，需缩放到 m
// SolidWorks/FreeCAD 导出 mm 时：scale = 0.001
// Blender 默认导出 m 时：scale = 1.0
m.scale.x = 0.001;
m.scale.y = 0.001;
m.scale.z = 0.001;

// 颜色（仅对 STL 有效；DAE 有内嵌材质时设 mesh_use_embedded_materials=true）
m.color.r = 0.25f;
m.color.g = 0.40f;
m.color.b = 0.65f;
m.color.a = 0.90f;                         // < 1.0 时透明

m.lifetime = ros::Duration(0);             // 永久
ma.markers.push_back(m);
```

#### STL / DAE 的颜色与材质对比

| 格式 | 颜色来源 | 透明度 | 推荐场景 |
|---|---|---|---|
| **STL** | Marker 的 `color` 字段 | 支持（`a < 1`） | 工程快速验证 |
| **DAE (Collada)** | 文件内嵌材质 / `color` 字段均可 | 支持 | 最终效果展示 |
| **OBJ** | RViz 不原生支持，需先转为 DAE | — | 转换后使用 |

---

### 2.3 方案 B：URDF + robot_state_publisher 方案

**原理**：将充电站 / 汽车作为独立机器人模型，通过各自的  
`robot_state_publisher` 发布 TF 树，在 RViz 中用 `RobotModel` 显示器展现。

**适用场景**：需要精确 TF 帧（供其他节点查询物体坐标）、或模型有活动关节（例如车门开合）。

#### ⚠️ TF 帧名冲突问题

`robot_state_publisher` 发布时，link 名称直接进入全局 TF 树。  
充电站 URDF 和机械臂 URDF 都声明了 `<link name="world"/>`，  
**同时加载会导致 TF 冲突**，RViz 显示异常。

**解决方法**：给场景模型的根 link 使用唯一名称，并通过  
`static_transform_publisher` 手动连接到公共 `world` 帧。

#### 实现步骤

**Step 1**：修改 `urdf/charging_station.urdf.xacro`，  
将根 link 从 `world` 改为 `charging_station_root`：

```xml
<link name="charging_station_root"/>

<joint name="charging_station_fixed" type="fixed">
  <parent link="charging_station_root"/>
  <child  link="charging_station_base"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

**Step 2**：在 `sim_with_grasp.launch` 中添加：

```xml
<!-- 充电站：独立 robot_description 参数空间，避免覆盖机械臂的 robot_description -->
<param name="charging_station_description"
       command="$(find xacro)/xacro
                '$(find fuel_gun_grasp)/urdf/charging_station.urdf.xacro'"/>

<node name="charging_station_state_publisher"
      pkg="robot_state_publisher"
      type="robot_state_publisher">
  <remap from="robot_description" to="charging_station_description"/>
</node>

<!-- 将 charging_station_root 连接到 world，位置来自 params.yaml 中的值 -->
<!-- 注意：此处坐标需与 config/params.yaml 的 charging_station.pose 手动保持一致 -->
<node pkg="tf2_ros" type="static_transform_publisher" name="cs_to_world"
      args="0.20 0.04 0.00  0 0 0  world  charging_station_root"/>
```

**Step 3**：在 RViz 中添加 RobotModel 显示器，  
将 `Robot Description` 改为 `charging_station_description`。

> **权衡**：方案 B 配置复杂，且位置需要在 launch 和 params.yaml 两处维护。  
> 仅在确实需要 TF 帧时采用，否则推荐方案 A。

---

### 2.4 3D 模型文件准备

#### 推荐工具链

| 工具 | 用途 | 导出格式 |
|---|---|---|
| **FreeCAD** | 机械建模，参数化设计 | STL（mm） / STEP |
| **Blender** | 有机形体，材质纹理 | DAE / STL（m） |
| **SolidWorks** | 精确工程模型 | STL（mm） |
| **MeshLab** | 模型修复、格式转换、简化面数 | STL / DAE / OBJ |

#### 模型制作要点

1. **坐标原点**：模型的原点应设置为**与 params.yaml 中 `pose` 字段对应的参考点**  
   （充电站 = 底部中心；汽车 = 底盘底部中心）。  
   这样 Marker/URDF 的 `pose.position` 直接等于 params.yaml 里的值，无需额外偏移。

2. **朝向**：建模时 +X 轴朝机械臂方向，+Z 轴朝上。  
   若导出模型朝向不对，在 Marker 的 `pose.orientation` 中添加旋转四元数纠正。

3. **单位**：大部分 CAD 软件默认单位为 **毫米**，  
   导出的 STL 需要在 Marker 的 `scale` 字段统一缩放：
   ```cpp
   m.scale.x = m.scale.y = m.scale.z = 0.001;  // mm → m
   ```

4. **面数控制**：RViz 渲染每帧都重绘，建议单个模型面数控制在 **< 50,000 三角面**，  
   复杂模型用 MeshLab 的 `Filters → Remeshing → Quadric Edge Collapse Decimation` 减面。

5. **DAE 材质**：若使用 DAE 格式，确保贴图文件与 DAE 文件**同目录**放置，  
   并在 `meshes/` 文件夹内一并提交。

#### 文件放置

```
fuel_gun_grasp/
└── meshes/
    ├── charging_station.stl   （或 .dae）
    ├── car.stl                （或 .dae）
    └── textures/              （DAE 贴图，可选）
        └── car_texture.png
```

在 `CMakeLists.txt` 中将 `meshes` 加入安装规则（已有 `urdf`，补充 `meshes`）：

```cmake
install(DIRECTORY config launch urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

---

### 2.5 坐标对齐注意事项

场景模型的物理意义坐标（加油枪插槽、加油口）**必须与抓取目标点对齐**，  
否则视觉模型与机械臂实际运动的目标位置不匹配。

**对齐关系（默认参数值）**：

```
充电站插槽绝对坐标 = charging_station.pose + gun_slot_offset
                   = (0.20, 0.04, 0.00) + (0.00, -0.04, 0.24)
                   = (0.20, 0.00, 0.24)   ← 等于 gun_pose ✓

汽车加油口绝对坐标 = car.pose + fuel_port_offset
                   = (0.08, -0.04, 0.00) + (0.12, 0.12, 0.12)
                   = (0.20, 0.08, 0.12)  ← 等于 install_pose ✓
```

当你修改自定义模型的尺寸或调整 `pose` 时，  
需要同步更新 `params.yaml` 中对应的 `gun_pose` / `install_pose`，  
确保绿球/紫球目标点标记落在模型的关键特征点（插槽口、加油口）上。

---

## 3. 机械臂基座位置与朝向自定义

### 3.1 URDF 结构分析

机械臂 URDF 入口文件为：

```
open_manipulator_description/urdf/open_manipulator_robot.urdf.xacro
```

其中定义了机械臂在世界坐标系中的固定安装关节：

```xml
<link name="world"/>

<joint name="world_fixed" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 0"/>   ← 基座位置和朝向在此定义
  <parent link="world"/>
  <child link="link1"/>
</joint>
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `xyz` | 基座 link1 原点相对于 world 的平移 | m |
| `rpy` | 基座绕 X/Y/Z 轴的旋转（Roll/Pitch/Yaw） | rad |

`rpy` 旋转遵循**固定轴（extrinsic）XYZ 顺序**：先绕世界 X 轴转 roll，再绕 Y 转 pitch，最后绕 Z 转 yaw。

---

### 3.2 方案 X：直接修改 world_fixed joint（最简单）

直接编辑 `open_manipulator_robot.urdf.xacro`，修改 `<origin>` 行：

```xml
<joint name="world_fixed" type="fixed">
  <origin xyz="0.1 0.05 0.0" rpy="0 0 1.5708"/>  <!-- 示例：平移 + 绕 Z 旋转 90° -->
  <parent link="world"/>
  <child link="link1"/>
</joint>
```

**优点**：改一行即生效，无需修改 launch 文件。  
**缺点**：修改了 upstream 文件，多人协作时容易产生 git 冲突；  
无法在 launch 时动态传入位置，每次调整都需要编辑文件。

---

### 3.3 方案 Y：xacro 参数化（推荐）

将基座位置提取为 xacro 参数，通过 launch 文件传入，保持 URDF 文件的通用性。

#### Step 1：修改 `open_manipulator_robot.urdf.xacro`

在文件头部 `<robot ...>` 标签之后添加参数声明，  
并将 `<origin>` 改为引用这些参数：

```xml
<robot name="open_manipulator" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- === 基座安装位置参数（可在 launch 文件中覆盖） === -->
  <xacro:arg name="base_x"   default="0.0"/>   <!-- 基座 X 偏移 (m) -->
  <xacro:arg name="base_y"   default="0.0"/>   <!-- 基座 Y 偏移 (m) -->
  <xacro:arg name="base_z"   default="0.0"/>   <!-- 基座 Z 偏移（安装台高度）(m) -->
  <xacro:arg name="base_yaw" default="0.0"/>   <!-- 绕 Z 轴旋转（偏航）(rad) -->

  <!-- ... （其他 include 保持不变） ... -->

  <link name="world"/>

  <joint name="world_fixed" type="fixed">
    <!-- $(arg ...) 从 xacro 参数读取 -->
    <origin xyz="$(arg base_x) $(arg base_y) $(arg base_z)"
            rpy="0 0 $(arg base_yaw)"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>

</robot>
```

> **说明**：通常不需要改变 roll/pitch（机械臂竖直安装），  
> 只需 xyz 平移 + yaw 偏航即可覆盖大多数场景。  
> 若需要任意 6-DOF 安装，可同理添加 `base_roll` / `base_pitch` 参数。

#### Step 2：修改 `open_manipulator_upload.launch`

在 xacro 命令中传入参数值：

```xml
<launch>
  <!-- 基座位置参数（按实际安装位置修改） -->
  <arg name="base_x"   default="0.0"/>
  <arg name="base_y"   default="0.0"/>
  <arg name="base_z"   default="0.0"/>
  <arg name="base_yaw" default="0.0"/>

  <param name="robot_description"
         command="$(find xacro)/xacro --inorder
                  '$(find open_manipulator_description)/urdf/open_manipulator_robot.urdf.xacro'
                  base_x:=$(arg base_x)
                  base_y:=$(arg base_y)
                  base_z:=$(arg base_z)
                  base_yaw:=$(arg base_yaw)"/>
</launch>
```

#### Step 3：从上层 launch 传参

```xml
<!-- sim_with_grasp.launch 或其他上层 launch 中 -->
<include file="$(find open_manipulator_description)/launch/open_manipulator_rviz_sim.launch">
  <arg name="base_x"   value="0.1"/>
  <arg name="base_y"   value="0.0"/>
  <arg name="base_yaw" value="1.5708"/>   <!-- 90° = π/2 -->
</include>
```

但 `open_manipulator_rviz_sim.launch` 目前直接 include 了 `open_manipulator_upload.launch`，  
需要将 `<arg>` 透传链一路配置好。

#### Step 4：同步更新 `params.yaml` 中的绝对坐标

**关键**：机械臂基座位置改变后，`gun_pose` / `install_pose` 以及  
`charging_station.pose` / `car.pose` 的绝对坐标也需要**重新校准**，  
因为这些坐标都基于 `world` 帧，而机械臂的工作空间已经整体平移/旋转。

推荐做法：

1. 先确定基座在 world 中的位置 `(bx, by, bz, yaw)`
2. 计算各目标点在 world 中的绝对坐标  
   （原始坐标以机械臂基座为参考，需要转换到 world 帧）
3. 更新 `params.yaml`

**坐标变换公式**（仅含 yaw 旋转时）：

```
x_world = bx + x_base * cos(yaw) - y_base * sin(yaw)
y_world = by + x_base * sin(yaw) + y_base * cos(yaw)
z_world = bz + z_base
```

其中 `(x_base, y_base, z_base)` 是目标点相对机械臂基座的坐标（原 params.yaml 值）。

---

### 3.4 常见基座位置场景示例

#### 场景 A：机械臂安装在桌面（抬高 80 cm）

```xml
<origin xyz="0 0 0.80" rpy="0 0 0"/>
```

`params.yaml` 中所有 `z` 值需减去 0.80（桌面以上的相对高度保持不变）：

```yaml
gun_pose:
  position:
    z: 0.24   # 保持桌面以上 24 cm 不变
```

实际上若坐标系含义是"相对机械臂基座"，则 params.yaml 无需修改，  
因为 `world_fixed` 的 xyz 偏移只影响 world 帧，不影响机械臂自身的关节空间计算。

> **重要澄清**：`params.yaml` 中的坐标（`gun_pose` 等）是传给  
> `goal_task_space_path_position_only` 服务的，该服务接受的是  
> **机械臂自身坐标系（manipulator frame）**中的位置，  
> **不是 world 帧坐标**。  
> 因此，**仅移动基座不需要修改 params.yaml 中的抓取坐标**，  
> 只需更新场景模型（充电站/汽车）的 world 帧坐标以保持 RViz 视觉对齐。

#### 场景 B：机械臂旋转 180°（面朝相反方向安装）

```xml
<origin xyz="0 0 0" rpy="0 0 3.1416"/>
```

`params.yaml` 的抓取坐标无需修改（运动学以机械臂自身坐标系计算），  
仅场景模型的 world 坐标需要镜像旋转。

#### 场景 C：机械臂侧装（Roll 90°，末端朝下工作）

```xml
<origin xyz="0 0 1.2" rpy="3.1416 0 0"/>
```

此场景中重力方向变化对逆运动学有显著影响，需重新验证工作空间边界。

---

## 4. 两者联动：统一坐标系设计

当同时修改机械臂基座位置和场景模型时，建议按以下顺序设计：

```
Step 1  确定 world 坐标系原点
        ──────────────────────────────────────────────────────────────
        常见做法：world 原点 = 地面上某固定参考点（如工作台角落）。
        机械臂基座相对 world 的坐标为 (bx, by, bz, yaw)。

Step 2  在 world 坐标系中规划充电站 / 汽车的摆放位置
        ──────────────────────────────────────────────────────────────
        根据实际场景测量充电站底部中心、汽车底盘中心在 world 中的坐标，
        填入 params.yaml 的 charging_station.pose / car.pose。

Step 3  计算加油枪取枪点和安装点的 manipulator 坐标
        ──────────────────────────────────────────────────────────────
        目标点 world 坐标已知 → 逆变换到 manipulator 坐标系（减去基座偏移，反向旋转）
        → 填入 params.yaml 的 gun_pose / install_pose。

Step 4  验证目标点在机械臂可达空间内
        ──────────────────────────────────────────────────────────────
        OpenManipulator-X 在 manipulator 坐标系中的安全工作范围：
            x ∈ [0.10, 0.28] m
            |y| < 0.15 m
            z ∈ [0.03, 0.22] m
        超出此范围的目标点调用 IK 会持续返回 is_planned=false。

Step 5  在 RViz 中验证：绿球/紫球小球（targets 命名空间 Marker）
        ──────────────────────────────────────────────────────────────
        小球位置 = gun_pose/install_pose 的 manipulator 坐标，
        由 robot_state_publisher 转换后在 world 帧下显示。
        观察小球是否落在充电站插槽 / 汽车加油口上，若偏差大则返回 Step 3 修正。
```

> **快速验证工具**：  
> `rosrun tf tf_echo world gripper`  
> 在机械臂运动到 HOME 位后查看末端在 world 帧的实际位置，  
> 可辅助校准基座坐标与 params.yaml 目标坐标的一致性。

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
workspace_cloud.py
──────────────────
在 sim_scene.launch 环境下，对 OpenManipulator-X 进行关节空间随机采样，
批量正运动学计算末端 end_effector_link 在 world 帧的可达位置，
发布为 PointCloud2（latched，RViz 订阅后即可看到解空间云图），
并将三视图密度热图保存为 PNG。

启动方式：
  # 先在另一个终端启动仿真
  roslaunch fuel_gun_grasp sim_scene.launch
  # 再运行本脚本
  rosrun fuel_gun_grasp workspace_cloud.py

可覆盖的私有参数：
  ~n_samples   (int,   default=150000) — 采样数量，越大越完整但越慢
  ~output_dir  (str,   default=~)      — PNG 保存目录
  ~frame_id    (str,   default=world)  — PointCloud2 坐标系

RViz 设置：
  1. Add → PointCloud2 → Topic: /workspace_cloud
  2. Color Transformer: AxisColor，Axis: Z（按高度着色）
  3. Size (m): 0.003 ~ 0.005

运动学链（从 open_manipulator.urdf.xacro 提取）：
  world ──[world_fixed: xyz=(bx,by,bz) rpy=(br,bp,by)]──► link1
  link1 ──[joint1:  revolveZ, offset=(0.012, 0, 0.017)]──► link2
  link2 ──[joint2:  revolveY, offset=(0, 0, 0.0595)  ]──► link3
  link3 ──[joint3:  revolveY, offset=(0.024, 0, 0.128)]──► link4
  link4 ──[joint4:  revolveY, offset=(0.124, 0, 0)   ]──► link5
  link5 ──[ee_fixed: offset=(0.126, 0, 0)             ]──► end_effector_link

关节限位（rad）：
  joint1: ±π*0.9
  joint2: [−π*0.57,  π*0.50]
  joint3: [−π*0.30,  π*0.44]
  joint4: [−π*0.57,  π*0.65]
"""
from __future__ import print_function

import math
import os
import time

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# ── 关节限位（来自 URDF）──────────────────────────────────────────────
JOINT_LIMITS = [
    (-math.pi * 0.90,  math.pi * 0.90),   # joint1  (绕 Z)
    (-math.pi * 0.57,  math.pi * 0.50),   # joint2  (绕 Y)
    (-math.pi * 0.30,  math.pi * 0.44),   # joint3  (绕 Y)
    (-math.pi * 0.57,  math.pi * 0.65),   # joint4  (绕 Y)
]


# ── 工具矩阵 ─────────────────────────────────────────────────────────

def _trans(x, y, z):
    """构造 4×4 平移矩阵。"""
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0,  0, 1]], dtype=np.float64)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[ c, 0, s, 0],
                     [ 0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [ 0, 0, 0, 1]], dtype=np.float64)


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]], dtype=np.float64)


# ── 批量旋转矩阵（形状 N×4×4）────────────────────────────────────────

def _batch_rot_z(angles):
    """angles: (N,) → (N,4,4) 绕 Z 轴旋转矩阵。"""
    N = len(angles)
    c = np.cos(angles)   # (N,)
    s = np.sin(angles)
    z = np.zeros(N)
    o = np.ones(N)
    # 逐行堆叠 → (4, N, 4) → transpose → (N, 4, 4)
    R = np.array([[c, -s, z, z],
                  [s,  c, z, z],
                  [z,  z, o, z],
                  [z,  z, z, o]])   # (4, 4, N) for the row-col expansion
    # numpy array() 对上面列表的解释：外层 4 是行，内层 4 是列向量 (N,)
    # shape = (4, 4, N)，需要 transpose → (N, 4, 4)
    return R.transpose(2, 0, 1)


def _batch_rot_y(angles):
    """angles: (N,) → (N,4,4) 绕 Y 轴旋转矩阵。"""
    N = len(angles)
    c = np.cos(angles)
    s = np.sin(angles)
    z = np.zeros(N)
    o = np.ones(N)
    R = np.array([[ c, z, s, z],
                  [ z, o, z, z],
                  [-s, z, c, z],
                  [ z, z, z, o]])   # (4, 4, N)
    return R.transpose(2, 0, 1)


# ── 正运动学（向量化）────────────────────────────────────────────────

def compute_fk(q_array, T_world_base):
    """
    批量正运动学，纯 numpy 实现，不调用 ROS 服务。

    参数
    ----
    q_array      : (N, 4) 关节角数组，单位 rad
    T_world_base : (4, 4) world → link1 的固定变换矩阵

    返回
    ----
    pts : (N, 3) 末端 end_effector_link 在 world 帧的位置
    """
    q1 = q_array[:, 0]
    q2 = q_array[:, 1]
    q3 = q_array[:, 2]
    q4 = q_array[:, 3]

    # 批量旋转矩阵，每个 (N, 4, 4)
    R1 = _batch_rot_z(q1)
    R2 = _batch_rot_y(q2)
    R3 = _batch_rot_y(q3)
    R4 = _batch_rot_y(q4)

    # 固定偏移矩阵（预先组合到各"段"前缀，减少批量乘法次数）
    # Seg1:  T_world_base  @ Trans(0.012, 0, 0.017)  →  (4,4)
    # Seg2:  Trans(0, 0, 0.0595)                      →  (4,4)
    # Seg3:  Trans(0.024, 0, 0.128)                   →  (4,4)
    # Seg4:  Trans(0.124, 0, 0)                       →  (4,4)
    # SegE:  Trans(0.126, 0, 0)                       →  (4,4)
    Seg1 = T_world_base @ _trans(0.012, 0.0,  0.017)
    Seg2 =                _trans(0.0,   0.0,  0.0595)
    Seg3 =                _trans(0.024, 0.0,  0.128)
    Seg4 =                _trans(0.124, 0.0,  0.0)
    SegE =                _trans(0.126, 0.0,  0.0)

    # 链式批量矩阵乘法：(4,4)@(N,4,4) 和 (N,4,4)@(4,4) 均由 numpy 广播处理
    T = Seg1 @ R1        # (N, 4, 4)
    T = T    @ Seg2      # (N, 4, 4)
    T = T    @ R2
    T = T    @ Seg3
    T = T    @ R3
    T = T    @ Seg4
    T = T    @ R4
    T = T    @ SegE

    return T[:, :3, 3]   # (N, 3)


# ── PointCloud2 打包 ─────────────────────────────────────────────────

def make_cloud_msg(pts, frame_id="world"):
    """
    将 (N,3) float64 数组打包为 PointCloud2 消息。
    extra intensity 字段 = 归一化 Z，供 RViz AxisColor 着色。
    """
    xyz = pts.astype(np.float32)
    z_min, z_max = float(xyz[:, 2].min()), float(xyz[:, 2].max())
    intensity = ((xyz[:, 2] - z_min) / max(z_max - z_min, 1e-6)).astype(np.float32)

    # 结构化数组，字段顺序与 PointField 一致（每点 16 字节）
    N = len(xyz)
    buf = np.empty(N, dtype=[
        ('x', np.float32), ('y', np.float32),
        ('z', np.float32), ('intensity', np.float32)
    ])
    buf['x']         = xyz[:, 0]
    buf['y']         = xyz[:, 1]
    buf['z']         = xyz[:, 2]
    buf['intensity'] = intensity

    hdr = Header(frame_id=frame_id, stamp=rospy.Time.now())
    return PointCloud2(
        header=hdr,
        height=1,
        width=N,
        is_dense=True,
        is_bigendian=False,
        fields=[
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ],
        point_step=16,
        row_step=16 * N,
        data=buf.tobytes(),
    )


# ── 三视图热图 ───────────────────────────────────────────────────────

def save_heatmap(pts, output_path, base_z=0.70, base_roll=math.pi):
    """
    生成并保存俯视(XY)、正视(XZ)、侧视(YZ) 三视图 log-密度热图。
    """
    import matplotlib
    matplotlib.use('Agg')   # 无 GUI 后端
    import matplotlib.pyplot as plt

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    title = ("Inverted mount: base_z={:.2f} m,  base_roll={:.4f} rad ({:.0f}°)  |  "
             "N={:,} samples").format(base_z, base_roll,
                                      math.degrees(base_roll), len(pts))

    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle(title, fontsize=11, y=1.01)

    views = [
        (x, y, "Top View  (XY Plane)",   "X  [m]", "Y  [m]"),
        (x, z, "Front View  (XZ Plane)", "X  [m]", "Z  [m]"),
        (y, z, "Side View  (YZ Plane)",  "Y  [m]", "Z  [m]"),
    ]

    for ax, (u, v, title_v, xlabel, ylabel) in zip(axes, views):
        bins = 300
        h, xe, ye = np.histogram2d(u, v, bins=bins)
        h_log = np.log1p(h)   # log-scale：突出稀疏区域

        im = ax.imshow(
            h_log.T, origin='lower',
            extent=[xe[0], xe[-1], ye[0], ye[-1]],
            aspect='auto', cmap='inferno',
        )
        ax.set_title(title_v, fontsize=11)
        ax.set_xlabel(xlabel, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.axhline(0, color='#00e5ff', lw=0.7, alpha=0.8)
        ax.axvline(0, color='#00e5ff', lw=0.7, alpha=0.8)
        ax.grid(True, color='white', alpha=0.08, lw=0.4)
        cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label('log(count+1)', fontsize=8)

        # ── 轴范围统计标注 ──
        ax.text(0.02, 0.97,
                "u∈[{:.2f},{:.2f}]\nv∈[{:.2f},{:.2f}]".format(
                    u.min(), u.max(), v.min(), v.max()),
                transform=ax.transAxes, fontsize=7,
                va='top', color='white', alpha=0.75,
                bbox=dict(boxstyle='round,pad=0.2', fc='black', alpha=0.4))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)


# ── 主函数 ───────────────────────────────────────────────────────────

def main():
    rospy.init_node("workspace_cloud", anonymous=False)

    # ── 参数读取 ──
    n_samples  = int(rospy.get_param("~n_samples",  150000))
    output_dir = rospy.get_param("~output_dir", os.path.expanduser("~"))
    frame_id   = rospy.get_param("~frame_id",   "world")

    # 基座变换参数（由 open_manipulator_upload.launch 通过 xacro 解析后
    # 写入 robot_description，但 xacro 参数本身不入参数服务器；
    # sim_scene.launch 在 arg 级别存在，需要显式在 launch 里 set，
    # 这里通过默认值与 sim_scene 一致，并允许命令行覆盖）
    base_x     = float(rospy.get_param("~base_x",     0.0))
    base_y     = float(rospy.get_param("~base_y",     0.0))
    base_z     = float(rospy.get_param("~base_z",     0.70))
    base_roll  = float(rospy.get_param("~base_roll",  math.pi))
    base_pitch = float(rospy.get_param("~base_pitch", 0.0))
    base_yaw   = float(rospy.get_param("~base_yaw",   0.0))

    # ── 发布者 ──
    pub = rospy.Publisher("/workspace_cloud", PointCloud2, queue_size=1, latch=True)

    # ── 构建 world→link1 变换 ──
    # 注意：urdf.xacro 的 rpy 用固定轴 XYZ 顺序（extrinsic），即先 Roll 后 Pitch 后 Yaw
    T_world_base = (_trans(base_x, base_y, base_z)
                    @ _rot_x(base_roll)
                    @ _rot_y(base_pitch)
                    @ _rot_z(base_yaw))

    rospy.loginfo("=" * 60)
    rospy.loginfo("[workspace_cloud] Base transform:")
    rospy.loginfo("  xyz = (%.3f, %.3f, %.3f)", base_x, base_y, base_z)
    rospy.loginfo("  rpy = (%.4f, %.4f, %.4f) rad", base_roll, base_pitch, base_yaw)
    rospy.loginfo("[workspace_cloud] Sampling %d joint configurations ...", n_samples)

    # ── 关节空间均匀随机采样 ──
    np.random.seed(0)
    q = np.column_stack([
        np.random.uniform(lo, hi, n_samples)
        for lo, hi in JOINT_LIMITS
    ])

    # ── 批量 FK ──
    t0 = time.time()
    pts = compute_fk(q, T_world_base)
    dt_fk = time.time() - t0
    rospy.loginfo("[workspace_cloud] FK computed: %d pts in %.2f s  (%.0f kpts/s)",
                  len(pts), dt_fk, len(pts) / dt_fk / 1000.0)
    rospy.loginfo("[workspace_cloud] X ∈ [%.3f, %.3f]  Y ∈ [%.3f, %.3f]  Z ∈ [%.3f, %.3f]",
                  pts[:,0].min(), pts[:,0].max(),
                  pts[:,1].min(), pts[:,1].max(),
                  pts[:,2].min(), pts[:,2].max())

    # ── 发布 PointCloud2 ──
    cloud_msg = make_cloud_msg(pts, frame_id=frame_id)
    pub.publish(cloud_msg)
    rospy.loginfo("[workspace_cloud] PointCloud2 published (latched) → /workspace_cloud")

    # ── 保存热图 ──
    output_path = os.path.join(output_dir, "workspace_cloud.png")
    rospy.loginfo("[workspace_cloud] Saving heatmap → %s", output_path)
    try:
        save_heatmap(pts, output_path, base_z=base_z, base_roll=base_roll)
        rospy.loginfo("[workspace_cloud] Heatmap saved ✓")
    except ImportError as e:
        rospy.logwarn("[workspace_cloud] matplotlib not available, skipping PNG: %s", e)

    rospy.loginfo("=" * 60)
    rospy.loginfo("[workspace_cloud] Ready. PointCloud2 stays latched until node exits.")
    rospy.loginfo("  RViz: Add → PointCloud2 → Topic: /workspace_cloud")
    rospy.loginfo("        Color Transformer: AxisColor  |  Axis: Z  |  Size: 0.003 m")
    rospy.loginfo("=" * 60)

    rospy.spin()


if __name__ == "__main__":
    main()

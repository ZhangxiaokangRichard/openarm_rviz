#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
scene_marker_pub.py
───────────────────
发布充电站和汽车的 MESH_RESOURCE MarkerArray（latched）。
模型原点坐标和姿态通过 ROS 参数传入，可在 launch 文件中覆盖。

话题：~scene_markers  (visualization_msgs/MarkerArray, latched)

参数（私有命名空间 ~）：
  cs_x, cs_y, cs_z          充电站原点坐标 (m)，默认 0.0
  cs_roll, cs_pitch, cs_yaw 充电站姿态 (rad)，默认 0.0
  car_x, car_y, car_z       汽车原点坐标 (m)，默认 0.0
  car_roll, car_pitch, car_yaw  汽车姿态 (rad)，默认 0.0

STL 单位为毫米，scale 统一设置为 0.001 (mm→m)。
"""
import math
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA


def euler_to_quaternion(roll, pitch, yaw):
    """固定轴 XYZ 顺序 (extrinsic) 欧拉角 → 四元数。"""
    cr = math.cos(roll  / 2.0)
    sr = math.sin(roll  / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw   / 2.0)
    sy = math.sin(yaw   / 2.0)

    return Quaternion(
        x = sr * cp * cy - cr * sp * sy,
        y = cr * sp * cy + sr * cp * sy,
        z = cr * cp * sy - sr * sp * cy,
        w = cr * cp * cy + sr * sp * sy,
    )


def make_mesh_marker(ns, marker_id,
                     mesh_pkg_path,
                     x, y, z,
                     roll, pitch, yaw,
                     r, g, b, a=0.9,
                     scale=0.001):
    """
    构建一个 MESH_RESOURCE 类型的 Marker。

    :param mesh_pkg_path: e.g. "package://fuel_gun_grasp/meshes/car.stl"
    :param scale:         统一缩放因子（STL 单位 mm 时传 0.001）
    """
    m = Marker()
    m.header.frame_id = "world"
    m.header.stamp    = rospy.Time(0)
    m.ns     = ns
    m.id     = marker_id
    m.type   = Marker.MESH_RESOURCE
    m.action = Marker.ADD

    m.pose.position    = Point(x, y, z)
    m.pose.orientation = euler_to_quaternion(roll, pitch, yaw)

    m.scale = Vector3(scale, scale, scale)
    m.color = ColorRGBA(r, g, b, a)
    m.mesh_resource = mesh_pkg_path
    m.mesh_use_embedded_materials = False   # STL 无内嵌材质，用 color 字段着色

    m.lifetime = rospy.Duration(0)          # 永久显示
    return m


def main():
    rospy.init_node("scene_marker_pub")

    # ── 充电站参数 ──
    cs_x     = rospy.get_param("~cs_x",     0.0)
    cs_y     = rospy.get_param("~cs_y",     0.0)
    cs_z     = rospy.get_param("~cs_z",     0.0)
    cs_roll  = rospy.get_param("~cs_roll",  0.0)
    cs_pitch = rospy.get_param("~cs_pitch", 0.0)
    cs_yaw   = rospy.get_param("~cs_yaw",   0.0)

    # ── 汽车参数 ──
    car_x     = rospy.get_param("~car_x",     0.0)
    car_y     = rospy.get_param("~car_y",     0.0)
    car_z     = rospy.get_param("~car_z",     0.0)
    car_roll  = rospy.get_param("~car_roll",  0.0)
    car_pitch = rospy.get_param("~car_pitch", 0.0)
    car_yaw   = rospy.get_param("~car_yaw",   0.0)

    pub = rospy.Publisher("~scene_markers", MarkerArray, queue_size=1, latch=True)

    ma = MarkerArray()

    # 充电站：蓝灰色
    ma.markers.append(make_mesh_marker(
        ns="charging_station", marker_id=0,
        mesh_pkg_path="package://fuel_gun_grasp/meshes/charging_station.stl",
        x=cs_x, y=cs_y, z=cs_z,
        roll=cs_roll, pitch=cs_pitch, yaw=cs_yaw,
        r=0.25, g=0.40, b=0.65, a=0.9,
    ))

    # 汽车：红色
    ma.markers.append(make_mesh_marker(
        ns="car", marker_id=0,
        mesh_pkg_path="package://fuel_gun_grasp/meshes/car.stl",
        x=car_x, y=car_y, z=car_z,
        roll=car_roll, pitch=car_pitch, yaw=car_yaw,
        r=0.75, g=0.15, b=0.15, a=0.9,
    ))

    # 以 20 Hz 持续发布：latched publisher 保证迟到的 RViz 订阅者收到缓存，
    # 持续发布则保证大面数 STL 在 RViz 重连或刷新时能立即显示
    rate = rospy.Rate(20)
    rospy.loginfo("[scene_marker_pub] Publishing %d mesh markers at 20 Hz ...",
                  len(ma.markers))
    while not rospy.is_shutdown():
        pub.publish(ma)
        rate.sleep()


if __name__ == "__main__":
    main()

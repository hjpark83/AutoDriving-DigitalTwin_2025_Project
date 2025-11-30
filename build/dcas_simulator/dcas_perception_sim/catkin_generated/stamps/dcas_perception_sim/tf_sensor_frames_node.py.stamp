#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math


def MakeTform(parent: str, child: str, x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    # rpy -> quaternion
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy
    return t


def Main() -> None:
    rospy.init_node("tf_sensor_frames_node")
    br = tf2_ros.StaticTransformBroadcaster()

    parent = rospy.get_param("~parent_frame", rospy.get_param("/tf_sensor_frames_node/parent_frame", "ego_frame"))
    frames = rospy.get_param("~frames", rospy.get_param("/tf_sensor_frames_node/frames", []))

    transforms = []
    if isinstance(frames, list) and frames:
        for f in frames:
            child = f.get("child")
            xyz = f.get("xyz", [0.0, 0.0, 0.0])
            rpy = f.get("rpy", [0.0, 0.0, 0.0])
            p = f.get("parent", parent)
            if child is None:
                rospy.logwarn("tf_sensor_frames_node: skipping frame without child name")
                continue
            transforms.append(MakeTform(p, child, float(xyz[0]), float(xyz[1]), float(xyz[2]), float(rpy[0]), float(rpy[1]), float(rpy[2])))
    else:
        # Fallback defaults
        defaults = [
            (parent, "lidar", 1.20, 0.00, 1.50, 0.0, 0.0, 0.0),
            (parent, "camera", 1.10, 0.00, 1.40, 0.0, 0.0, 0.0),
            (parent, "radar",  1.00, 0.00, 0.50, 0.0, 0.0, 0.0),
            (parent, "imu",    0.00, 0.00, 0.80, 0.0, 0.0, 0.0),
            (parent, "gnss",   0.00, 0.00, 1.70, 0.0, 0.0, 0.0),
        ]
        transforms = [MakeTform(*args) for args in defaults]

    br.sendTransform(transforms)
    rospy.loginfo("tf_sensor_frames_node: published static transforms for %d frames", len(transforms))
    rospy.spin()


if __name__ == "__main__":
    Main()



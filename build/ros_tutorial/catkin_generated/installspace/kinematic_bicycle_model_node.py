#!/usr/bin/env python3
import threading
import sys
import math
import rospy
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf_conversions
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ros_tutorial.msg import TutorialVehicleState

class KinematicBicycleModelNode:
    def __init__(self):
        rospy.init_node("kinematic_bicycle_model_node", anonymous=False)

        self.steering_topic_name = rospy.get_param("~steering_command_topic", "/steering_cmd")
        self.vehicle_state_topic_name = rospy.get_param("~vehicle_state_topic", "/vehicle/state")
        self.marker_topic_name = rospy.get_param("~vehicle_marker_topic", "/vehicle/marker")

        self.speed = rospy.get_param("~initial_speed_mps", 5.0)
        self.wheel_base = rospy.get_param("~wheel_base", 2.7)

        # 상태
        self.x = rospy.get_param("~initial_x", 0.0)
        self.y = rospy.get_param("~initial_y", 0.0)
        self.yaw = rospy.get_param("~initial_yaw", 0.0)
        self.steering_angle = 0.0

        self.last_time = rospy.get_time()

        # Marker 
        self.state_publisher = rospy.Publisher(self.vehicle_state_topic_name, TutorialVehicleState, queue_size=10)
        self.marker_publisher = rospy.Publisher(self.marker_topic_name, Marker, queue_size=10)
        self.subscriber = rospy.Subscriber(self.steering_topic_name, Float64, self.OnSteeringCommand)

        self.loop_rate_hz = rospy.get_param("~loop_rate_hz", 50.0)

        self.input_thread_should_run = True
        self.input_thread = threading.Thread(target=self.HandleUserInput)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.vehicle_frame_id = rospy.get_param("~vehicle_frame_id", "vehicle")

    def OnSteeringCommand(self, message):
        self.steering_angle = float(message.data)

    def HandleUserInput(self):
        usage = (
            "\n[KinematicVehicleModelNode] Command Examples:\n"
            "  speed 8.0          # m/s\n"
            "  wheelbase 2.8      # m (reflect in runtimes)\n"
            "  show\n"
        )
        sys.stdout.write(usage)
        sys.stdout.flush()

        while self.input_thread_should_run and not rospy.is_shutdown():
            line = sys.stdin.readline()
            if not line:
                rospy.sleep(0.1)
                continue
            tokens = line.strip().split()
            if len(tokens) == 0:
                continue

            command = tokens[0].lower()
            try:
                if command == "speed" and len(tokens) == 2:
                    value = float(tokens[1])
                    self.speed = value
                    rospy.loginfo("speed update: %.6f m/s", self.speed)

                elif command == "wheelbase" and len(tokens) == 2:
                    value = float(tokens[1])
                    if value <= 0.0:
                        rospy.logwarn("wheel_base must be positive.")
                    else:
                        self.wheel_base = value
                        rospy.loginfo("wheel_base update: %.6f m", self.wheel_base)

                elif command == "show":
                    rospy.loginfo("current settings | speed: %.6f m/s, wheel_base: %.6f m", self.speed, self.wheel_base)
                else:
                    rospy.logwarn("unknown command: %s", line.strip())
                    sys.stdout.write(usage)
                    sys.stdout.flush()
            except ValueError:
                rospy.logwarn("invalid numeric format: %s", line.strip())

    # TODO: 자전거 모델 방정식 완성
    def VehicleStateUpdate(self, dt):
        # 자전거 모델
        v = self.speed
        yaw = self.yaw
        steering_angle = self.steering_angle
        wheel_base = self.wheel_base
        
        self.x = self.x + v * math.cos(yaw) * dt
        self.y = self.y + v * math.sin(yaw) * dt

        yaw_rate = v * math.tan(steering_angle) / wheel_base * dt

        self.yaw = yaw + yaw_rate

    # TODO: 차량 상태 메시지 발행
    def PublishState(self):
        message = TutorialVehicleState()
        message.x = self.x
        message.y = self.y
        message.yaw = self.yaw
        message.steering_angle = self.steering_angle
        self.state_publisher.publish(message)

    def PublishMarker(self):
        marker = Marker()
        marker.header.frame_id = rospy.get_param("~frame_id", "map")
        marker.header.stamp = rospy.Time.now()
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # 중심 기준 박스
        length = self.wheel_base + 1.0  # 뒤 0.5 + 앞 0.5
        width = 2.2
        height = 2.5

        marker.scale.x = length
        marker.scale.y = width
        marker.scale.z = height

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = height * 0.5

        half_yaw = self.yaw * 0.5
        marker.pose.orientation.z = math.sin(half_yaw)
        marker.pose.orientation.w = math.cos(half_yaw)

        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.9

        marker.lifetime = rospy.Duration(0.0)
        self.marker_publisher.publish(marker)

    def PublishTf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = rospy.get_param("~frame_id", "map")
        t.child_frame_id = self.vehicle_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def Run(self):
        rate = rospy.Rate(self.loop_rate_hz)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            dt = max(0.0, now - self.last_time)
            self.last_time = now

            self.VehicleStateUpdate(dt)
            self.PublishState()
            self.PublishMarker()
            self.PublishTf()     
            rate.sleep()

if __name__ == "__main__":
    node = KinematicBicycleModelNode()
    node.Run()

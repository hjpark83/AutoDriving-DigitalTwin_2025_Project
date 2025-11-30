#!/usr/bin/env python
"""
환경: 차선 퍼블리셔 (간단한 합성 지오메트리)

- CSV에서 차선 중심선과 타입을 읽어 /env/lanes 로 퍼블리시합니다.
- RViz에는 LINE_STRIP 마커로 시각화하며, 타입에 따라 색/두께를 다르게 표현합니다.
"""
import rospy
import math
from dcas_msgs.msg import Lane, LaneArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32, Point
import random
import os
import csv




class EnvLanePublisher:
    """Publishes simple synthetic lane geometry as ground-truth.

    This is intentionally minimal: it generates parallel lane boundaries
    along X with mild curvature for demonstration.
    """

    def __init__(self) -> None:
        # Parameters
        self.csv_path = rospy.get_param("/map/lanes_csv", "lanes.csv")
        period = float(rospy.get_param("/env_lane_node/period_s", 0.1))

        # Publishers
        self.publisher = rospy.Publisher("/env/lanes", LaneArray, queue_size=10)
        self.viz_pub = rospy.Publisher("/viz/env/lanes", MarkerArray, queue_size=10)

        # Timer
        self.duration = period
        self.timer = rospy.Timer(rospy.Duration(period), self.CallbackTimer)

        # CSV path from config
        self._csv_warned = False

    # 주기 콜백
    # - CSV에서 차선을 읽어 퍼블리시하고, 동시에 RViz 마커도 퍼블리시
    def CallbackTimer(self, _event) -> None:
        """주기적으로 CSV를 읽어 차선 메시지/마커를 퍼블리시합니다."""
        # Create a LaneArray message
        msg = LaneArray()
        msg.header.stamp = rospy.Time.now()

        # Resolve full path for warning
        full = self.csv_path
        if not os.path.isabs(full):
            pkg_dir = os.path.dirname(os.path.dirname(__file__))
            base_dir = os.path.abspath(os.path.join(pkg_dir, "maps"))
            full = os.path.join(base_dir, self.csv_path)
        lanes_csv = self.ReadLanesFromCsv(self.csv_path)
        if lanes_csv:
            msg.lanes = lanes_csv
        else:
            msg.lanes = []
            if not self._csv_warned:
                rospy.logwarn("env_lane_node: CSV not found or empty: %s", full)
                self._csv_warned = True
        
        # Publish the LaneArray message
        self.publisher.publish(msg)
        
        # Publish the markers
        marker_arr = self.LanesToMarkers(msg)
        self.viz_pub.publish(marker_arr)

    # CSV에서 차선 폴리라인과 타입을 읽어 Lane 리스트 생성
    # - 스키마: lane_id, x, y, (optional z), lane_type
    def ReadLanesFromCsv(self, path: str):
        """CSV에서 차선 폴리라인과 타입을 읽어 리스트로 반환합니다.

        인자:
            path: CSV 파일 경로
        """
        # Check if the path is absolute
        full = path
        if not os.path.isabs(full):
            pkg_dir = os.path.dirname(os.path.dirname(__file__))
            base_dir = os.path.abspath(os.path.join(pkg_dir, "maps"))
            full = os.path.join(base_dir, path)
        if not os.path.exists(full):
            return []
        
        # Polyline per lane with lane type
        lanes_map = {}  # lane_id -> {'points': List[Point32], 'lane_type': int}
        with open(full, newline='') as f:
            reader = csv.DictReader(f)
            headers = [h.strip().lower() for h in (reader.fieldnames or [])]
            for row in reader:
                try:
                    lane_id = int(row.get('lane_id', 0))
                    x = float(row['x'])
                    y = float(row['y'])
                    z = float(row.get('z', 0.0))
                    lane_type = int(row.get('lane_type', 0))  # Default to TYPE_UNKNOWN
                except Exception:
                    continue
                
                if lane_id not in lanes_map:
                    lanes_map[lane_id] = {'points': [], 'lane_type': lane_type}
                lanes_map[lane_id]['points'].append(Point32(x=x, y=y, z=z))

        # Create a LaneArray message
        lanes = []
        for lid, lane_data in sorted(lanes_map.items()):
            lane = Lane()
            lane.id = lid
            lane.lane_lines = lane_data['points']  # Center points
            lane.lane_type = lane_data['lane_type']  # Lane type
            lanes.append(lane)
        return lanes

    # LaneArray를 RViz MarkerArray로 변환
    # - 각 차선을 LINE_STRIP로 표현, 타입별 두께/색상 지정
    def LanesToMarkers(self, msg: LaneArray) -> MarkerArray:
        """LaneArray를 RViz MarkerArray로 변환합니다.

        인자:
            msg: 차선 배열(LaneArray)
        """
        arr = MarkerArray()
        marker_id = 0
        for lane in msg.lanes:
            m = self.MakeMarker(ns="lanes_center", mid=marker_id, mtype=Marker.LINE_STRIP)
            marker_id += 1
            
            # Set line thickness based on lane type
            if lane.lane_type == 5:  # TYPE_DOUBLE_YELLOW_SOLID
                m.scale.x = 0.15  # Thicker for double lines
            else:
                m.scale.x = 0.1
            
            # Set color based on lane type
            if lane.lane_type == 1:  # TYPE_WHITE_SOLID
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0  # white
            elif lane.lane_type == 2:  # TYPE_WHITE_DASHED
                m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.8, 0.8, 1.0  # light gray for dashed
            elif lane.lane_type == 3:  # TYPE_YELLOW_SOLID
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0  # yellow
            elif lane.lane_type == 4:  # TYPE_YELLOW_DASHED
                m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.8, 0.0, 1.0  # dark yellow for dashed
            elif lane.lane_type == 5:  # TYPE_DOUBLE_YELLOW_SOLID
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 1.0  # orange-yellow for double
            else:  # TYPE_UNKNOWN or other
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 1.0  # gray
            
            m.points = [Point(x=p.x, y=p.y, z=p.z) for p in lane.lane_lines]
            arr.markers.append(m)
        return arr

    # RViz에서 사용할 차선 Marker 생성
    # - 입력: 네임스페이스, 마커 ID, 타입, 프레임
    # - 출력: 기본 설정이 적용된 Marker
    def MakeMarker(self, ns: str, mid: int, mtype: int, frame: str = "map") -> Marker:
        """차선 시각화를 위한 마커를 생성합니다.

        인자:
            ns: RViz 네임스페이스
            mid: 마커 ID
            mtype: 마커 타입
            frame: 기준 프레임
        """
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = rospy.Duration(self.duration)
        return m


def Main() -> None:
    # Initialize the node
    rospy.init_node("env_lane_node")

    # Create an instance of the EnvLanePublisher class
    EnvLanePublisher()

    # Log the start of the node
    rospy.loginfo("env_lane_node started: publishing /env/lanes")
    rospy.spin()


if __name__ == "__main__":
    Main()



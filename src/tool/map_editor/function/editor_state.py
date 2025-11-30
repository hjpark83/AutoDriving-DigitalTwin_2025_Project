# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_state.py
from dataclasses import dataclass

# ---- Lane type enum mapping (name ↔ code) ----
LANE_TYPE_NAME_TO_CODE = {
    "TYPE_UNKNOWN": 0,
    "TYPE_WHITE_SOLID": 1,
    "TYPE_WHITE_DASHED": 2,
    "TYPE_YELLOW_SOLID": 3,
    "TYPE_YELLOW_DASHED": 4,
    "TYPE_DOUBLE_YELLOW_SOLID": 5,
}
LANE_TYPE_CODE_TO_NAME = {v: k for k, v in LANE_TYPE_NAME_TO_CODE.items()}

# (선택) UI에서 쓸 수 있도록 옵션 튜플도 제공
LANE_TYPE_OPTIONS = [
    ("TYPE_UNKNOWN", 0),
    ("TYPE_WHITE_SOLID", 1),
    ("TYPE_WHITE_DASHED", 2),
    ("TYPE_YELLOW_SOLID", 3),
    ("TYPE_YELLOW_DASHED", 4),
    ("TYPE_DOUBLE_YELLOW_SOLID", 5),
]

# ---- Lanes ----
@dataclass
class LanePolyline:
    lane_id: int
    lane_type: int          # CSV에 숫자 코드 저장 (0~5)
    points_xyz: list        # [(x, y, z), ...]

# ---- Buildings ----
@dataclass
class BuildingPolygon:
    building_id: int
    center_xy: tuple     # (x, y)
    vertices_xy: list    # [(x, y), ...]
    height: float
    rotation_deg: float

# ---- Guardrails ----
@dataclass
class GuardrailItem:
    cx: float
    cy: float
    length: float
    yaw_deg: float

# ---- Signs ----
@dataclass
class SignItem:
    x: float
    y: float
    z: float
    yaw: float
    sign_class: int  # CSV에 숫자 코드

# ---- Traffic Lights ----
@dataclass
class TrafficLightItem:
    x: float
    y: float
    z: float
    state: int  # 0 red, 1 yellow, 2 green

# ---- GNSS Shadow Zones ----
@dataclass
class GNSSShadowZone:
    x: float
    y: float
    pos_std: float   # 위치 표준편차(오차) [m]
    radius: float    # 영향 반경 [m]

# ---- Surrounding Vehicles ----
@dataclass
class SurroundingVehicleTrajectory:
    vehicle_id: int
    points_xy: list      # [(x, y), ...]
    speed_mps: float     # 고정 속도 (CSV에 그대로 기록)

class EditorState:
    def __init__(self):
        # 공통
        self.mode = "lanes"
        self.draw_handles = {
            "lanes": [],
            "buildings": [],
            "guardrails": [],
            "signs": [],
            "traffic_lights": [],
            "gnss_shadow_zones": [],
            "surrounding_vehicles": [],     # ★ 추가
        }

        # Lanes
        self.lanes = []
        self.next_lane_id = 1
        self.lanes_submode = "view"
        self.current_lane_type = 0  # enum code (0~5)
        self.current_lane_points_xyz = []
        self.selected_lane_index = None
        self.selected_vertex_index = None

        self.lanes_modify_mode = "node"   # "node" | "lane"
        self.lanes_delete_mode = "node"   # "node" | "lane"

        # Buildings
        self.buildings = []
        self.next_building_id = 1
        self.buildings_submode = "view"
        self.selected_building_index = None
        self.selected_building_vertex_index = None
        self.building_create_vertices_count = 4
        self.building_create_vertex_radius = 10.0
        self.building_create_height = 3.0

        # Guardrails
        self.guardrails: list[GuardrailItem] = []
        self.guardrails_submode = "view"
        self.selected_guardrail_index = None
        self.guardrails_create_length = 2.0
        self.guardrails_create_yaw_deg = 0.0

        # Signs
        self.signs: list[SignItem] = []
        self.signs_submode = "view"
        self.selected_sign_index = None
        self.signs_create_class = 0
        self.signs_create_z = 1.0
        self.signs_create_yaw = 90.0

        # Traffic Lights
        self.traffic_lights: list[TrafficLightItem] = []
        self.traffic_lights_submode = "view"
        self.selected_traffic_light_index = None
        self.traffic_lights_create_state = 0
        self.traffic_lights_create_z = 0.0

        # GNSS Shadow Zones
        self.gnss_shadow_zones: list[GNSSShadowZone] = []
        self.gnss_shadow_zones_submode = "view"
        self.selected_gnss_shadow_zone_index = None
        self.gnss_shadow_zones_create_pos_std = 1.0
        self.gnss_shadow_zones_create_radius = 10.0

        # ---- Surrounding Vehicles ----
        self.surrounding_vehicles: list[SurroundingVehicleTrajectory] = []
        self.surrounding_vehicles_submode = "view"
        self.surrounding_vehicles_modify_mode = "node"   # "node" | "vehicle"
        self.surrounding_vehicles_delete_mode = "node"   # "node" | "vehicle"
        self.current_surrounding_vehicle_points_xy = []  # Create 중 임시 포인트들
        self.surrounding_vehicles_create_speed_mps = 5.0
        self.selected_surrounding_vehicle_index = None
        self.selected_surrounding_vehicle_vertex_index = None
        self.next_surrounding_vehicle_id = 1

        # 저장 위치
        self.results_root_dir = "../results"        
        self.active_map_folder_name = "map_1"

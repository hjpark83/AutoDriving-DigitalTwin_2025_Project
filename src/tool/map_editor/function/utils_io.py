# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/utils_io.py
import os
import csv

# ----- 안전 변환 유틸 (문자열/숫자 모두 수용) -----

SIGN_NAME_TO_CODE = {
    "CLASS_UNKNOWN": 0,
    "CLASS_STOP": 1,
    "CLASS_YIELD": 2,
    "CLASS_SPEED_LIMIT_30": 3,
    "CLASS_SPEED_LIMIT_50": 4,
    "CLASS_SPEED_LIMIT_60": 5,
    "CLASS_SPEED_LIMIT_80": 6,
    "CLASS_NO_ENTRY": 7,
    "CLASS_NO_PARKING": 8,
    "CLASS_PEDESTRIAN_CROSSING": 9,
    "CLASS_SCHOOL_ZONE": 10,
    "CLASS_CONSTRUCTION": 11,
    "CLASS_TURN_LEFT": 12,
    "CLASS_TURN_RIGHT": 13,
    "CLASS_GO_STRAIGHT": 14,
}

# ✅ 추가: 폴리곤 서명 면적(>0 이면 CCW)
def _polygon_signed_area(verts_xy):
    if len(verts_xy) < 3:
        return 0.0
    area = 0.0
    n = len(verts_xy)
    for i in range(n):
        x1, y1 = verts_xy[i]
        x2, y2 = verts_xy[(i + 1) % n]
        area += (x1 * y2 - x2 * y1)
    return 0.5 * area

# ✅ 추가: 시계방향 강제 (표준 좌표계에서 면적>0 이면 CCW이므로 reverse)
def _ensure_clockwise(verts_xy):
    if len(verts_xy) < 3:
        return list(verts_xy)
    return list(reversed(verts_xy)) if _polygon_signed_area(verts_xy) > 0 else list(verts_xy)

def ToSignClassCode(value) -> int:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, str):
        s = value.strip()
        if s.isdigit():
            return int(s)
        su = s.upper()
        if su in SIGN_NAME_TO_CODE:
            return SIGN_NAME_TO_CODE[su]
        if not su.startswith("CLASS_"):
            alt = "CLASS_" + su
            return SIGN_NAME_TO_CODE.get(alt, 0)
    return 0

def ToTrafficLightStateCode(value) -> int:
    """red/yellow/green 또는 0/1/2 문자열/숫자를 모두 0,1,2로 매핑"""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        v = int(value)
        return 0 if v < 0 else (2 if v > 2 else v)
    if isinstance(value, str):
        s = value.strip().lower()
        if s.isdigit():
            v = int(s)
            return 0 if v < 0 else (2 if v > 2 else v)
        if s in ("red", "r"):
            return 0
        if s in ("yellow", "amber", "y", "a", "orange"):
            return 1
        if s in ("green", "g"):
            return 2
    return 0


def GetNextMapFolderName(root_dir):
    base = "map_"
    n = 1
    while os.path.exists(os.path.join(root_dir, f"{base}{n}")):
        n += 1
    return f"{base}{n}"


def SaveMapBundle(editor_state):
    out_dir = os.path.join(editor_state.results_root_dir, editor_state.active_map_folder_name)
    os.makedirs(out_dir, exist_ok=True)

    # lanes.csv
    with open(os.path.join(out_dir, "lanes.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lane_id", "x", "y", "z", "lane_type"])
        for lane in editor_state.lanes:
            for (x, y, z) in lane.points_xyz:
                # lane_type은 이미 코드(int)일 것으로 가정하되, 방어적으로 int()
                w.writerow([lane.lane_id, x, y, z, int(lane.lane_type)])

    # buildings.csv (꼭짓점 1회씩 기록)
    # with open(os.path.join(out_dir, "buildings.csv"), "w", newline="") as f:
    #     w = csv.writer(f)
    #     w.writerow(["id", "x", "y", "h"])
    #     for b in editor_state.buildings:
    #         if len(b.vertices_xy) >= 3:
    #             for (x, y) in b.vertices_xy:
    #                 w.writerow([b.building_id, x, y, b.height])
    with open(os.path.join(out_dir, "buildings.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "x", "y", "h"])
        for b in editor_state.buildings:
            if len(b.vertices_xy) >= 3:
                # ✅ 저장 직전에 시계방향 보정
                verts = _ensure_clockwise(b.vertices_xy)
                for (x, y) in verts:
                    w.writerow([b.building_id, x, y, b.height])

    # guardrails.csv
    with open(os.path.join(out_dir, "guardrails.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cx", "cy", "length", "yaw_deg"])
        for g in editor_state.guardrails:
            # g.cx/g.cy를 사용하는 현재 데이터클래스에 맞춤
            w.writerow([g.cx, g.cy, g.length, g.yaw_deg])

    # signs.csv  ← 안전 변환 사용
    with open(os.path.join(out_dir, "signs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "yaw", "sign_class"])
        for s in editor_state.signs:
            w.writerow([s.x, s.y, s.z, s.yaw, ToSignClassCode(s.sign_class)])

    # traffic_lights.csv  ← 안전 변환 사용
    with open(os.path.join(out_dir, "traffic_lights.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "state"])
        for t in editor_state.traffic_lights:
            w.writerow([t.x, t.y, t.z, ToTrafficLightStateCode(t.state)])

    # ★ NEW: gnss_shadow_zones.csv
    with open(os.path.join(out_dir, "gnss_shadow_zones.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "pos_std", "radius"])
        for z in getattr(editor_state, "gnss_shadow_zones", []):
            w.writerow([z.x, z.y, z.pos_std, z.radius])
    
    # ★ surrounding_vehicles.csv
    with open(os.path.join(out_dir, "surrounding_vehicles.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "seq", "x", "y", "speed_mps"])
        for v in editor_state.surrounding_vehicles:
            speed = float(v.speed_mps)
            for seq, (x, y) in enumerate(v.points_xy):
                w.writerow([int(v.vehicle_id), int(seq), float(x), float(y), speed])

    return out_dir

def SaveMapBundleSelective(editor_state, out_dir, which=None):
    """
    which: {"lanes","buildings","guardrails","signs","traffic_lights","gnss_shadow_zones","surrounding_vehicles"}
    없으면 전부 저장.
    """
    os.makedirs(out_dir, exist_ok=True)
    which = set(which or {
        "lanes","buildings","guardrails","signs","traffic_lights","gnss_shadow_zones","surrounding_vehicles"
    })

    # lanes.csv
    if "lanes" in which:
        with open(os.path.join(out_dir, "lanes.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["lane_id", "x", "y", "z", "lane_type"])
            for lane in getattr(editor_state, "lanes", []):
                for (x, y, z) in lane.points_xyz:
                    w.writerow([lane.lane_id, x, y, z, int(lane.lane_type)])

    # buildings.csv
    if "buildings" in which:
        with open(os.path.join(out_dir, "buildings.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "x", "y", "h"])
            for b in getattr(editor_state, "buildings", []):
                if len(b.vertices_xy) >= 3:
                    for (x, y) in b.vertices_xy:
                        w.writerow([b.building_id, x, y, b.height])

    # guardrails.csv
    if "guardrails" in which:
        with open(os.path.join(out_dir, "guardrails.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["cx", "cy", "length", "yaw_deg"])
            for g in getattr(editor_state, "guardrails", []):
                w.writerow([g.cx, g.cy, g.length, g.yaw_deg])

    # signs.csv
    if "signs" in which:
        with open(os.path.join(out_dir, "signs.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "z", "yaw", "sign_class"])
            for s in getattr(editor_state, "signs", []):
                from utils_io import ToSignClassCode  # self import 안전
                w.writerow([s.x, s.y, s.z, s.yaw, ToSignClassCode(s.sign_class)])

    # traffic_lights.csv
    if "traffic_lights" in which:
        with open(os.path.join(out_dir, "traffic_lights.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "z", "state"])
            for t in getattr(editor_state, "traffic_lights", []):
                from utils_io import ToTrafficLightStateCode
                w.writerow([t.x, t.y, t.z, ToTrafficLightStateCode(t.state)])

    # gnss_shadow_zones.csv  (있으면 저장)
    if "gnss_shadow_zones" in which and hasattr(editor_state, "gnss_shadow_zones"):
        with open(os.path.join(out_dir, "gnss_shadow_zones.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "pos_std", "radius"])
            for z in getattr(editor_state, "gnss_shadow_zones", []):
                w.writerow([z.x, z.y, z.pos_std, z.radius])

    # surrounding_vehicles.csv (있으면 저장)
    if "surrounding_vehicles" in which and hasattr(editor_state, "surrounding_vehicles"):
        with open(os.path.join(out_dir, "surrounding_vehicles.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "seq", "x", "y", "speed_mps"])
            for ov in getattr(editor_state, "surrounding_vehicles", []):
                # 예상 데이터 구조: ov.vehicle_id, ov.points_xy, ov.speed_mps
                vid = getattr(ov, "vehicle_id", None)
                pts = getattr(ov, "points_xy", [])
                spd = float(getattr(ov, "speed_mps", 0.0))
                if vid is None:
                    # vehicle_id가 없다면 1부터 자동 부여
                    # (편의용: 목록 인덱스로 대체)
                    try:
                        vid = editor_state.surrounding_vehicles.index(ov) + 1
                    except Exception:
                        vid = 1
                for seq, (x, y) in enumerate(pts):
                    w.writerow([vid, seq, x, y, spd])

    return out_dir


def LoadMapBundle(editor_state, folder_path):
    # 지연 import: 타입 사용 시점에만 import해서 순환참조 여지 최소화
    from editor_state import LanePolyline, BuildingPolygon, GuardrailItem, SignItem, TrafficLightItem, GNSSShadowZone, SurroundingVehicleTrajectory

    # lanes
    lpath = os.path.join(folder_path, "lanes.csv")
    if os.path.exists(lpath):
        editor_state.lanes.clear()
        max_lane_id = 0
        with open(lpath, "r") as f:
            r = csv.DictReader(f)
            rows_by_lane_id = {}
            for row in r:
                lane_id = int(row["lane_id"])
                x = float(row["x"]); y = float(row["y"]); z = float(row["z"])
                lane_type = int(row["lane_type"])
                rows_by_lane_id.setdefault(lane_id, {"points": [], "lane_type": lane_type})
                rows_by_lane_id[lane_id]["points"].append((x, y, z))
                max_lane_id = max(max_lane_id, lane_id)
            for lane_id, info in sorted(rows_by_lane_id.items()):
                editor_state.lanes.append(LanePolyline(
                    lane_id=lane_id,
                    lane_type=int(info["lane_type"]),
                    points_xyz=list(info["points"])
                ))
        editor_state.next_lane_id = (max_lane_id + 1) if editor_state.lanes else 1

    # buildings
    bpath = os.path.join(folder_path, "buildings.csv")
    if os.path.exists(bpath):
        import collections
        by_id = collections.defaultdict(lambda: {"verts": [], "h": 3.0})
        with open(bpath, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                bid = int(row["id"]); x = float(row["x"]); y = float(row["y"]); h = float(row["h"])
                by_id[bid]["verts"].append((x, y)); by_id[bid]["h"] = h
        editor_state.buildings.clear()
        for bid, info in sorted(by_id.items()):
            verts = info["verts"]
            if len(verts) >= 3 and verts[0] == verts[-1]:
                verts = verts[:-1]
            # ✅ 로딩 시에도 시계방향 보정(선택)
            verts = _ensure_clockwise(verts)
            cx = sum(v[0] for v in verts) / len(verts)
            cy = sum(v[1] for v in verts) / len(verts)
            editor_state.buildings.append(BuildingPolygon(
                building_id=bid, center_xy=(cx, cy), vertices_xy=verts,
                height=info["h"], rotation_deg=0.0
            ))
        editor_state.next_building_id = (max(by_id.keys()) + 1) if by_id else 1

    # guardrails
    gpath = os.path.join(folder_path, "guardrails.csv")
    if os.path.exists(gpath):
        editor_state.guardrails.clear()
        with open(gpath, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                editor_state.guardrails.append(
                    GuardrailItem(
                        cx=float(row["cx"]),
                        cy=float(row["cy"]),
                        length=float(row["length"]),
                        yaw_deg=float(row["yaw_deg"]),
                    )
                )

    # signs
    spath = os.path.join(folder_path, "signs.csv")
    if os.path.exists(spath):
        editor_state.signs.clear()
        with open(spath, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                editor_state.signs.append(
                    SignItem(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        z=float(row["z"]),
                        yaw=float(row["yaw"]),
                        sign_class=ToSignClassCode(row["sign_class"]),
                    )
                )

    # traffic lights
    tpath = os.path.join(folder_path, "traffic_lights.csv")
    if os.path.exists(tpath):
        editor_state.traffic_lights.clear()
        with open(tpath, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                editor_state.traffic_lights.append(
                    TrafficLightItem(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        z=float(row["z"]),
                        state=ToTrafficLightStateCode(row["state"]),
                    )
                )
    
    # ★ NEW: gnss_shadow_zones
    zpath = os.path.join(folder_path, "gnss_shadow_zones.csv")
    if os.path.exists(zpath):
        editor_state.gnss_shadow_zones.clear()
        with open(zpath, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                editor_state.gnss_shadow_zones.append(
                    GNSSShadowZone(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        pos_std=float(row["pos_std"]),
                        radius=float(row["radius"]),
                    )
                )
    
    # ★ surrounding_vehicles
    ov_path = os.path.join(folder_path, "surrounding_vehicles.csv")
    if os.path.exists(ov_path):
        import collections
        by_id = collections.defaultdict(list)  # id -> [(seq, x, y, speed_mps), ...]
        with open(ov_path, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                vid = int(row["id"])
                seq = int(row["seq"])
                x = float(row["x"])
                y = float(row["y"])
                sp = float(row.get("speed_mps", 0.0))
                by_id[vid].append((seq, x, y, sp))

        editor_state.surrounding_vehicles.clear()
        max_vehicle_id = 0
        for vid, items in sorted(by_id.items()):
            items.sort(key=lambda t: t[0])  # seq 기준 정렬
            points = [(x, y) for (_seq, x, y, _sp) in items]
            # 속도는 첫 행 기준(파일이 일정하지 않아도 일관성 유지)
            speed = float(items[0][3]) if items else 0.0
            editor_state.surrounding_vehicles.append(
                SurroundingVehicleTrajectory(vehicle_id=int(vid), points_xy=points, speed_mps=speed)
            )
            if vid > max_vehicle_id:
                max_vehicle_id = vid
        editor_state.next_surrounding_vehicle_id = (max_vehicle_id + 1) if by_id else 1

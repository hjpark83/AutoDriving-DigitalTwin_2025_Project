# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_manager.py
import os
# from editor_state import EditorState
from editor_state import EditorState, LANE_TYPE_NAME_TO_CODE, LANE_TYPE_CODE_TO_NAME
from editor_base import EditorBase
from editor_lanes import LanesEditor
from editor_buildings import BuildingsEditor
from editor_guardrails import GuardrailsEditor
from editor_gnss_shadow_zones import GnssShadowZonesEditor
from editor_surrounding_vehicles import SurroundingVehiclesEditor  # ★ 추가
from editor_signs import SignsEditor
from editor_traffic_lights import TrafficLightsEditor
from utils_io import SaveMapBundle, LoadMapBundle, GetNextMapFolderName


class EditorManager:
    def __init__(self, figure, axes):
        self.figure = figure
        self.axes = axes
        self.editor_state = EditorState()
        self.editors = {
            "lanes": LanesEditor(self.editor_state, figure, axes),
            "buildings": BuildingsEditor(self.editor_state, figure, axes),
            "guardrails": GuardrailsEditor(self.editor_state, figure, axes),
            "signs": SignsEditor(self.editor_state, figure, axes),
            "traffic_lights": TrafficLightsEditor(self.editor_state, figure, axes),
            "gnss_shadow_zones": GnssShadowZonesEditor(self.editor_state, figure, axes),
            "surrounding_vehicles": SurroundingVehiclesEditor(self.editor_state, figure, axes),
        }
        self.current_editor: EditorBase = self.editors[self.editor_state.mode]

        # View
        self.lock_view = True
        self.common_view_mode = False
        self.is_panning = False
        self.last_pan_px = (0.0, 0.0)

        # callbacks
        self.on_lanes_changed = None
        self.on_buildings_changed = None
        self.on_signs_changed = None
        self.on_traffic_lights_changed = None
        self.on_gnss_shadow_zones_changed = None
        self.on_surrounding_vehicles_changed = None

        self._last_lanes_count = 0
        self._last_buildings_count = 0
        self._last_signs_count = 0
        self._last_traffic_count = 0
        self._last_gnss_shadow_zones_count = 0
        self._last_surrounding_count = 0
        self._last_surrounding_nodes_total = 0

        # lanes
        self._last_lanes_nodes_total = 0

        # callbacks
        self.on_guardrails_changed = None
        self._last_guardrails_count = 0

    def Initialize(self):
        self._ConnectEvents()
        self.current_editor.OnModeEntered()
        self._last_lanes_count = len(self.editor_state.lanes)
        self._last_buildings_count = len(self.editor_state.buildings)
        self._last_signs_count = len(self.editor_state.signs)
        self._last_traffic_count = len(self.editor_state.traffic_lights)
        self._last_gnss_shadow_zones_count = len(self.editor_state.gnss_shadow_zones)  
        self._last_surrounding_count = len(self.editor_state.surrounding_vehicles)

        self._last_lanes_nodes_total = sum(len(l.points_xyz) for l in self.editor_state.lanes)
        self._last_surrounding_nodes_total = sum(len(v.points_xy) for v in self.editor_state.surrounding_vehicles)
        # guardrails
        self._last_guardrails_count = len(self.editor_state.guardrails)

        # geometry changed 콜백 전달(있는 경우만)
        lanes = self.editors.get("lanes")
        if hasattr(lanes, "SetOnGeometryChanged"):
            lanes.SetOnGeometryChanged(self.TriggerLanesGeometryChanged)
        buildings = self.editors.get("buildings")
        if hasattr(buildings, "SetOnGeometryChanged"):
            buildings.SetOnGeometryChanged(self.TriggerBuildingsGeometryChanged)
        signs = self.editors.get("signs")
        if hasattr(signs, "SetOnGeometryChanged"):
            signs.SetOnGeometryChanged(self.TriggerSignsGeometryChanged)
        tls = self.editors.get("traffic_lights")
        if hasattr(tls, "SetOnGeometryChanged"):
            tls.SetOnGeometryChanged(self.TriggerTrafficLightsGeometryChanged)
        guard = self.editors.get("guardrails")
        if hasattr(guard, "SetOnGeometryChanged"):
            guard.SetOnGeometryChanged(self.TriggerGuardrailsGeometryChanged)

        # ★ surrounding vehicles 지오메트리 변경 콜백 전달
        ov = self.editors.get("surrounding_vehicles")
        if hasattr(ov, "SetOnGeometryChanged"):
            ov.SetOnGeometryChanged(self.TriggerSurroundingVehiclesGeometryChanged)
        
        if hasattr(self.editors.get("gnss_shadow_zones"), "SetOnGeometryChanged"):     # ← 추가
            self.editors["gnss_shadow_zones"].SetOnGeometryChanged(self.TriggerGnssShadowZonesGeometryChanged)

    # ---------- callbacks ----------
    def SetOnLanesChanged(self, cb): self.on_lanes_changed = cb
    def TriggerLanesChanged(self):
        if callable(self.on_lanes_changed): self.on_lanes_changed()

    def SetOnBuildingsChanged(self, cb): self.on_buildings_changed = cb
    def TriggerBuildingsChanged(self):
        if callable(self.on_buildings_changed): self.on_buildings_changed()

    def SetOnSignsChanged(self, cb): self.on_signs_changed = cb
    def TriggerSignsChanged(self):
        if callable(self.on_signs_changed): self.on_signs_changed()

    def SetOnTrafficLightsChanged(self, cb): self.on_traffic_lights_changed = cb
    def TriggerTrafficLightsChanged(self):
        if callable(self.on_traffic_lights_changed): self.on_traffic_lights_changed()

    def SetOnLanesGeometryChanged(self, callback):
        self.on_lanes_geometry_changed = callback
        lanes = self.editors.get("lanes")
        if hasattr(lanes, "SetOnGeometryChanged"):
            lanes.SetOnGeometryChanged(self.TriggerLanesGeometryChanged)

    def TriggerLanesGeometryChanged(self):
        if hasattr(self, "on_lanes_geometry_changed") and callable(self.on_lanes_geometry_changed):
            self.on_lanes_geometry_changed()

    def SetOnBuildingsGeometryChanged(self, callback):
        self.on_buildings_geometry_changed = callback
        buildings = self.editors.get("buildings")
        if hasattr(buildings, "SetOnGeometryChanged"):
            buildings.SetOnGeometryChanged(self.TriggerBuildingsGeometryChanged)

    def TriggerBuildingsGeometryChanged(self):
        if hasattr(self, "on_buildings_geometry_changed") and callable(self.on_buildings_geometry_changed):
            self.on_buildings_geometry_changed()

    def SetOnSignsGeometryChanged(self, callback):
        self.on_signs_geometry_changed = callback
        signs = self.editors.get("signs")
        if hasattr(signs, "SetOnGeometryChanged"):
            signs.SetOnGeometryChanged(self.TriggerSignsGeometryChanged)

    def TriggerSignsGeometryChanged(self):
        if hasattr(self, "on_signs_geometry_changed") and callable(self.on_signs_geometry_changed):
            self.on_signs_geometry_changed()

    def SetOnTrafficLightsGeometryChanged(self, callback):
        self.on_traffic_lights_geometry_changed = callback
        tls = self.editors.get("traffic_lights")
        if hasattr(tls, "SetOnGeometryChanged"):
            tls.SetOnGeometryChanged(self.TriggerTrafficLightsGeometryChanged)

    def TriggerTrafficLightsGeometryChanged(self):
        if hasattr(self, "on_traffic_lights_geometry_changed") and callable(self.on_traffic_lights_geometry_changed):
            self.on_traffic_lights_geometry_changed()

    # surrounding vehicles
    def SetOnSurroundingVehiclesChanged(self, cb):
        self.on_surrounding_vehicles_changed = cb

    def TriggerSurroundingVehiclesChanged(self):
        if callable(self.on_surrounding_vehicles_changed):
            self.on_surrounding_vehicles_changed()

    # ======== Callback: gnss_shadow_zone
    def SetOnGnssShadowZonesChanged(self, cb): self.on_gnss_shadow_zones_changed = cb
    def TriggerGnssShadowZonesChanged(self):
        if callable(self.on_gnss_shadow_zones_changed): self.on_gnss_shadow_zones_changed()

    def SetOnGnssShadowZonesGeometryChanged(self, callback):
        self.on_gnss_shadow_zones_geometry_changed = callback
        ed = self.editors.get("gnss_shadow_zones")
        if hasattr(ed, "SetOnGeometryChanged"):
            ed.SetOnGeometryChanged(self.TriggerGnssShadowZonesGeometryChanged)

    def TriggerGnssShadowZonesGeometryChanged(self):
        if hasattr(self, "on_gnss_shadow_zones_geometry_changed") and callable(self.on_gnss_shadow_zones_geometry_changed):
            self.on_gnss_shadow_zones_geometry_changed()
    # =========

    # ======= Callback: surrounding vehicles
    def SetOnSurroundingVehiclesGeometryChanged(self, callback):
        self.on_surrounding_vehicles_geometry_changed = callback
        ov = self.editors.get("surrounding_vehicles")
        if hasattr(ov, "SetOnGeometryChanged"):
            ov.SetOnGeometryChanged(self.TriggerSurroundingVehiclesGeometryChanged)

    def TriggerSurroundingVehiclesGeometryChanged(self):
        if hasattr(self, "on_surrounding_vehicles_geometry_changed") and callable(self.on_surrounding_vehicles_geometry_changed):
            self.on_surrounding_vehicles_geometry_changed()
    # =======

    # Guard rails
    def SetOnGuardrailsChanged(self, cb): self.on_guardrails_changed = cb
    def TriggerGuardrailsChanged(self):
        if callable(self.on_guardrails_changed): self.on_guardrails_changed()

    def SetOnGuardrailsGeometryChanged(self, callback):
        self.on_guardrails_geometry_changed = callback
        guard = self.editors.get("guardrails")
        if hasattr(guard, "SetOnGeometryChanged"):
            guard.SetOnGeometryChanged(self.TriggerGuardrailsGeometryChanged)

    def TriggerGuardrailsGeometryChanged(self):
        if hasattr(self, "on_guardrails_geometry_changed") and callable(self.on_guardrails_geometry_changed):
            self.on_guardrails_geometry_changed()

    def SetGuardrailsSubmode(self, submode: str):
        self.editors["guardrails"].SetSubmode(submode)
        self.editors["guardrails"].Redraw()

    def SetGuardrailsCreateDefaults(self, length: float = None, yaw_deg: float = None):
        self.editors["guardrails"].SetCreateDefaults(length=length, yaw_deg=yaw_deg)

    def GetGuardrailsSubmode(self) -> str:
        return self.editor_state.guardrails_submode

    def GetGuardrailsSummary(self):
        out = []
        for i, g in enumerate(self.editor_state.guardrails):
            out.append((i, (g.cx, g.cy), g.length, g.yaw_deg))
        return out

    def SelectGuardrailByIndex(self, idx):
        self.editor_state.selected_guardrail_index = int(idx) if idx is not None else None

    def GetSelectedGuardrailIndex(self):
        return self.editor_state.selected_guardrail_index

    def GetSelectedGuardrailDetails(self):
        i = self.editor_state.selected_guardrail_index
        if i is None or not (0 <= i < len(self.editor_state.guardrails)):
            return None
        g = self.editor_state.guardrails[i]
        return {"index": i, "center_xy": (g.cx, g.cy), "length": g.length, "yaw_deg": g.yaw_deg}

    def UpdateSelectedGuardrailCenter(self, x: float, y: float):
        i = self.editor_state.selected_guardrail_index
        if i is None or not (0 <= i < len(self.editor_state.guardrails)):
            return False, "No guardrail selected."
        g = self.editor_state.guardrails[i]
        g.cx = float(x); g.cy = float(y)
        self.editors["guardrails"].Redraw(); return True, "Center updated."

    def UpdateSelectedGuardrailLength(self, length: float):
        i = self.editor_state.selected_guardrail_index
        if i is None or not (0 <= i < len(self.editor_state.guardrails)):
            return False, "No guardrail selected."
        self.editor_state.guardrails[i].length = float(length)
        self.editors["guardrails"].Redraw(); return True, "Length updated."

    def UpdateSelectedGuardrailYaw(self, yaw_deg: float):
        i = self.editor_state.selected_guardrail_index
        if i is None or not (0 <= i < len(self.editor_state.guardrails)):
            return False, "No guardrail selected."
        self.editor_state.guardrails[i].yaw_deg = float(yaw_deg)
        self.editors["guardrails"].Redraw(); return True, "Yaw updated."

        

    # ---------- mode switch ----------
    def SwitchMode(self, mode: str):
        if self.editor_state.mode == mode: return
        self.current_editor.OnModeExited()
        self.editor_state.mode = mode
        self.current_editor = self.editors[mode]
        self.current_editor.OnModeEntered()

    # ---------- persistence ----------
    def NewBundleName(self):
        next_name = GetNextMapFolderName(self.editor_state.results_root_dir)
        self.editor_state.active_map_folder_name = next_name
        os.makedirs(os.path.join(self.editor_state.results_root_dir, next_name), exist_ok=True)
        print(f"[EditorManager] Active bundle -> {next_name}")
        return next_name

    def SaveBundle(self, folder_path: str = None, which: set = None):
        try:
            if folder_path is None and which is None:
                # 기존 동작(기본 경로에 전부 저장)
                out_dir = SaveMapBundle(self.editor_state)
                return True, f"Bundle saved to: {out_dir}"
            else:
                # 선택 저장
                out_dir = folder_path or os.path.join(
                    self.editor_state.results_root_dir,
                    self.editor_state.active_map_folder_name
                )
                os.makedirs(out_dir, exist_ok=True)
                from utils_io import SaveMapBundleSelective
                SaveMapBundleSelective(self.editor_state, out_dir, which)
                return True, f"Bundle saved to: {out_dir}"
        except Exception as e:
            return False, f"Save failed: {e}"

    def OpenBundle(self, folder_path: str):
        try:
            # draw handle 정리
            for key in self.editor_state.draw_handles.keys():
                for h in self.editor_state.draw_handles[key]:
                    try: h.remove()
                    except Exception: pass
                self.editor_state.draw_handles[key].clear()

            LoadMapBundle(self.editor_state, folder_path)

            # active_map_folder_name 동기화
            try:
                base = os.path.basename(os.path.abspath(folder_path))
                if base:
                    self.editor_state.active_map_folder_name = base
            except Exception:
                pass

            self._CheckChanged()
            return True, f"Bundle loaded from: {folder_path}"
        except Exception as e:
            return False, f"Open failed: {e}"

    # ---------- view ----------
    def FitView(self):
        self.axes.relim(); self.axes.autoscale(); self.figure.canvas.draw_idle()

    def ToggleViewLock(self, lock: bool):
        self.lock_view = bool(lock)

    def SetCommonViewMode(self, on: bool):
        self.common_view_mode = bool(on)

    # ---------- lanes API ----------
    def SetLaneSubmode(self, submode: str):
        self.editors["lanes"].SetSubmode(submode)
        self.editors["lanes"].Redraw()

    def SetLaneType(self, lane_type: str):
        # UI에서 enum "이름"을 넘기면 여기서 코드로 변환해서 상태에 저장
        if lane_type in LANE_TYPE_NAME_TO_CODE:
            self.editor_state.current_lane_type = LANE_TYPE_NAME_TO_CODE[lane_type]
            # 선택된 lane에도 즉시 반영하고 싶으면 아래 두 줄 사용
            # if self.editor_state.selected_lane_index is not None:
            #     self.editor_state.lanes[self.editor_state.selected_lane_index].lane_type = self.editor_state.current_lane_type
            self.editors["lanes"].Redraw()

    def SetLaneModifyMode(self, which: str):
        if which in ("node", "lane"):
            self.editor_state.lanes_modify_mode = which

    def SetLaneDeleteMode(self, which: str):
        if which in ("node", "lane"):
            self.editor_state.lanes_delete_mode = which

    def GetLaneSubmode(self) -> str:
        return self.editor_state.lanes_submode

    def GetLanesSummary(self):
        out = []
        for i, lane in enumerate(self.editor_state.lanes):
            out.append((i, lane.lane_id, lane.lane_type, len(lane.points_xyz)))
        return out

    def SelectLaneByIndex(self, idx):
        self.editor_state.selected_lane_index = int(idx) if idx is not None else None

    def GetSelectedLaneIndex(self):
        return self.editor_state.selected_lane_index

    # def GetSelectedLaneDetails(self):
    #     i = self.editor_state.selected_lane_index
    #     if i is None or not (0 <= i < len(self.editor_state.lanes)):
    #         return None
    #     lane = self.editor_state.lanes[i]
    #     sx, sy, _ = lane.points_xyz[0]
    #     ex, ey, _ = lane.points_xyz[-1]
    #     return {
    #         "lane_index": i,
    #         "lane_id": lane.lane_id,
    #         "lane_type": lane.lane_type,
    #         "start_xy": (sx, sy),
    #         "end_xy": (ex, ey),
    #     }
    def GetSelectedLaneDetails(self):
        i = self.editor_state.selected_lane_index
        if i is None or not (0 <= i < len(self.editor_state.lanes)):
            return None
        lane = self.editor_state.lanes[i]
        sx, sy, _ = lane.points_xyz[0]; ex, ey, _ = lane.points_xyz[-1]
        lane_type_code = lane.lane_type
        lane_type_name = LANE_TYPE_CODE_TO_NAME.get(lane_type_code, str(lane_type_code))
        return {
            "lane_index": i,
            "lane_id": lane.lane_id,
            "lane_type_code": lane_type_code,
            "lane_type_name": lane_type_name,   # ← UI는 이름을 사용
            "start_xy": (sx, sy),
            "end_xy": (ex, ey),
        }

    def UpdateSelectedLanePoints(self, start_xy, end_xy):
        i = self.editor_state.selected_lane_index
        if i is None or not (0 <= i < len(self.editor_state.lanes)):
            return False, "No lane selected."
        lane = self.editor_state.lanes[i]
        z0 = lane.points_xyz[0][2]
        z1 = lane.points_xyz[-1][2]
        lane.points_xyz[0] = (float(start_xy[0]), float(start_xy[1]), z0)
        lane.points_xyz[-1] = (float(end_xy[0]), float(end_xy[1]), z1)
        self.editors["lanes"].Redraw()
        return True, "Points updated."

    # def UpdateSelectedLaneType(self, lane_type):
    #     i = self.editor_state.selected_lane_index
    #     if i is None or not (0 <= i < len(self.editor_state.lanes)):
    #         return False, "No lane selected."
    #     self.editor_state.lanes[i].lane_type = lane_type
    #     self.editors["lanes"].Redraw()
    #     return True, "Lane type updated."

    def UpdateSelectedLaneType(self, lane_type):
        # lane_type이 "이름"으로 들어온다고 가정 → 코드로 변환
        i = self.editor_state.selected_lane_index
        if i is None or not (0 <= i < len(self.editor_state.lanes)):
            return False, "No lane selected."
        if isinstance(lane_type, str):
            if lane_type not in LANE_TYPE_NAME_TO_CODE:
                return False, "Invalid lane type."
            code = LANE_TYPE_NAME_TO_CODE[lane_type]
        else:
            code = int(lane_type)
        self.editor_state.lanes[i].lane_type = code  # ← int 코드 저장
        self.editors["lanes"].Redraw()
        return True, "Lane type updated."

    def CommitCurrentLaneCreation(self):
        ok, msg = self.editors["lanes"].CommitCurrentCreatingLane()
        if ok and callable(self.on_lanes_changed):
            self.on_lanes_changed()
        return ok, msg
    
    def GetSelectedLaneAllPoints(self):
        i = self.editor_state.selected_lane_index
        if i is None or not (0 <= i < len(self.editor_state.lanes)):
            return []
        return list(self.editor_state.lanes[i].points_xyz)

    def UpdateSelectedLaneNode(self, node_index: int, x: float, y: float):
        i = self.editor_state.selected_lane_index
        if i is None or not (0 <= i < len(self.editor_state.lanes)):
            return False, "No lane selected."
        lane = self.editor_state.lanes[i]
        if not (0 <= node_index < len(lane.points_xyz)):
            return False, "Invalid node index."
        z = lane.points_xyz[node_index][2]
        lane.points_xyz[node_index] = (float(x), float(y), z)
        self.editors["lanes"].Redraw()
        # 우측 패널 값도 즉시 동기화
        if hasattr(self, "on_lanes_geometry_changed") and callable(self.on_lanes_geometry_changed):
            self.on_lanes_geometry_changed()
        return True, f"Node {node_index} updated."

    # ---------- buildings API ----------
    def SetBuildingSubmode(self, submode: str):
        self.editors["buildings"].SetSubmode(submode)

    def SetBuildingCreateDefaults(self, count: int = None, radius: float = None, height: float = None):
        if count is not None:
            self.editor_state.building_create_vertices_count = int(max(3, count))
        if radius is not None:
            self.editor_state.building_create_vertex_radius = float(radius)
        if height is not None:
            self.editor_state.building_create_height = float(height)

    def GetBuildingSubmode(self) -> str:
        return self.editor_state.buildings_submode

    def GetBuildingsSummary(self):
        out = []
        for i, b in enumerate(self.editor_state.buildings):
            out.append((i, b.building_id, len(b.vertices_xy), b.height))
        return out

    def SelectBuildingByIndex(self, idx):
        self.editor_state.selected_building_index = int(idx) if idx is not None else None

    def GetSelectedBuildingIndex(self):
        return self.editor_state.selected_building_index

    def GetSelectedBuildingDetails(self):
        i = self.editor_state.selected_building_index
        if i is None or not (0 <= i < len(self.editor_state.buildings)):
            return None
        b = self.editor_state.buildings[i]
        return {
            "building_index": i,
            "building_id": b.building_id,
            "center_xy": b.center_xy,
            "vertices_xy": list(b.vertices_xy),
            "height": b.height,
            "rotation_deg": b.rotation_deg,
        }

    def UpdateSelectedBuildingCenter(self, center_xy):
        i = self.editor_state.selected_building_index
        if i is None or not (0 <= i < len(self.editor_state.buildings)):
            return False, "No building selected."
        b = self.editor_state.buildings[i]
        cx, cy = b.center_xy
        nx, ny = float(center_xy[0]), float(center_xy[1])
        dx, dy = nx - cx, ny - cy
        b.center_xy = (nx, ny)
        b.vertices_xy = [(vx + dx, vy + dy) for (vx, vy) in b.vertices_xy]
        self.editors["buildings"].Redraw()
        return True, "Center updated."

    def UpdateSelectedBuildingRotation(self, rotation_deg):
        i = self.editor_state.selected_building_index
        if i is None or not (0 <= i < len(self.editor_state.buildings)):
            return False, "No building selected."
        b = self.editor_state.buildings[i]
        delta = float(rotation_deg) - b.rotation_deg
        from editor_buildings import _RotatePoint
        cx, cy = b.center_xy
        b.vertices_xy = [_RotatePoint(vx, vy, cx, cy, delta) for (vx, vy) in b.vertices_xy]
        b.rotation_deg = float(rotation_deg)
        self.editors["buildings"].Redraw()
        return True, "Rotation updated."

    def UpdateSelectedBuildingHeight(self, height):
        i = self.editor_state.selected_building_index
        if i is None or not (0 <= i < len(self.editor_state.buildings)):
            return False, "No building selected."
        self.editor_state.buildings[i].height = float(height)
        return True, "Height updated."

    def UpdateSelectedBuildingVertex(self, vertex_index: int, xy):
        i = self.editor_state.selected_building_index
        if i is None or not (0 <= i < len(self.editor_state.buildings)):
            return False, "No building selected."
        b = self.editor_state.buildings[i]
        if not (0 <= vertex_index < len(b.vertices_xy)):
            return False, "Invalid vertex index."
        verts = list(b.vertices_xy)
        verts[vertex_index] = (float(xy[0]), float(xy[1]))
        b.vertices_xy = verts
        self.editors["buildings"].Redraw()
        return True, f"Vertex {vertex_index} updated."

    # ---------- signs API ----------
    def SetSignSubmode(self, submode: str):
        self.editors["signs"].SetSubmode(submode)
        self.editors["signs"].Redraw()

    def SetSignCreateDefaults(self, sign_class: int = None, z: float = None, yaw: float = None):
        self.editors["signs"].SetCreateDefaults(sign_class=sign_class, z=z, yaw=yaw)

    def GetSignSubmode(self) -> str:
        return self.editor_state.signs_submode

    def GetSignsSummary(self):
        out = []
        for i, s in enumerate(self.editor_state.signs):
            out.append((i, (s.x, s.y), s.z, s.yaw, s.sign_class))
        return out

    def SelectSignByIndex(self, idx):
        self.editor_state.selected_sign_index = int(idx) if idx is not None else None

    def GetSelectedSignIndex(self):
        return self.editor_state.selected_sign_index

    def GetSelectedSignDetails(self):
        i = self.editor_state.selected_sign_index
        if i is None or not (0 <= i < len(self.editor_state.signs)):
            return None
        s = self.editor_state.signs[i]
        return {"sign_index": i, "x": s.x, "y": s.y, "z": s.z, "yaw": s.yaw, "sign_class": s.sign_class}

    def UpdateSelectedSignCenter(self, x: float, y: float):
        i = self.editor_state.selected_sign_index
        if i is None or not (0 <= i < len(self.editor_state.signs)):
            return False, "No sign selected."
        s = self.editor_state.signs[i]
        s.x = float(x)
        s.y = float(y)
        self.editors["signs"].Redraw()
        return True, "Center updated."

    def UpdateSelectedSignHeight(self, z: float):
        i = self.editor_state.selected_sign_index
        if i is None or not (0 <= i < len(self.editor_state.signs)):
            return False, "No sign selected."
        self.editor_state.signs[i].z = float(z)
        self.editors["signs"].Redraw()
        return True, "Height updated."

    def UpdateSelectedSignYaw(self, yaw: float):
        i = self.editor_state.selected_sign_index
        if i is None or not (0 <= i < len(self.editor_state.signs)):
            return False, "No sign selected."
        self.editor_state.signs[i].yaw = float(yaw)
        self.editors["signs"].Redraw()
        return True, "Yaw updated."

    def UpdateSelectedSignClass(self, sign_class: int):
        i = self.editor_state.selected_sign_index
        if i is None or not (0 <= i < len(self.editor_state.signs)):
            return False, "No sign selected."
        self.editor_state.signs[i].sign_class = int(sign_class)
        self.editors["signs"].Redraw()
        return True, "Class updated."

    # ---------- traffic lights API ----------
    def SetTrafficLightsSubmode(self, submode: str):
        self.editors["traffic_lights"].SetSubmode(submode)
        self.editors["traffic_lights"].Redraw()

    def SetTrafficLightsCreateDefaults(self, state_code: int = None, z: float = None):
        self.editors["traffic_lights"].SetCreateDefaults(state_code=state_code, z=z)

    def GetTrafficLightsSubmode(self) -> str:
        return self.editor_state.traffic_lights_submode

    def GetTrafficLightsSummary(self):
        out = []
        for i, t in enumerate(self.editor_state.traffic_lights):
            out.append((i, (t.x, t.y), t.z, t.state))
        return out

    def SelectTrafficLightByIndex(self, idx):
        self.editor_state.selected_traffic_light_index = int(idx) if idx is not None else None

    def GetSelectedTrafficLightIndex(self):
        return self.editor_state.selected_traffic_light_index

    def GetSelectedTrafficLightDetails(self):
        i = self.editor_state.selected_traffic_light_index
        if i is None or not (0 <= i < len(self.editor_state.traffic_lights)):
            return None
        t = self.editor_state.traffic_lights[i]
        return {"index": i, "x": t.x, "y": t.y, "z": t.z, "state": t.state}

    def UpdateSelectedTrafficLightCenter(self, x: float, y: float):
        i = self.editor_state.selected_traffic_light_index
        if i is None or not (0 <= i < len(self.editor_state.traffic_lights)):
            return False, "No traffic light selected."
        t = self.editor_state.traffic_lights[i]
        t.x = float(x)
        t.y = float(y)
        self.editors["traffic_lights"].Redraw()
        return True, "Center updated."

    def UpdateSelectedTrafficLightHeight(self, z: float):
        i = self.editor_state.selected_traffic_light_index
        if i is None or not (0 <= i < len(self.editor_state.traffic_lights)):
            return False, "No traffic light selected."
        self.editor_state.traffic_lights[i].z = float(z)
        self.editors["traffic_lights"].Redraw()
        return True, "Height updated."

    def UpdateSelectedTrafficLightState(self, state_code: int):
        i = self.editor_state.selected_traffic_light_index
        if i is None or not (0 <= i < len(self.editor_state.traffic_lights)):
            return False, "No traffic light selected."
        self.editor_state.traffic_lights[i].state = int(state_code)
        self.editors["traffic_lights"].Redraw()
        return True, "State updated."


    # ---------- GNSS shadow zones API ----------
    def SetGnssShadowZonesSubmode(self, submode: str):
        self.editors["gnss_shadow_zones"].SetSubmode(submode)
        self.editors["gnss_shadow_zones"].Redraw()

    def SetGnssShadowZonesCreateDefaults(self, pos_std: float = None, radius: float = None):
        self.editors["gnss_shadow_zones"].SetCreateDefaults(pos_std=pos_std, radius=radius)

    def GetGnssShadowZonesSubmode(self) -> str:
        return self.editor_state.gnss_shadow_zones_submode

    def GetGnssShadowZonesSummary(self):
        out = []
        for i, z in enumerate(self.editor_state.gnss_shadow_zones):
            out.append((i, (z.x, z.y), z.pos_std, z.radius))
        return out

    def SelectGnssShadowZoneByIndex(self, idx):
        self.editor_state.selected_gnss_shadow_zone_index = int(idx) if idx is not None else None

    def GetSelectedGnssShadowZoneIndex(self):
        return self.editor_state.selected_gnss_shadow_zone_index

    def GetSelectedGnssShadowZoneDetails(self):
        i = self.editor_state.selected_gnss_shadow_zone_index
        if i is None or not (0 <= i < len(self.editor_state.gnss_shadow_zones)):
            return None
        z = self.editor_state.gnss_shadow_zones[i]
        return {"index": i, "x": z.x, "y": z.y, "pos_std": z.pos_std, "radius": z.radius}

    def UpdateSelectedGnssShadowZoneCenter(self, x: float, y: float):
        i = self.editor_state.selected_gnss_shadow_zone_index
        if i is None or not (0 <= i < len(self.editor_state.gnss_shadow_zones)):
            return False, "No GNSS shadow zone selected."
        z = self.editor_state.gnss_shadow_zones[i]
        z.x = float(x); z.y = float(y)
        self.editors["gnss_shadow_zones"].Redraw(); return True, "Center updated."

    def UpdateSelectedGnssShadowZonePosStd(self, pos_std: float):
        i = self.editor_state.selected_gnss_shadow_zone_index
        if i is None or not (0 <= i < len(self.editor_state.gnss_shadow_zones)):
            return False, "No GNSS shadow zone selected."
        self.editor_state.gnss_shadow_zones[i].pos_std = float(pos_std)
        self.editors["gnss_shadow_zones"].Redraw(); return True, "pos_std updated."

    def UpdateSelectedGnssShadowZoneRadius(self, radius: float):
        i = self.editor_state.selected_gnss_shadow_zone_index
        if i is None or not (0 <= i < len(self.editor_state.gnss_shadow_zones)):
            return False, "No GNSS shadow zone selected."
        self.editor_state.gnss_shadow_zones[i].radius = float(radius)
        self.editors["gnss_shadow_zones"].Redraw(); return True, "radius updated."


    # ---------- surrounding vehicles API ----------
    def SetSurroundingVehiclesSubmode(self, submode: str):
        self.editors["surrounding_vehicles"].SetSubmode(submode)
        self.editors["surrounding_vehicles"].Redraw()

    def SetSurroundingVehiclesModifyMode(self, which: str):
        if which in ("node", "vehicle"):
            self.editor_state.surrounding_vehicles_modify_mode = which

    def SetSurroundingVehiclesDeleteMode(self, which: str):
        if which in ("node", "vehicle"):
            self.editor_state.surrounding_vehicles_delete_mode = which

    def SetSurroundingVehiclesCreateDefaults(self, speed_mps: float = None):
        self.editors["surrounding_vehicles"].SetCreateDefaults(speed_mps=speed_mps)

    def GetSurroundingVehiclesSubmode(self) -> str:
        return self.editor_state.surrounding_vehicles_submode

    def GetSurroundingVehiclesSummary(self):
        out = []
        for i, v in enumerate(self.editor_state.surrounding_vehicles):
            out.append((i, v.vehicle_id, len(v.points_xy), v.speed_mps))
        return out

    def SelectSurroundingVehicleByIndex(self, idx):
        self.editor_state.selected_surrounding_vehicle_index = int(idx) if idx is not None else None

    def GetSelectedSurroundingVehicleIndex(self):
        return self.editor_state.selected_surrounding_vehicle_index

    def GetSelectedSurroundingVehicleDetails(self):
        i = self.editor_state.selected_surrounding_vehicle_index
        if i is None or not (0 <= i < len(self.editor_state.surrounding_vehicles)):
            return None
        v = self.editor_state.surrounding_vehicles[i]
        sx, sy = v.points_xy[0]
        ex, ey = v.points_xy[-1]
        return {
            "vehicle_index": i,
            "vehicle_id": v.vehicle_id,
            "speed_mps": v.speed_mps,
            "start_xy": (sx, sy),
            "end_xy": (ex, ey),
            "points_xy": list(v.points_xy),
        }

    def UpdateSelectedSurroundingVehicleNode(self, node_index: int, x: float, y: float):
        i = self.editor_state.selected_surrounding_vehicle_index
        if i is None or not (0 <= i < len(self.editor_state.surrounding_vehicles)):
            return False, "No surrounding vehicle selected."
        v = self.editor_state.surrounding_vehicles[i]
        if not (0 <= node_index < len(v.points_xy)):
            return False, "Invalid node index."
        pts = list(v.points_xy)
        pts[node_index] = (float(x), float(y))
        v.points_xy = pts
        self.editors["surrounding_vehicles"].Redraw()
        if hasattr(self, "on_surrounding_vehicles_geometry_changed") and callable(self.on_surrounding_vehicles_geometry_changed):
            self.on_surrounding_vehicles_geometry_changed()
        return True, f"Node {node_index} updated."

    def UpdateSelectedSurroundingVehicleSpeed(self, speed_mps: float):
        i = self.editor_state.selected_surrounding_vehicle_index
        if i is None or not (0 <= i < len(self.editor_state.surrounding_vehicles)):
            return False, "No surrounding vehicle selected."
        self.editor_state.surrounding_vehicles[i].speed_mps = float(speed_mps)
        self.editors["surrounding_vehicles"].Redraw()
        return True, "Speed updated."

    def CommitCurrentSurroundingVehicleCreation(self):
        ok, msg = self.editors["surrounding_vehicles"].CommitCurrentCreatingSurroundingVehicle()
        if ok and callable(self.on_surrounding_vehicles_changed):
            self.on_surrounding_vehicles_changed()
        return ok, msg

    # ---------- events ----------
    def _ConnectEvents(self):
        self.figure.canvas.mpl_connect("button_press_event", self.OnMousePress)
        self.figure.canvas.mpl_connect("motion_notify_event", self.OnMouseMove)
        self.figure.canvas.mpl_connect("button_release_event", self.OnMouseRelease)
        self.figure.canvas.mpl_connect("key_press_event", self.OnKeyPress)
        self.figure.canvas.mpl_connect("scroll_event", self.OnScroll)

    def _IsInViewMode(self) -> bool:
        if self.common_view_mode:
            return True
        if self.editor_state.mode == "lanes" and self.GetLaneSubmode() == "view":
            return True
        if self.editor_state.mode == "buildings" and self.GetBuildingSubmode() == "view":
            return True
        if self.editor_state.mode == "signs" and self.GetSignSubmode() == "view":
            return True
        if self.editor_state.mode == "traffic_lights" and self.GetTrafficLightsSubmode() == "view":
            return True
        if self.editor_state.mode == "guardrails" and self.GetGuardrailsSubmode() == "view": 
            return True
        if self.editor_state.mode == "gnss_shadow_zones" and self.GetGnssShadowZonesSubmode() == "view":  # ← 추가
            return True
        if self.editor_state.mode == "surrounding_vehicles" and self.GetSurroundingVehiclesSubmode() == "view":  # ★ 추가
            return True
        return False

    def OnMousePress(self, event):
        if self._IsInViewMode():
            if event.button == 1 and event.x is not None:
                self.is_panning = True
                self.last_pan_px = (event.x, event.y)
            return
        self.current_editor.OnMousePress(event)
        self._CheckChanged()

    def OnMouseMove(self, event):
        if self._IsInViewMode():
            if self.is_panning and event.x is not None:
                self._PanWithMouse(event.x, event.y)
            return
        self.current_editor.OnMouseMove(event)

    def OnMouseRelease(self, event):
        if self._IsInViewMode():
            self.is_panning = False
            return
        self.current_editor.OnMouseRelease(event)
        self._CheckChanged()

    def OnScroll(self, event):
        if not self._IsInViewMode():
            return
        self._ZoomAt(event.x, event.y, zoom_in=(event.button == "up"))

    def OnKeyPress(self, event):
        key = event.key
        if key in ["q", "Q", "escape"]:
            import sys
            sys.exit(0)
        else:
            self.current_editor.OnKeyPress(event)
            self._CheckChanged()

    def _PanWithMouse(self, cur_x, cur_y):
        inv = self.axes.transData.inverted()
        dx_px = cur_x - self.last_pan_px[0]
        dy_px = cur_y - self.last_pan_px[1]
        self.last_pan_px = (cur_x, cur_y)
        x0, y0 = inv.transform((0, 0))
        x1, y1 = inv.transform((dx_px, dy_px))
        ddx, ddy = x0 - x1, y0 - y1
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()
        self.axes.set_xlim(xlim[0] + ddx, xlim[1] + ddx)
        self.axes.set_ylim(ylim[0] + ddy, ylim[1] + ddy)
        self.figure.canvas.draw_idle()

    def _ZoomAt(self, px, py, zoom_in: bool):
        inv = self.axes.transData.inverted()
        try:
            xdata, ydata = inv.transform((px, py))
        except Exception:
            xdata = sum(self.axes.get_xlim()) * 0.5
            ydata = sum(self.axes.get_ylim()) * 0.5
        scale = 1.2 if zoom_in else (1 / 1.2)
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()
        new_xlim = (xdata + (xlim[0] - xdata) / scale, xdata + (xlim[1] - xdata) / scale)
        new_ylim = (ydata + (ylim[0] - ydata) / scale, ydata + (ylim[1] - ydata) / scale)
        self.axes.set_xlim(new_xlim)
        self.axes.set_ylim(new_ylim)
        self.figure.canvas.draw_idle()

    def _CheckChanged(self):
        if len(self.editor_state.lanes) != self._last_lanes_count:
            self._last_lanes_count = len(self.editor_state.lanes)
            if callable(self.on_lanes_changed): self.on_lanes_changed()
        if len(self.editor_state.buildings) != self._last_buildings_count:
            self._last_buildings_count = len(self.editor_state.buildings)
            if callable(self.on_buildings_changed): self.on_buildings_changed()
        if len(self.editor_state.signs) != self._last_signs_count:
            self._last_signs_count = len(self.editor_state.signs)
            if callable(self.on_signs_changed): self.on_signs_changed()
        if len(self.editor_state.traffic_lights) != self._last_traffic_count:
            self._last_traffic_count = len(self.editor_state.traffic_lights)
            if callable(self.on_traffic_lights_changed): self.on_traffic_lights_changed()

        # ★ 추가: lane의 노드 총합 변동 감지
        current_nodes_total = sum(len(l.points_xyz) for l in self.editor_state.lanes)
        if current_nodes_total != self._last_lanes_nodes_total:
            self._last_lanes_nodes_total = current_nodes_total
            if callable(self.on_lanes_changed): self.on_lanes_changed()

        # Guardrails
        if len(self.editor_state.guardrails) != self._last_guardrails_count:
            self._last_guardrails_count = len(self.editor_state.guardrails)
            if callable(self.on_guardrails_changed): self.on_guardrails_changed()
        
        # ← 추가: GNSS 섀도우 존 개수 변동 감지
        if len(self.editor_state.gnss_shadow_zones) != self._last_gnss_shadow_zones_count:
            self._last_gnss_shadow_zones_count = len(self.editor_state.gnss_shadow_zones)
            if callable(self.on_gnss_shadow_zones_changed): self.on_gnss_shadow_zones_changed()
        
        # surrounding vehicles
        if len(self.editor_state.surrounding_vehicles) != self._last_surrounding_count:
            self._last_surrounding_count = len(self.editor_state.surrounding_vehicles)
            if callable(self.on_surrounding_vehicles_changed):
                self.on_surrounding_vehicles_changed()

        # surrounding vehicles 노드 총합 변동
        current_nodes_total = sum(len(v.points_xy) for v in self.editor_state.surrounding_vehicles)
        if current_nodes_total != self._last_surrounding_nodes_total:
            self._last_surrounding_nodes_total = current_nodes_total
            if callable(self.on_surrounding_vehicles_changed):
                self.on_surrounding_vehicles_changed()

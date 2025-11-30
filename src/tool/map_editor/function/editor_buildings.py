# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_buildings.py
import math
import matplotlib.patches as mpatches
from editor_base import EditorBase
from editor_state import BuildingPolygon
from typing import Optional, List

def _RotatePoint(px, py, cx, cy, deg):
    rad = math.radians(deg)
    dx, dy = px - cx, py - cy
    rx = cx + dx * math.cos(rad) - dy * math.sin(rad)
    ry = cy + dx * math.sin(rad) + dy * math.cos(rad)
    return rx, ry


class BuildingsEditor(EditorBase):
    def __init__(self, editor_state, figure, axes):
        super().__init__(editor_state, figure, axes)
        self.moving_vertex = False
        self.moving_center = False
        self.rotating = False
        self.active_vertex_index = None
        self.rotation_sensitivity_deg_per_pixel = 0.5
        self.last_mouse_x = None

        # geometry changed 콜백
        self.on_geometry_changed = None

        # 픽킹 반경(픽셀 기준)
        self.vertex_pick_radius_px = 24.0
        self.edge_pick_radius_px = 18.0
        self.center_pick_radius_px = 16.0

        # Undo 스택
        self.undo_stack: List[list] = []
        self.max_undo: int = 50

    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback

    # ---- helpers ----
    def _PixelsToDataDistanceAtMouse(self, event, pixel_radius: float = 28.0) -> float:
        try:
            px, py = float(event.x), float(event.y)
            inv = self.axes.transData.inverted()
            x0, y0 = inv.transform((px, py))
            x1, y1 = inv.transform((px + pixel_radius, py))
            x2, y2 = inv.transform((px, py + pixel_radius))
            return float(max(abs(x1 - x0), abs(y2 - y0)))
        except Exception:
            return 1.0

    def _DistancePoint(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def _DistancePointToSegment(self, px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        vx, vy = x2 - x1, y2 - y1
        wx, wy = px - x1, py - y1
        seg_len2 = vx * vx + vy * vy
        if seg_len2 <= 1e-12:
            return self._DistancePoint(px, py, x1, y1)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len2))
        proj_x = x1 + t * vx
        proj_y = y1 + t * vy
        return self._DistancePoint(px, py, proj_x, proj_y)

    def _PolygonMinDistance(self, verts, px, py) -> float:
        # 버텍스/에지에 대한 최소거리
        best = float("inf")
        n = len(verts)
        if n == 0:
            return best
        # vertices
        for (vx, vy) in verts:
            d = self._DistancePoint(px, py, vx, vy)
            if d < best:
                best = d
        # edges
        if n >= 2:
            for i in range(n):
                x1, y1 = verts[i]
                x2, y2 = verts[(i + 1) % n]
                d = self._DistancePointToSegment(px, py, x1, y1, x2, y2)
                if d < best:
                    best = d
        return best

    def _PickNearestBuildingIndex(self, x: float, y: float, accept_threshold: float):
        best_i = None
        best_d = float("inf")
        for i, b in enumerate(self.editor_state.buildings):
            # 중심 거리
            cx, cy = b.center_xy
            d_center = self._DistancePoint(x, y, cx, cy)
            # 다각형 버텍스/에지 거리
            d_poly = self._PolygonMinDistance(b.vertices_xy, x, y)
            d = min(d_center, d_poly)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i if (best_i is not None and best_d <= accept_threshold) else None

    def _FindClosestVertex(self, building, x, y, threshold):
        best_idx = None
        best_d = float("inf")
        for i, (vx, vy) in enumerate(building.vertices_xy):
            d = math.hypot(vx - x, vy - y)
            if d <= threshold and d < best_d:
                best_d = d
                best_idx = i
        return best_idx

    def _IsNearCenter(self, building, x, y, threshold):
        cx, cy = building.center_xy
        return math.hypot(cx - x, cy - y) <= threshold
    
    def _IsCtrlLikePressed(self, event) -> bool:
        try:
            if hasattr(event, "guiEvent") and event.guiEvent is not None:
                from PyQt5 import QtCore
                mods = int(event.guiEvent.modifiers())
                return bool(
                    (mods & int(QtCore.Qt.ControlModifier)) or
                    (mods & int(QtCore.Qt.MetaModifier)) or
                    (mods & int(QtCore.Qt.KeyboardModifierMask))
                )
        except Exception:
            pass
        return False

    # ---- undo helpers ----
    def _Snapshot(self) -> list:
        return [BuildingPolygon(building_id=building.building_id, 
                                center_xy=building.center_xy, 
                                vertices_xy=building.vertices_xy,
                                height=building.height, 
                                rotation_deg=building.rotation_deg)
                for building in self.editor_state.buildings] 

    def _PushUndoSnapshot(self):
        try:
            snap = self._Snapshot()
            self.undo_stack.append(snap)
            if len(self.undo_stack) > self.max_undo:
                self.undo_stack.pop(0)
        except Exception:
            pass

    def _RestoreFromSnapshot(self, snapshot: list):
        self.editor_state.buildings.clear()
        for building in snapshot:
            self.editor_state.buildings.append(
                BuildingPolygon(building_id=building.building_id, 
                                center_xy=building.center_xy, 
                                vertices_xy=building.vertices_xy, 
                                height=building.height, 
                                rotation_deg=building.rotation_deg)
            )
            
    # ---- mode transitions ----
    def OnModeEntered(self):
        print("[BuildingsEditor] Entered. Submodes: view/create/modify/delete")
        self.Redraw()

    # ---- mouse/keys ----
    def OnKeyPress(self, event):
        key_text = (event.key or "").lower()

        # Ctrl+Z / Cmd+Z → Undo
        is_ctrl_like_sequence = key_text in ("ctrl+z", "control+z", "cmd+z", "meta+z", "super+z")
        is_ctrl_like_modifier = (key_text == "z" and self._IsCtrlLikePressed(event))
        
        if is_ctrl_like_sequence or is_ctrl_like_modifier:
            if self.undo_stack:
                snap = self.undo_stack.pop()
                self._RestoreFromSnapshot(snap)
                self.Redraw()
                if callable(self.on_geometry_changed):
                    self.on_geometry_changed()
                print("[BuildingsEditor] Undo.")
            return

    def OnMousePress(self, event):
        submode = self.editor_state.buildings_submode
        if submode == "view":
            return

        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse

        # 픽킹 거리(데이터 좌표) 계산
        vthr = self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px)
        ethr = self._PixelsToDataDistanceAtMouse(event, self.edge_pick_radius_px)
        cthr = self._PixelsToDataDistanceAtMouse(event, self.center_pick_radius_px)

        # ------- CREATE -------
        if submode == "create" and event.button == 1:
            self._PushUndoSnapshot()
            n = max(3, int(self.editor_state.building_create_vertices_count))
            r = float(self.editor_state.building_create_vertex_radius)
            h = float(self.editor_state.building_create_height)
            cx, cy = x, y
            verts = []
            for i in range(n):
                ang = 360.0 * i / n
                vx = cx + r * math.cos(math.radians(ang))
                vy = cy + r * math.sin(math.radians(ang))
                verts.append((vx, vy))
            poly = BuildingPolygon(
                building_id=self.editor_state.next_building_id,
                center_xy=(cx, cy),
                vertices_xy=verts,
                height=h,
                rotation_deg=0.0,
            )
            self.editor_state.buildings.append(poly)
            self.editor_state.next_building_id += 1
            print(f"[BuildingsEditor] Building committed. Total={len(self.editor_state.buildings)}")
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        # ------- MODIFY / DELETE -------
        if event.button == 1:
            # 1) 항상 클릭 위치에서 가장 가까운 빌딩 자동 선택
            pick_thr = max(vthr * 3.0, ethr * 3.5, cthr * 4.0)
            picked = self._PickNearestBuildingIndex(x, y, accept_threshold=pick_thr)
            if picked is None:
                print("[BuildingsEditor] No building near cursor.")
                return

            # 선택 갱신
            if self.editor_state.selected_building_index != picked:
                self.editor_state.selected_building_index = picked
                self.Redraw()

            building = self.editor_state.buildings[picked]

            if submode == "modify":
                # 회전 시작(Shift 감지 가능한 환경일 때만)
                if getattr(event, "key", None) in ("shift", "shift+control", "control+shift"):
                    self.rotating = True
                    self.last_mouse_x = event.x
                    return

                # 2) 꼭짓점 우선 픽
                vi = self._FindClosestVertex(building, x, y, vthr)
                if vi is None:
                    vi = self._FindClosestVertex(building, x, y, vthr * 1.8)
                if vi is not None:
                    self._PushUndoSnapshot()
                    self.moving_vertex = True
                    self.active_vertex_index = vi
                    return

                # 3) 센터 픽 (조금 더 타이트)
                if self._IsNearCenter(building, x, y, cthr * 0.9) or self._IsNearCenter(building, x, y, cthr * 1.5):
                    self.moving_center = True
                    self.last_mouse_x, _ = event.x, event.y
                    self._PushUndoSnapshot()
                    return

                # (선택) 에지 근처를 집으면 전체 이동으로 보고 싶다면 여기에 로직 추가 가능
                # 현재 요구사항은 '센터' 또는 '버텍스' 이동이므로 생략

            elif submode == "delete":
                sel = self.editor_state.selected_building_index
                if sel is not None and 0 <= sel < len(self.editor_state.buildings):
                    self._PushUndoSnapshot()
                    self.editor_state.buildings.pop(picked)
                    if self.editor_state.selected_building_index == picked:
                        self.editor_state.selected_building_index = None
                    print("[BuildingsEditor] Building deleted.")
                    self.Redraw()
                    if callable(self.on_geometry_changed):
                        self.on_geometry_changed()
                else:
                    print("[BuildingsEditor] Select a building in the right panel first.")
                return

    def OnMouseMove(self, event):
        if self.editor_state.buildings_submode != "modify":
            return
        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse
        sel = self.editor_state.selected_building_index
        if sel is None or not (0 <= sel < len(self.editor_state.buildings)):
            return
        building = self.editor_state.buildings[sel]

        if self.rotating and self.last_mouse_x is not None and event.x is not None:
            dx_px = event.x - self.last_mouse_x
            self.last_mouse_x = event.x
            delta_deg = dx_px * self.rotation_sensitivity_deg_per_pixel
            building.rotation_deg += delta_deg
            cx, cy = building.center_xy
            new_verts = [_RotatePoint(vx, vy, cx, cy, delta_deg) for (vx, vy) in building.vertices_xy]
            building.vertices_xy = new_verts
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        if self.moving_center:
            cx, cy = building.center_xy
            dx, dy = x - cx, y - cy
            building.center_xy = (x, y)
            building.vertices_xy = [(vx + dx, vy + dy) for (vx, vy) in building.vertices_xy]
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        if self.moving_vertex and self.active_vertex_index is not None:
            vx, vy = x, y
            verts = list(building.vertices_xy)
            verts[self.active_vertex_index] = (vx, vy)
            building.vertices_xy = verts
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

    def OnMouseRelease(self, event):
        self.moving_vertex = False
        self.moving_center = False
        self.rotating = False
        self.active_vertex_index = None

    # ---- UI hooks ----
    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "modify", "delete"}
        if submode in allowed:
            self.editor_state.buildings_submode = submode
            print(f"[BuildingsEditor] Submode -> {submode}")
            self.Redraw()

    def Redraw(self):
        xlim = self.axes.get_xlim(); ylim = self.axes.get_ylim()
        for h in self.editor_state.draw_handles["buildings"]:
            try:
                h.remove()
            except Exception:
                pass
        self.editor_state.draw_handles["buildings"].clear()

        selected_index = self.editor_state.selected_building_index
        is_modify_mode = (self.editor_state.buildings_submode == "modify")

        # 시각 스타일
        fill_alpha = 0.25
        vertex_size = 60
        center_size = 80
        base_color = "orange"               # 선/꼭짓점/채움 색상
        selected_edge_color = "red"         # 선택된 빌딩 외곽선 강조 색
        selected_line_width = 3.0
        normal_line_width = 2.0

        for idx, b in enumerate(self.editor_state.buildings):
            # 선분(외곽선)
            if len(b.vertices_xy) >= 2:
                xs = [p[0] for p in b.vertices_xy] + [b.vertices_xy[0][0]]
                ys = [p[1] for p in b.vertices_xy] + [b.vertices_xy[0][1]]
                line_width = selected_line_width if (is_modify_mode and idx == selected_index) else normal_line_width
                edge_color = selected_edge_color if (is_modify_mode and idx == selected_index) else base_color
                line, = self.axes.plot(xs, ys, linewidth=line_width, color=edge_color, alpha=0.95)
                self.editor_state.draw_handles["buildings"].append(line)

            # 면 채우기
            if len(b.vertices_xy) >= 3:
                poly = mpatches.Polygon(
                    b.vertices_xy,
                    closed=True,
                    facecolor=base_color,
                    edgecolor="none",
                    alpha=fill_alpha,
                )
                self.axes.add_patch(poly)
                self.editor_state.draw_handles["buildings"].append(poly)

            # 꼭짓점
            if b.vertices_xy:
                pts = self.axes.scatter(
                    [p[0] for p in b.vertices_xy],
                    [p[1] for p in b.vertices_xy],
                    s=vertex_size, marker="o", c=base_color, edgecolors="black", linewidths=0.5, alpha=0.95
                )
                self.editor_state.draw_handles["buildings"].append(pts)

            # 중심
            ctr = self.axes.scatter(
                [b.center_xy[0]], [b.center_xy[1]],
                s=center_size, marker="s", c=base_color, edgecolors="black", linewidths=0.6
            )
            self.editor_state.draw_handles["buildings"].append(ctr)

        self.axes.set_xlim(xlim); self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()

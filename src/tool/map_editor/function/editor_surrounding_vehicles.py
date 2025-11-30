# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_surrounding_vehicles.py
import math
from typing import Optional, Tuple
import matplotlib.patches as mpatches
import matplotlib.transforms as mtransforms
from editor_base import EditorBase
from editor_state import EditorState, SurroundingVehicleTrajectory

try:
    from PyQt5 import QtWidgets, QtCore
except Exception:
    QtWidgets = None
    QtCore = None


class SurroundingVehiclesEditor(EditorBase):
    def __init__(self, editor_state: EditorState, figure, axes):
        super().__init__(editor_state, figure, axes)

        # 상태 플래그
        self.moving_vertex = False
        self.moving_vehicle = False
        self.move_anchor_xy: Optional[Tuple[float, float]] = None
        self.original_points_xy = None

        # 픽킹 반경(픽셀)
        self.vertex_pick_radius_px = 24.0
        self.polyline_pick_radius_px = 18.0

        # 지오메트리 변경 콜백
        self.on_geometry_changed = None

        # 차량 박스 기본 치수(미터)
        self.vehicle_length_m = 4.0
        self.vehicle_width_m = 2.0

    def ComputeYawDegreesPerNode(self, points_xy):
        """각 노드가 바라볼 yaw를 도(deg)로 반환. 마지막 노드는 이전 점을 바라봄."""
        n = len(points_xy)
        if n == 0:
            return []
        if n == 1:
            return [0.0]
        yaw_list = []
        for i in range(n):
            if i < n - 1:
                dx = points_xy[i + 1][0] - points_xy[i][0]
                dy = points_xy[i + 1][1] - points_xy[i][1]
            else:
                dx = points_xy[i][0] - points_xy[i - 1][0]
                dy = points_xy[i][1] - points_xy[i - 1][1]
            yaw_deg = math.degrees(math.atan2(dy, dx))
            yaw_list.append(float(yaw_deg))
        return yaw_list

    def AddVehicleBoxPatch(self, x, y, yaw_deg, edge_color="black", line_width=1.8, alpha=0.95):
        """중심 (x,y)에 길이×너비 박스를 yaw_deg만큼 회전해서 그린다(윤곽선만)."""
        half_l = 0.5 * float(self.vehicle_length_m)
        half_w = 0.5 * float(self.vehicle_width_m)
        # 원점 기준 중앙정렬된 사각형을 만든 뒤 회전+이동 변환을 적용
        rect = mpatches.Rectangle(
            (-half_l, -half_w),
            2.0 * half_l,
            2.0 * half_w,
            facecolor="none",
            edgecolor=edge_color,
            linewidth=line_width,
            alpha=alpha,
        )
        trans = mtransforms.Affine2D().rotate_deg(yaw_deg).translate(float(x), float(y)) + self.axes.transData
        rect.set_transform(trans)
        self.axes.add_patch(rect)
        self.editor_state.draw_handles["surrounding_vehicles"].append(rect)


    def _AddSegmentArrow(self, x1, y1, x2, y2, edge_color="black", line_width=2.0, alpha=0.95):
        arr = self.axes.annotate(
            "",
            xy=(float(x2), float(y2)),
            xytext=(float(x1), float(y1)),
            arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0,
                            linewidth=float(line_width), color=edge_color, alpha=alpha)
        )
        self.editor_state.draw_handles["surrounding_vehicles"].append(arr)

    # ---------- 콜백 ----------
    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback



    # ---------- UI 훅 ----------
    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "modify", "delete"}
        if submode in allowed:
            self.editor_state.surrounding_vehicles_submode = submode
            print(f"[SurroundingVehiclesEditor] Submode -> {submode}")

    def SetCreateDefaults(self, speed_mps: float = None):
        if speed_mps is not None:
            self.editor_state.surrounding_vehicles_create_speed_mps = float(speed_mps)

    # ---------- 헬퍼 ----------
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

    def _PickNearestVehicleIndex(self, x: float, y: float, accept_threshold: float):
        best_i = None
        best_d = float("inf")
        for i, vehicle in enumerate(self.editor_state.surrounding_vehicles):
            pts = vehicle.points_xy
            if not pts:
                continue
            # 노드 거리
            for (vx, vy) in pts:
                d = self._DistancePoint(x, y, vx, vy)
                if d < best_d:
                    best_d = d; best_i = i
            # 선분 거리
            for j in range(len(pts) - 1):
                x1, y1 = pts[j]; x2, y2 = pts[j + 1]
                d = self._DistancePointToSegment(x, y, x1, y1, x2, y2)
                if d < best_d:
                    best_d = d; best_i = i
        return best_i if (best_i is not None and best_d <= accept_threshold) else None

    def _FindClosestVehicleVertex(self, vehicle_index: int, x: float, y: float, threshold: float):
        pts = self.editor_state.surrounding_vehicles[vehicle_index].points_xy
        best_idx = None
        best_d = float("inf")
        for vi, (vx, vy) in enumerate(pts):
            d = self._DistancePoint(x, y, vx, vy)
            if d < best_d and d <= threshold:
                best_d = d; best_idx = vi
        return best_idx

    def _IsCtrlLikePressed(self, event) -> bool:
        try:
            if QtWidgets is not None and QtCore is not None:
                mods = QtWidgets.QApplication.keyboardModifiers()
                return bool(mods & (QtCore.Qt.ControlModifier | QtCore.Qt.MetaModifier))
        except Exception:
            pass
        try:
            if hasattr(event, "guiEvent") and event.guiEvent is not None and QtCore is not None:
                mods = int(event.guiEvent.modifiers())
                return bool(mods & (int(QtCore.Qt.ControlModifier) | int(QtCore.Qt.MetaModifier)))
        except Exception:
            pass
        return False

    # ---------- 라이프사이클 ----------
    def OnModeEntered(self):
        print("[SurroundingVehiclesEditor] Entered. Submodes: view/create/modify/delete")
        print("[SurroundingVehiclesEditor] Create: left-click to add points, Enter or Apply to commit, Ctrl+Z to undo last, Right-click to cancel.")
        self.Redraw()

    # ---------- 커밋/언두 ----------
    def CommitCurrentCreatingSurroundingVehicle(self):
        pts = self.editor_state.current_surrounding_vehicle_points_xy
        if len(pts) < 2:
            return False, "Need at least 2 points to create surrounding vehicle trajectory."
        new_vehicle = SurroundingVehicleTrajectory(
            vehicle_id=self.editor_state.next_surrounding_vehicle_id,
            points_xy=list(pts),
            speed_mps=float(self.editor_state.surrounding_vehicles_create_speed_mps),
        )
        self.editor_state.surrounding_vehicles.append(new_vehicle)
        self.editor_state.next_surrounding_vehicle_id += 1
        pts.clear()
        print(f"[SurroundingVehiclesEditor] Vehicle committed. Total vehicles: {len(self.editor_state.surrounding_vehicles)}")
        self.Redraw()
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()
        return True, "Surrounding vehicle committed."

    def UndoLastCreatingPoint(self):
        pts = self.editor_state.current_surrounding_vehicle_points_xy
        if pts:
            pts.pop()
            self.Redraw()

    # ---------- 입력 ----------
    def OnKeyPress(self, event):
        key_text = (event.key or "").lower()

        # 모드 스위치
        if key_text in ["v"]:
            self.SetSubmode("view"); return
        elif key_text in ["c"]:
            self.SetSubmode("create"); return
        elif key_text in ["m"]:
            self.SetSubmode("modify"); return
        elif key_text in ["d"]:
            self.SetSubmode("delete"); return

        # Undo (Create 모드에서만)
        is_ctrl_like_sequence = key_text in ("ctrl+z", "control+z", "cmd+z", "meta+z", "super+z")
        is_ctrl_like_modifier = (key_text == "z" and self._IsCtrlLikePressed(event))
        is_undo = is_ctrl_like_sequence or is_ctrl_like_modifier or (key_text == "backspace")
        if self.editor_state.surrounding_vehicles_submode == "create" and is_undo:
            if self.editor_state.current_surrounding_vehicle_points_xy:
                self.editor_state.current_surrounding_vehicle_points_xy.pop()
                print(f"[SurroundingVehiclesEditor] Undo last node. Remaining: {len(self.editor_state.current_surrounding_vehicle_points_xy)}")
                self.Redraw()
            return

        # Enter/Return → 커밋
        if key_text in ["enter", "return"]:
            try:
                ok, msg = self.CommitCurrentCreatingSurroundingVehicle()
                if msg:
                    print(f"[SurroundingVehiclesEditor] {msg}")
            except Exception:
                pass
            return

    def OnMousePress(self, event):
        submode = getattr(self.editor_state, "surrounding_vehicles_submode", "view")
        if submode == "view":
            return

        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse

        if event.button == 1:
            # ---------- CREATE ----------
            if submode == "create":
                self.editor_state.current_surrounding_vehicle_points_xy.append((float(x), float(y)))
                self.Redraw()
                return

            # ---------- MODIFY / DELETE ----------
            # 우선 클릭 위치에서 가장 가까운 차량 궤적 자동 선택
            pick_thr = max(
                self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px) * 3.0,
                self._PixelsToDataDistanceAtMouse(event, self.polyline_pick_radius_px) * 3.0
            )
            picked_idx = self._PickNearestVehicleIndex(x, y, accept_threshold=pick_thr)
            if picked_idx is None:
                print("[SurroundingVehiclesEditor] No vehicle near cursor.")
                return

            if self.editor_state.selected_surrounding_vehicle_index != picked_idx:
                self.editor_state.selected_surrounding_vehicle_index = picked_idx
                self.Redraw()

            vehicle = self.editor_state.surrounding_vehicles[picked_idx]

            if submode == "modify":
                if self.editor_state.surrounding_vehicles_modify_mode == "node":
                    vthr = self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px)
                    vthr_expand = vthr * 1.8
                    vidx = self._FindClosestVehicleVertex(picked_idx, x, y, vthr)
                    if vidx is None:
                        vidx = self._FindClosestVehicleVertex(picked_idx, x, y, vthr_expand)
                    if vidx is not None:
                        self.editor_state.selected_surrounding_vehicle_vertex_index = vidx
                        self.moving_vertex = True
                        print(f"[SurroundingVehiclesEditor] Node-move start: vehicle_index={picked_idx}, vertex={vidx}")
                    else:
                        print("[SurroundingVehiclesEditor] No node near cursor.")
                else:
                    # vehicle 강체 이동
                    lthr = self._PixelsToDataDistanceAtMouse(event, self.polyline_pick_radius_px)
                    near_ok = False
                    pts = vehicle.points_xy
                    if len(pts) >= 2:
                        for j in range(len(pts) - 1):
                            x1, y1 = pts[j]; x2, y2 = pts[j + 1]
                            if self._DistancePointToSegment(x, y, x1, y1, x2, y2) <= lthr:
                                near_ok = True; break
                    if not near_ok:
                        for (vx, vy) in pts:
                            if self._DistancePoint(x, y, vx, vy) <= lthr:
                                near_ok = True; break
                    if near_ok:
                        self.moving_vehicle = True
                        self.move_anchor_xy = (x, y)
                        self.original_points_xy = list(vehicle.points_xy)
                        print(f"[SurroundingVehiclesEditor] Vehicle-move start: vehicle_index={picked_idx}")
                    else:
                        print("[SurroundingVehiclesEditor] No near geometry to start vehicle move.")

            elif submode == "delete":
                if self.editor_state.surrounding_vehicles_delete_mode == "node":
                    vthr = self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px)
                    vthr_expand = vthr * 1.8
                    vidx = self._FindClosestVehicleVertex(picked_idx, x, y, vthr)
                    if vidx is None:
                        vidx = self._FindClosestVehicleVertex(picked_idx, x, y, vthr_expand)
                    if vidx is not None:
                        vehicle.points_xy.pop(vidx)
                        if len(vehicle.points_xy) < 2:
                            self.editor_state.surrounding_vehicles.pop(picked_idx)
                            self.editor_state.selected_surrounding_vehicle_index = None
                        print("[SurroundingVehiclesEditor] Node deleted.")
                        self.Redraw()
                        if callable(self.on_geometry_changed):
                            self.on_geometry_changed()
                    else:
                        print("[SurroundingVehiclesEditor] No node near cursor to delete.")
                else:
                    self.editor_state.surrounding_vehicles.pop(picked_idx)
                    self.editor_state.selected_surrounding_vehicle_index = None
                    print("[SurroundingVehiclesEditor] Vehicle deleted.")
                    self.Redraw()
                    if callable(self.on_geometry_changed):
                        self.on_geometry_changed()

        elif event.button == 3:
            if submode == "create":
                self.editor_state.current_surrounding_vehicle_points_xy.clear()
                self.Redraw()

    def OnMouseMove(self, event):
        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse

        # node 이동
        if self.moving_vertex:
            vi = self.editor_state.selected_surrounding_vehicle_vertex_index
            oi = self.editor_state.selected_surrounding_vehicle_index
            if vi is None or oi is None:
                return
            vehicle = self.editor_state.surrounding_vehicles[oi]
            pts = list(vehicle.points_xy)
            pts[vi] = (float(x), float(y))
            vehicle.points_xy = pts
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        # vehicle 강체 이동
        if self.moving_vehicle and self.move_anchor_xy and self.original_points_xy is not None:
            oi = self.editor_state.selected_surrounding_vehicle_index
            if oi is None:
                return
            vehicle = self.editor_state.surrounding_vehicles[oi]
            dx = x - self.move_anchor_xy[0]
            dy = y - self.move_anchor_xy[1]
            vehicle.points_xy = [(px + dx, py + dy) for (px, py) in self.original_points_xy]
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

    def OnMouseRelease(self, event):
        if self.moving_vertex:
            self.moving_vertex = False
            self.editor_state.selected_surrounding_vehicle_vertex_index = None
            print("[SurroundingVehiclesEditor] Node-move end.")
        if self.moving_vehicle:
            self.moving_vehicle = False
            self.move_anchor_xy = None
            self.original_points_xy = None
            print("[SurroundingVehiclesEditor] Vehicle-move end.")
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()

    # ---------- 그리기 ----------
    def Redraw(self):
        xlim = self.axes.get_xlim(); ylim = self.axes.get_ylim()

        # 기존 핸들 제거
        for h in self.editor_state.draw_handles["surrounding_vehicles"]:
            try:
                h.remove()
            except Exception:
                pass
        self.editor_state.draw_handles["surrounding_vehicles"].clear()

        selected_index = self.editor_state.selected_surrounding_vehicle_index
        is_modify_mode = (self.editor_state.surrounding_vehicles_submode == "modify")

        # 스타일
        line_width_normal = 2.0
        line_width_selected = 3.0
        color_normal = "black"
        color_selected = "red"

        # -------- 커밋된 surrounding vehicle들 --------
        for idx, vehicle in enumerate(self.editor_state.surrounding_vehicles):
            points_xy = list(vehicle.points_xy)  # [(x,y), ...]
            if not points_xy:
                continue

            xs = [p[0] for p in points_xy]
            ys = [p[1] for p in points_xy]

            is_selected = (is_modify_mode and idx == selected_index)
            color = color_selected if is_selected else color_normal
            lw = line_width_selected if is_selected else line_width_normal

            # 궤적(폴리라인)
            line, = self.axes.plot(xs, ys, linestyle="--", linewidth=lw, alpha=0.95, color=color)
            self.editor_state.draw_handles["surrounding_vehicles"].append(line)

            # 각 노드의 yaw를 계산해서 박스 렌더링
            yaw_list = self.ComputeYawDegreesPerNode(points_xy)
            for (px, py), yaw_deg in zip(points_xy, yaw_list):
                self.AddVehicleBoxPatch(px, py, yaw_deg, edge_color=color, line_width=lw, alpha=0.95)

            # 진행방향 화살표(마지막 구간)
            if len(points_xy) >= 2:
                # x1, y1 = points_xy[-2]
                # x2, y2 = points_xy[-1]
                # arr = self.axes.annotate(
                #     "", xy=(x2, y2), xytext=(x1, y1),
                #     arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0, linewidth=lw, color=color)
                # )
                # self.editor_state.draw_handles["surrounding_vehicles"].append(arr)
                for seg_index in range(len(points_xy) - 1):
                    x1, y1 = points_xy[seg_index]
                    x2, y2 = points_xy[seg_index + 1]
                    self._AddSegmentArrow(x1, y1, x2, y2, edge_color=color, line_width=lw, alpha=0.95)

        # -------- Create 프리뷰(클릭 중 포인트) --------
        if self.editor_state.surrounding_vehicles_submode == "create" and self.editor_state.current_surrounding_vehicle_points_xy:
            cur_pts = list(self.editor_state.current_surrounding_vehicle_points_xy)
            xs = [p[0] for p in cur_pts]
            ys = [p[1] for p in cur_pts]

            # 점선으로 연결
            cur_line, = self.axes.plot(xs, ys, linestyle="--", linewidth=2.0, alpha=0.85)
            self.editor_state.draw_handles["surrounding_vehicles"].append(cur_line)

            # 각 점을 박스로 (yaw는 위와 동일 규칙)
            yaw_list = self.ComputeYawDegreesPerNode(cur_pts)
            for (px, py), yaw_deg in zip(cur_pts, yaw_list):
                self.AddVehicleBoxPatch(px, py, yaw_deg, edge_color="black", line_width=2.0, alpha=0.9)

            # 마지막 구간 화살표
            if len(cur_pts) >= 2:
                # ax1, ay1 = cur_pts[-2]
                # ax2, ay2 = cur_pts[-1]
                # arr = self.axes.annotate(
                #     "", xy=(ax2, ay2), xytext=(ax1, ay1),
                #     arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0, linewidth=2.0)
                # )
                # self.editor_state.draw_handles["surrounding_vehicles"].append(arr)
                for seg_index in range(len(cur_pts) - 1):
                    ax1, ay1 = cur_pts[seg_index]
                    ax2, ay2 = cur_pts[seg_index + 1]
                    self._AddSegmentArrow(ax1, ay1, ax2, ay2, edge_color="black", line_width=2.0, alpha=0.9)

        # 축 복원 및 갱신
        self.axes.set_xlim(xlim); self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()
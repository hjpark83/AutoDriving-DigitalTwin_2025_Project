import math
from editor_base import EditorBase
from editor_state import EditorState, LanePolyline
try:
    from PyQt5 import QtWidgets, QtCore
except Exception:
    QtWidgets = None
    QtCore = None

from editor_state import EditorState, LanePolyline, LANE_TYPE_NAME_TO_CODE, LANE_TYPE_CODE_TO_NAME


class LanesEditor(EditorBase):
    def __init__(self, editor_state: EditorState, figure, axes):
        super().__init__(editor_state, figure, axes)
        self.moving_vertex = False
        self.moving_lane = False
        self.move_anchor_xy = None
        self.original_points_xyz = None
        self.on_geometry_changed = None


        # 픽킹 반경(픽셀)
        self.vertex_pick_radius_px = 24.0       # 노드 잡기
        self.polyline_pick_radius_px = 18.0     # 선분(폴리라인) 잡기

        # ✅ 추가: Modify/Delete용 undo 스택
        self.undo_stack = []

    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback

    def SetLaneType(self, lane_type: str):
        # enum 이름만 허용
        if lane_type in LANE_TYPE_NAME_TO_CODE:
            self.editor_state.current_lane_type = LANE_TYPE_NAME_TO_CODE[lane_type]
            print(f"[LanesEditor] Lane type -> {lane_type} ({self.editor_state.current_lane_type})")

    # ---- UI hooks ----
    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "modify", "delete"}
        if submode in allowed:
            self.editor_state.lanes_submode = submode
            print(f"[LanesEditor] Submode -> {submode}")

    def SetLaneType(self, lane_type: str):
        allowed = {"driving", "shoulder", "bike", "bus", "parking"}
        if lane_type in allowed:
            self.editor_state.current_lane_type = lane_type
            print(f"[LanesEditor] Lane type -> {lane_type}")

    # ---- helpers ----
    def _PixelsToDataDistanceAtMouse(self, event, pixel_radius: float = 28.0) -> float:
        try:
            px, py = event.x, event.y
            inv = self.axes.transData.inverted()
            x0, y0 = inv.transform((px, py))
            x1, y1 = inv.transform((px + pixel_radius, py))
            x2, y2 = inv.transform((px, py + pixel_radius))
            dx = abs(x1 - x0)
            dy = abs(y2 - y0)
            return float(max(dx, dy))
        except Exception:
            return 1.0

    def _FindClosestLaneVertexLimited(self, lane_index: int, x: float, y: float, threshold: float):
        lane = self.editor_state.lanes[lane_index]
        best_vertex = None
        best_dist = float("inf")
        for vidx, point in enumerate(lane.points_xyz):
            d = math.hypot(point[0] - x, point[1] - y)
            if d < best_dist and d <= threshold:
                best_dist = d
                best_vertex = vidx
        return (lane_index, best_vertex) if best_vertex is not None else (None, None)

    def CommitCurrentCreatingLane(self):
        pts = self.editor_state.current_lane_points_xyz
        if len(pts) < 2:
            return False, "Need at least 2 points to create a lane."

        new_lane = LanePolyline(
            lane_id=self.editor_state.next_lane_id,
            lane_type=self.editor_state.current_lane_type,  # int 코드(0~5) 유지
            points_xyz=list(pts),  # ★ 클릭한 모든 노드를 그대로 살림
        )

        self.editor_state.lanes.append(new_lane)
        self.editor_state.next_lane_id += 1
        pts.clear()
        print(f"[LanesEditor] Lane committed. Total lanes: {len(self.editor_state.lanes)}")
        self.Redraw()
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()
        return True, "Lane committed."

    def UndoLastCreatingPoint(self):
        pts = self.editor_state.current_lane_points_xyz
        if pts:
            pts.pop()
            self.Redraw()

    # ---- lifecycle ----
    def OnModeEntered(self):
        print("[LanesEditor] Entered. Submodes: view/create/modify/delete")
        print("[LanesEditor] Create: left-click to add points, Enter or Apply to commit, Ctrl+Z to undo last, Right-click to cancel.")
        self.undo_stack.clear()  # (선택) 모드 들어올 때 과거 스냅샷 비우기
        self.Redraw()

    def _IsCtrlLikePressed(self, event) -> bool:
        """
        가장 신뢰도 높은 방법: QApplication.keyboardModifiers()로 Ctrl/Meta 감지.
        (백엔드별 event.key 문자열이 'z'만 오는 경우도 커버)
        """
        try:
            if QtWidgets is not None and QtCore is not None:
                mods = QtWidgets.QApplication.keyboardModifiers()
                return bool(mods & (QtCore.Qt.ControlModifier | QtCore.Qt.MetaModifier))
        except Exception:
            pass

        # fallback: guiEvent에 modifiers가 담겨오는 경우(백업 경로)
        try:
            if hasattr(event, "guiEvent") and event.guiEvent is not None and QtCore is not None:
                mods = int(event.guiEvent.modifiers())
                return bool(mods & (int(QtCore.Qt.ControlModifier) | int(QtCore.Qt.MetaModifier)))
        except Exception:
            pass

        return False


    # ✅ 추가: 클래스 내부 메서드 2개
    def _PushUndoSnapshot(self):
        """현재 lanes와 선택 인덱스를 깊은 복사로 저장."""
        import copy
        snap = (copy.deepcopy(self.editor_state.lanes),
                self.editor_state.selected_lane_index)
        self.undo_stack.append(snap)

    def _Undo(self):
        if not self.undo_stack:
            return
        import copy
        lanes, sel = self.undo_stack.pop()
        self.editor_state.lanes = copy.deepcopy(lanes)
        self.editor_state.selected_lane_index = sel
        self.Redraw()
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()

    def OnKeyPress(self, event):
        key_text = (event.key or "").lower()

        # ---- 모드 전환 ----
        if key_text in ["v"]:
            self.SetSubmode("view"); return
        elif key_text in ["c"]:
            self.SetSubmode("create"); return
        elif key_text in ["m"]:
            self.SetSubmode("modify"); return
        elif key_text in ["d"]:
            self.SetSubmode("delete"); return

        # ---- Undo: Ctrl(or Meta)+Z, 또는 백스페이스 (Create 모드에서만) ----
        is_ctrl_like_sequence = key_text in ("ctrl+z", "control+z", "cmd+z", "meta+z", "super+z")
        is_ctrl_like_modifier = (key_text == "z" and self._IsCtrlLikePressed(event))
        is_undo = is_ctrl_like_sequence or is_ctrl_like_modifier or (key_text == "backspace")

        if self.editor_state.lanes_submode == "create" and is_undo:
            if self.editor_state.current_lane_points_xyz:
                self.editor_state.current_lane_points_xyz.pop()
                print(f"[LanesEditor] Undo last node. Remaining: {len(self.editor_state.current_lane_points_xyz)}")
                self.Redraw()
            return
        
        if self.editor_state.lanes_submode in ("modify", "delete") and is_undo:
            self._Undo()
            return

        # (선택) Enter/Return 커밋 로직이 있으면 유지
        if key_text in ["enter", "return"]:
            if hasattr(self, "CommitCurrentCreatingLane"):
                try:
                    ok, msg = self.CommitCurrentCreatingLane()
                    if msg: print(f"[LanesEditor] {msg}")
                except Exception:
                    pass
            return

        # Enter/Return 으로 커밋하는 로직이 있다면 여기서 처리 (프로젝트 구현에 맞게 유지)
        if key_text in ["enter", "return"]:
            if hasattr(self, "CommitCurrentCreatingLane"):
                try:
                    ok, msg = self.CommitCurrentCreatingLane()
                    if msg: print(f"[LanesEditor] {msg}")
                except Exception:
                    pass
            return

    def OnMousePress(self, event):
        if getattr(self.editor_state, "lanes_submode", "view") == "view":
            return

        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse
        distance_threshold = self._PixelsToDataDistanceAtMouse(event, pixel_radius=28.0)
        expanded_threshold = distance_threshold * 2.5

        selected_idx = self.editor_state.selected_lane_index

        if event.button == 1:
            # ---------- CREATE: 점을 계속 추가 ----------
            if self.editor_state.lanes_submode == "create":
                self.editor_state.current_lane_points_xyz.append((x, y, 0.0))
                self.Redraw()
                return

            # ---------- MODIFY / DELETE ----------
            # 1) 항상 캔버스 클릭으로 '가장 가까운 레인' 자동 선택
            #    (이미 선택돼 있어도, 더 가까운 레인을 찍으면 그 레인으로 교체)
            pick_thr = max(
                self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px) * 3.0,
                self._PixelsToDataDistanceAtMouse(event, self.polyline_pick_radius_px) * 3.0
            )
            picked_idx = self._PickNearestLaneIndex(x, y, accept_threshold=pick_thr)
            if picked_idx is None:
                print("[LanesEditor] No lane near cursor.")
                return

            if self.editor_state.selected_lane_index != picked_idx:
                self.editor_state.selected_lane_index = picked_idx
                self.Redraw()

            selected_idx = picked_idx
            lane = self.editor_state.lanes[selected_idx]

            if self.editor_state.lanes_submode == "modify":
                if self.editor_state.lanes_modify_mode == "node":
                    # 가까운 노드 잡아 드래그 시작
                    vthr = self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px)
                    vthr_expand = vthr * 1.8
                    vidx = self._FindClosestLaneVertex(selected_idx, x, y, vthr)
                    if vidx is None:
                        vidx = self._FindClosestLaneVertex(selected_idx, x, y, vthr_expand)
                    if vidx is not None:
                        self._PushUndoSnapshot()   # ✅ 추가
                        self.editor_state.selected_vertex_index = vidx
                        self.moving_vertex = True
                        print(f"[LanesEditor] Node-move start: lane_index={selected_idx}, vertex={vidx}")
                    else:
                        print("[LanesEditor] No node near cursor.")
                else:
                    # lane mode: 폴리라인 근처면 전체 이동 시작
                    lthr = self._PixelsToDataDistanceAtMouse(event, self.polyline_pick_radius_px)
                    # 선분 혹은 노드 중 가까운 지점이 lthr 이내면 강체 이동 시작
                    near_ok = False
                    if len(lane.points_xyz) >= 2:
                        for j in range(len(lane.points_xyz) - 1):
                            x1, y1, _ = lane.points_xyz[j]
                            x2, y2, _ = lane.points_xyz[j + 1]
                            if self._DistancePointToSegment(x, y, x1, y1, x2, y2) <= lthr:
                                near_ok = True; break
                    if not near_ok:
                        # 노드로도 한 번 확인
                        for (lx, ly, _lz) in lane.points_xyz:
                            if self._DistancePoint(x, y, lx, ly) <= lthr:
                                near_ok = True; break
                    if near_ok:
                        self._PushUndoSnapshot()
                        self.moving_lane = True
                        self.move_anchor_xy = (x, y)
                        self.original_points_xyz = list(lane.points_xyz)
                        print(f"[LanesEditor] Lane-move start: lane_index={selected_idx}")
                    else:
                        print("[LanesEditor] No near geometry to start lane move.")

            elif self.editor_state.lanes_submode == "delete":
                if self.editor_state.lanes_delete_mode == "node":
                    vthr = self._PixelsToDataDistanceAtMouse(event, self.vertex_pick_radius_px)
                    vthr_expand = vthr * 1.8
                    vidx = self._FindClosestLaneVertex(selected_idx, x, y, vthr)
                    if vidx is None:
                        vidx = self._FindClosestLaneVertex(selected_idx, x, y, vthr_expand)
                    if vidx is not None:
                        self._PushUndoSnapshot()   # ✅ 추가
                        lane.points_xyz.pop(vidx)
                        if len(lane.points_xyz) < 2:
                            self.editor_state.lanes.pop(selected_idx)
                            self.editor_state.selected_lane_index = None
                        print("[LanesEditor] Node deleted.")
                        self.Redraw()
                    else:
                        print("[LanesEditor] No node near cursor to delete.")
                else:
                    # lane delete: 클릭한 곳과 가장 가까운 레인 전체 삭제
                    self._PushUndoSnapshot()           # ✅ 추가
                    self.editor_state.lanes.pop(selected_idx)
                    self.editor_state.selected_lane_index = None
                    print("[LanesEditor] Lane deleted.")
                    self.Redraw()
        

        elif event.button == 3:
            if self.editor_state.lanes_submode == "create":
                self.editor_state.current_lane_points_xyz.clear()
                self.Redraw()

    def OnMouseMove(self, event):
        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse

        # node move
        if self.moving_vertex:
            li = self.editor_state.selected_lane_index
            vi = self.editor_state.selected_vertex_index
            if li is None or vi is None:
                return
            lane = self.editor_state.lanes[li]
            z = lane.points_xyz[vi][2]
            lane.points_xyz[vi] = (x, y, z)
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        # lane move
        if self.moving_lane and self.move_anchor_xy and self.original_points_xyz is not None:
            li = self.editor_state.selected_lane_index
            if li is None:
                return
            lane = self.editor_state.lanes[li]
            dx = x - self.move_anchor_xy[0]
            dy = y - self.move_anchor_xy[1]
            lane.points_xyz = [(px + dx, py + dy, pz) for (px, py, pz) in self.original_points_xyz]
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

    def OnMouseRelease(self, event):
        if self.moving_vertex:
            self.moving_vertex = False
            self.editor_state.selected_vertex_index = None
            print("[LanesEditor] Node-move end.")
        if self.moving_lane:
            self.moving_lane = False
            self.move_anchor_xy = None
            self.original_points_xyz = None
            print("[LanesEditor] Lane-move end.")
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()


    def _DistancePoint(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def _DistancePointToSegment(self, px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        # 선분 (x1,y1)-(x2,y2) 에 대한 점 (px,py)의 최단거리
        vx, vy = x2 - x1, y2 - y1
        wx, wy = px - x1, py - y1
        seg_len2 = vx * vx + vy * vy
        if seg_len2 <= 1e-12:
            return self._DistancePoint(px, py, x1, y1)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len2))
        proj_x = x1 + t * vx
        proj_y = y1 + t * vy
        return self._DistancePoint(px, py, proj_x, proj_y)

    def _PickNearestLaneIndex(self, x: float, y: float, accept_threshold: float):
        # 각 레인에 대해: 모든 노드와 모든 선분까지의 최소거리 중 최솟값을 비교
        best_i = None
        best_d = float("inf")
        for i, lane in enumerate(self.editor_state.lanes):
            if len(lane.points_xyz) == 0:
                continue
            # 노드 거리
            for (lx, ly, _lz) in lane.points_xyz:
                d = self._DistancePoint(x, y, lx, ly)
                if d < best_d:
                    best_d = d
                    best_i = i
            # 선분 거리
            for j in range(len(lane.points_xyz) - 1):
                x1, y1, _ = lane.points_xyz[j]
                x2, y2, _ = lane.points_xyz[j + 1]
                d = self._DistancePointToSegment(x, y, x1, y1, x2, y2)
                if d < best_d:
                    best_d = d
                    best_i = i
        return best_i if (best_i is not None and best_d <= accept_threshold) else None

        # ---- lane mark helpers (parallel offsets) ----
    def _OffsetPolyline(self, points_xy, offset_dist):
        """
        주어진 (x,y) 폴리라인을 법선 방향으로 offset_dist 만큼 평행이동한 폴리라인 반환.
        points_xy: [(x,y,z), ...] or [(x,y), ...] 섞여도 OK
        """
        if len(points_xy) < 2:
            return [(float(points_xy[0][0]), float(points_xy[0][1]))] if points_xy else []

        xs = [float(p[0]) for p in points_xy]
        ys = [float(p[1]) for p in points_xy]

        # 각 점에서의 접선 벡터를 만들고, 그 법선을 평균해서 부드럽게
        dirs = []
        n = len(xs)
        for i in range(n):
            if i == 0:
                dx = xs[1] - xs[0]; dy = ys[1] - ys[0]
            elif i == n - 1:
                dx = xs[-1] - xs[-2]; dy = ys[-1] - ys[-2]
            else:
                dx = (xs[i+1] - xs[i-1]) * 0.5
                dy = (ys[i+1] - ys[i-1]) * 0.5
            length = math.hypot(dx, dy) or 1.0
            tx, ty = dx / length, dy / length                 # unit tangent
            nx, ny = -ty, tx                                   # unit normal (좌측법선)
            dirs.append((nx, ny))

        out = []
        for (x, y), (nx, ny) in zip(zip(xs, ys), dirs):
            out.append((x + nx * offset_dist, y + ny * offset_dist))
        return out

    # def _PlotLine(self, pts_xy, linewidth=2.0, color="black", linestyle="-",
    #             zorder=3, dashes=None, alpha=0.95, label=None,
    #             cap="round", join="round"):
    #     xs = [p[0] for p in pts_xy]; ys = [p[1] for p in pts_xy]
    #     (h,) = self.axes.plot(xs, ys,
    #                         linewidth=linewidth, color=color, linestyle=linestyle,
    #                         zorder=zorder, alpha=alpha, label=label,
    #                         solid_capstyle=cap, solid_joinstyle=join)
    #     if dashes is not None:
    #         h.set_dashes(dashes)
    #     return h

    def _PlotLine(self, pts, **kw):
        """pts=[(x,y),...]; axes.plot 래퍼. plot에 없는 옵션도 안전 처리."""
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        # plot에는 없는 옵션들 미리 꺼내서 사후 적용
        dash_capstyle = kw.pop("dash_capstyle", None)

        line, = self.axes.plot(xs, ys, **kw)

        if dash_capstyle:
            try:
                line.set_dash_capstyle(dash_capstyle)  # 'butt' | 'round' | 'projecting'
            except Exception:
                pass
        return line

    def _FindClosestLaneVertex(self, lane_index: int, x: float, y: float, threshold: float):
        # 선택된 레인 내에서 가장 가까운 노드 찾기
        lane = self.editor_state.lanes[lane_index]
        best_vertex = None
        best_dist = float("inf")
        for vidx, (vx, vy, _vz) in enumerate(lane.points_xyz):
            d = self._DistancePoint(x, y, vx, vy)
            if d < best_dist and d <= threshold:
                best_dist = d
                best_vertex = vidx
        return best_vertex

    # def Redraw(self):
    #     xlim = self.axes.get_xlim(); ylim = self.axes.get_ylim()
    #     for h in self.editor_state.draw_handles["lanes"]:
    #         try: h.remove()
    #         except Exception: pass
    #     self.editor_state.draw_handles["lanes"].clear()

    #     selected_idx = self.editor_state.selected_lane_index
    #     is_move_mode = (self.editor_state.lanes_submode == "modify")  # ← 기존 "move" 를 "modify"로 쓰는 경우를 고려

    #     for idx, lane in enumerate(self.editor_state.lanes):
    #         xs = [p[0] for p in lane.points_xyz]
    #         ys = [p[1] for p in lane.points_xyz]
    #         color = "red" if (is_move_mode and idx == selected_idx) else "black"
    #         lw = 3.2 if (is_move_mode and idx == selected_idx) else 2.0

    #         # 본선
    #         line, = self.axes.plot(
    #             xs, ys, linestyle="--", linewidth=lw, alpha=0.95,
    #             color=color, label=f"lane_{lane.lane_id} ({lane.lane_type})"
    #         )
    #         self.editor_state.draw_handles["lanes"].append(line)

    #         # 기존 작은 점(유지)
    #         pts = self.axes.scatter(xs, ys, s=36, marker=".", alpha=0.95, c=color)
    #         self.editor_state.draw_handles["lanes"].append(pts)

    #         # ★ 중간 노드(첫/끝 제외): 윤곽선만 있는 큰 원
    #         if len(xs) >= 3:
    #             mid_xs = xs[1:-1]
    #             mid_ys = ys[1:-1]
    #             mid_ring = self.axes.scatter(
    #                 mid_xs, mid_ys,
    #                 s=180, marker="o",
    #                 facecolors="none", edgecolors=color,
    #                 linewidths=2.0, alpha=0.95
    #             )
    #             self.editor_state.draw_handles["lanes"].append(mid_ring)

    #         # 시작/끝 강조(기존 그대로)
    #         sx, sy, _ = lane.points_xyz[0]; ex, ey, _ = lane.points_xyz[-1]
    #         h_s = self.axes.scatter([sx], [sy], s=90, marker="o", c=color)
    #         h_e = self.axes.scatter([ex], [ey], s=90, marker="s", c=color)
    #         self.editor_state.draw_handles["lanes"].extend([h_s, h_e])

    #         # 진행 방향 화살표(기존 그대로)
    #         if len(lane.points_xyz) >= 2:
    #             x1, y1, _ = lane.points_xyz[-2]; x2, y2, _ = lane.points_xyz[-1]
    #             arr = self.axes.annotate(
    #                 "", xy=(x2, y2), xytext=(x1, y1),
    #                 arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0,
    #                                 linewidth=lw, color=color)
    #             )
    #             self.editor_state.draw_handles["lanes"].append(arr)
    #     # ---- creating preview (points you are clicking now) ----
    #     if self.editor_state.lanes_submode == "create" and self.editor_state.current_lane_points_xyz:
    #         xs = [p[0] for p in self.editor_state.current_lane_points_xyz]
    #         ys = [p[1] for p in self.editor_state.current_lane_points_xyz]

    #         # 점들을 점선으로 연결
    #         cur_line, = self.axes.plot(
    #             xs, ys, linestyle="--", linewidth=2.0, alpha=0.8
    #         )
    #         # 클릭한 지점들에 X 마커
    #         cur_pts = self.axes.scatter(
    #             xs, ys, s=70, marker="x", alpha=0.9
    #         )
    #         self.editor_state.draw_handles["lanes"].extend([cur_line, cur_pts])

    #         # 마지막 구간 진행 방향 표시
    #         if len(xs) >= 2:
    #             arrow = self.axes.annotate(
    #                 "",
    #                 xy=(xs[-1], ys[-1]),
    #                 xytext=(xs[-2], ys[-2]),
    #                 arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0, linewidth=2.0)
    #             )
    #             self.editor_state.draw_handles["lanes"].append(arrow)

    #         # ★ 클릭 중인 각 점에도 '윤곽선만 있는 큰 원' 추가(요청 반영)
    #         ring = self.axes.scatter(
    #             xs, ys,
    #             s=180, marker="o",
    #             facecolors="none", edgecolors="black", linewidths=1.5, alpha=0.85
    #         )
    #         self.editor_state.draw_handles["lanes"].append(ring)

    #     try:
    #         if self.editor_state.lanes or self.editor_state.current_lane_points_xyz:
    #             self.axes.legend(loc="upper right", fontsize=8)
    #     except Exception:
    #         pass

    #     self.axes.set_xlim(xlim)
    #     self.axes.set_ylim(ylim)
    #     self.figure.canvas.draw_idle()

    def Redraw(self):
        xlim = self.axes.get_xlim(); ylim = self.axes.get_ylim()
        for h in self.editor_state.draw_handles["lanes"]:
            try: h.remove()
            except Exception: pass
        self.editor_state.draw_handles["lanes"].clear()

        selected_idx = self.editor_state.selected_lane_index
        is_modify_mode = (self.editor_state.lanes_submode == "modify")

        # 시각 파라미터
        offset_dist = 0.30                  # 중앙선에서 좌우 보조선까지 오프셋 (m)
        lw_center   = 2.2
        lw_side     = 2.4
        lw_sel_gain = 1.2                   # 선택 시 라인 두께 가중치
        dash_white  = (4, 3)                # 흰색 점선 패턴
        dash_yellow = (4, 3)               # 노란 점선 패턴

        for idx, lane in enumerate(self.editor_state.lanes):
            pts_c = [(p[0], p[1]) for p in lane.points_xyz]
            if len(pts_c) < 2:
                continue

            selected = (is_modify_mode and idx == selected_idx)

            # --- 공통 파라미터 ---
            lane_type  = int(getattr(lane, "lane_type", 0))
            offset_dist = 0.30
            lw_center   = 2.2
            lw_side     = 2.4
            sel_boost   = (1.2 if selected else 1.0)
            dash_white  = (8, 8)
            dash_yellow = (10, 7)

            pts_l = self._OffsetPolyline(lane.points_xyz, +offset_dist)
            pts_r = self._OffsetPolyline(lane.points_xyz, -offset_dist)

            # ---------- 타입별 본선 렌더 ----------
            if lane_type == 0:
                # TYPE_UNKNOWN: 기본 점선
                h = self._PlotLine(pts_c, linewidth=2.0*sel_boost, color="black",
                                linestyle="--", zorder=6)
                self.editor_state.draw_handles["lanes"].append(h)

            elif lane_type == 1:
                # TYPE_WHITE_SOLID: 검은 옆선 + 중앙 흰 실선(검은 underlay 뒤 흰 overlay)
                h1 = self._PlotLine(pts_l, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h2 = self._PlotLine(pts_r, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h3u = self._PlotLine(pts_c, linewidth=(lw_center+1.6)*sel_boost, color="black", zorder=9)
                h3  = self._PlotLine(pts_c, linewidth=lw_center*sel_boost, color="white", zorder=10)
                self.editor_state.draw_handles["lanes"].extend([h1, h2, h3u, h3])

            elif lane_type == 2:
                # TYPE_WHITE_DASHED: 옆 검은 실선 + 중앙 흰 점선(검 underlay + 흰 overlay)
                # h1 = self._PlotLine(pts_l, linewidth=lw_side*sel_boost, color="black", zorder=8)
                # h2 = self._PlotLine(pts_r, linewidth=lw_side*sel_boost, color="black", zorder=8)
                # h3u = self._PlotLine(pts_c, linewidth=(lw_center+1.2)*sel_boost, color="black",
                #                     linestyle="--", dashes=dash_white, zorder=9)
                # h3  = self._PlotLine(pts_c, linewidth=lw_center*sel_boost, color="white",
                #                     linestyle="--", dashes=dash_white, zorder=10)
                h1 = self._PlotLine(pts_l, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h2 = self._PlotLine(pts_r, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h3u = self._PlotLine(
                    pts_c, linewidth=(lw_center+1.2)*sel_boost, color="black",
                    linestyle="--", dashes=dash_white, dash_capstyle="butt", zorder=9
                )
                h3  = self._PlotLine(
                    pts_c, linewidth=lw_center*sel_boost, color="white",
                    linestyle="--", dashes=dash_white, dash_capstyle="butt", zorder=10
                )
                self.editor_state.draw_handles["lanes"].extend([h1, h2, h3u, h3])

            elif lane_type == 3:
                # TYPE_YELLOW_SOLID: 옆 검은 실선 + 중앙 노란 실선
                h1 = self._PlotLine(pts_l, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h2 = self._PlotLine(pts_r, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h3 = self._PlotLine(pts_c, linewidth=lw_center*sel_boost, color="gold",  zorder=10)
                self.editor_state.draw_handles["lanes"].extend([h1, h2, h3])

            elif lane_type == 4:
                # TYPE_YELLOW_DASHED
                # 1) 양옆 검은 보조선(그대로)
                h1 = self._PlotLine(pts_l, linewidth=lw_side*sel_boost, color="black", zorder=8)
                h2 = self._PlotLine(pts_r, linewidth=lw_side*sel_boost, color="black", zorder=8)

                # 2) 중앙선: 검은 실선 언더레이 → 위에 노란 점선 오버레이
                #    언더레이를 살짝 두껍게 해서 점선의 '빈칸'이 검게 보이도록
                h3_under = self._PlotLine(
                    pts_c, linewidth=(lw_center + 1.2) * sel_boost, color="black", zorder=9
                )
                h3_top = self._PlotLine(
                    pts_c, linewidth=lw_center * sel_boost, color="gold",
                    linestyle="--", dashes=dash_yellow, zorder=10
                )
                # 점선 끝 모양이 캡으로 메워지지 않도록
                try:
                    h3_top.set_dash_capstyle("butt")
                except Exception:
                    pass

                self.editor_state.draw_handles["lanes"].extend([h1, h2, h3_under, h3_top])

            elif lane_type == 5:
                # 옆 노란 점선(검은 언더레이) + 중앙 검은 실선
                # 1) 사이드 언더레이(검은 실선) — 점선보다 살짝 두껍게
                h1_under = self._PlotLine(pts_l, linewidth=(lw_side + 1.2) * sel_boost, color="black", zorder=9)
                h2_under = self._PlotLine(pts_r, linewidth=(lw_side + 1.2) * sel_boost, color="black", zorder=9)

                # 2) 사이드 탑(노란 점선) — 언더레이 위에 올림
                h1 = self._PlotLine(
                    pts_l, linewidth=lw_side * sel_boost, color="gold",
                    linestyle="--", dashes=dash_yellow, zorder=10
                )
                h2 = self._PlotLine(
                    pts_r, linewidth=lw_side * sel_boost, color="gold",
                    linestyle="--", dashes=dash_yellow, zorder=10
                )
                # 점선 끝이 둥글게 메워지지 않도록(캡 제거)
                try:
                    h1.set_dash_capstyle("butt")
                    h2.set_dash_capstyle("butt")
                except Exception:
                    pass

                # 3) 중앙 검은 실선
                h0 = self._PlotLine(pts_c, linewidth=lw_center * sel_boost, color="black", zorder=11)

                self.editor_state.draw_handles["lanes"].extend([h1_under, h2_under, h1, h2, h0])

            else:
                h = self._PlotLine(pts_c, linewidth=2.0*sel_boost, color="black",
                                linestyle="--", zorder=6)
                self.editor_state.draw_handles["lanes"].append(h)

            # ---------- 선택 하이라이트(주변 붉게) ----------
            if selected:
                glow_side = self._PlotLine(pts_l, linewidth=(lw_side+2.2), color="red",
                                        alpha=0.25, zorder=7)
                glow_side2 = self._PlotLine(pts_r, linewidth=(lw_side+2.2), color="red",
                                            alpha=0.25, zorder=7)
                glow_c = self._PlotLine(pts_c, linewidth=(lw_center+2.2), color="red",
                                        alpha=0.22, zorder=7)
                self.editor_state.draw_handles["lanes"].extend([glow_side, glow_side2, glow_c])

            # ---------- 노드 표시는 그대로 유지 ----------
            xs = [p[0] for p in lane.points_xyz]
            ys = [p[1] for p in lane.points_xyz]
            node_color = "red" if selected else "black"
            z_nodes = 20  # 선 위에 보이도록

            # 작은 점(전체)
            pts = self.axes.scatter(xs, ys, s=36, marker=".", alpha=0.95,
                                    c=node_color, zorder=z_nodes)
            self.editor_state.draw_handles["lanes"].append(pts)

            # 중간 노드 링(첫/끝 제외)
            if len(xs) >= 3:
                mid_xs = xs[1:-1]; mid_ys = ys[1:-1]
                mid_ring = self.axes.scatter(
                    mid_xs, mid_ys, s=180, marker="o",
                    facecolors="none", edgecolors=node_color,
                    linewidths=2.0, alpha=0.95, zorder=z_nodes+1
                )
                self.editor_state.draw_handles["lanes"].append(mid_ring)

            # 시작/끝 강조(수정 버전: 끝점은 빈 사각형으로)
            sx, sy, _ = lane.points_xyz[0]
            ex, ey, _ = lane.points_xyz[-1]
            # node_color = color  # 선택 시 red, 그 외 black 그대로 따라감

            h_s = self.axes.scatter([sx], [sy], s=90, marker="o",
                        c=("red" if selected else "black"),
                        zorder=z_nodes)
            # 끝점: 항상 '검은색 채운 사각형'
            h_e = self.axes.scatter([ex], [ey], s=90, marker="s",
                                    c="black", edgecolors="black",
                                    zorder=z_nodes+1)
            self.editor_state.draw_handles["lanes"].extend([h_s, h_e])

            # --- 여기서부터 '끝점 캡' 추가 (중앙선 색이 끝점에서도 보이도록) ---
            # lane_type = int(lane.lane_type)
            # if   lane_type in (1, 2):  cap_color = "white"   # WHITE_* 계열
            # elif lane_type in (3, 4):  cap_color = "gold"    # YELLOW_* 계열
            # elif lane_type == 5:       cap_color = "black"   # 가운데 검정
            # else:                      cap_color = "black"   # unknown 기본

            # ax_face = self.axes.get_facecolor()
            # cap_mask = self.axes.scatter(
            #     [ex], [ey], s=120, marker="s",
            #     c=[ax_face], edgecolors='none', linewidths=0, zorder=99
            # )
            # self.editor_state.draw_handles["lanes"].append(cap_mask)

            # 2) 실제 캡: 중앙선 색으로 덮기 (테두리 없음)
            # cap = self.axes.scatter(
            #     [ex], [ey], s=70, marker="s",
            #     c=cap_color, edgecolors='none', linewidths=0, zorder=100
            # )
            # self.editor_state.draw_handles["lanes"].append(cap)
        

            # 진행방향 화살표(노드 색과 동일)
            if len(lane.points_xyz) >= 2:
                x1, y1, _ = lane.points_xyz[-2]; x2, y2, _ = lane.points_xyz[-1]

                # 중앙선 색 결정(타입별)
                lt = int(lane.lane_type)
                if   lt in (1, 2):   dir_color = "white"  # WHITE_* 계열
                elif lt in (3, 4):   dir_color = "gold"   # YELLOW_* 계열
                elif lt == 5:        dir_color = "black"  # 중앙 검정
                else:                dir_color = "black"  # unknown

                arr = self.axes.annotate(
                    "", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(
                        arrowstyle="->", shrinkA=0, shrinkB=0,
                        linewidth=2.4 if selected else 2.0,
                        color=dir_color
                    ),
                    # 중앙선과 같은 색이므로 위에 올려도 위화감 없음
                    zorder=z_nodes+2
                )
                self.editor_state.draw_handles["lanes"].append(arr)

        # ---- creating preview (기존 유지) ----
        if self.editor_state.lanes_submode == "create" and self.editor_state.current_lane_points_xyz:
            xs = [p[0] for p in self.editor_state.current_lane_points_xyz]
            ys = [p[1] for p in self.editor_state.current_lane_points_xyz]
            cur_line, = self.axes.plot(xs, ys, linestyle="--", linewidth=2.0, alpha=0.8, color="black", zorder=3)
            cur_pts = self.axes.scatter(xs, ys, s=70, marker="x", alpha=0.9, color="black", zorder=4)
            self.editor_state.draw_handles["lanes"].extend([cur_line, cur_pts])
            if len(xs) >= 2:
                arrow = self.axes.annotate("", xy=(xs[-1], ys[-1]), xytext=(xs[-2], ys[-2]),
                                           arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0, linewidth=2.0, color="black"),
                                           zorder=5)
                self.editor_state.draw_handles["lanes"].append(arrow)

        # try:
        #     if self.editor_state.lanes or self.editor_state.current_lane_points_xyz:
        #         self.axes.legend(loc="upper right", fontsize=8)
        # except Exception:
        #     pass
        # 교체: 안전하게 legend 갱신
        handles, labels = self.axes.get_legend_handles_labels()
        # 유효한(빈 문자열/None 아님, 그리고 '_nolegend_' 아닌) 라벨이 하나라도 있으면 표시
        valid = [lbl for lbl in labels if lbl and not lbl.startswith("_")]
        if valid:
            self.axes.legend(loc="upper right", fontsize=8)
        else:
            # 기존에 떠 있던 범례가 있으면 제거
            leg = self.axes.get_legend()
            if leg:
                leg.remove()

        self.axes.set_xlim(xlim); self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()


# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_gnss_shadow_zones.py
import math
from typing import Optional, Tuple
import matplotlib.patches as mpatches

from editor_base import EditorBase
from editor_state import GNSSShadowZone


class GnssShadowZonesEditor(EditorBase):
    """
    Submodes: "view" | "create" | "move" | "delete"
    좌클릭 기반 조작:
      - create: 클릭 위치에 (x,y) 생성, pos_std/radius는 editor_state 기본값 사용
      - move  : 가장 가까운 존을 픽업해 드래그로 중심 이동
      - delete: 가까운 존 제거
    """

    def __init__(self, editor_state, figure, axes):
        super().__init__(editor_state, figure, axes)
        self.resizing_radius = False          # ← 원 반지름 드래그 중인지
        self.moving_center = False            # ← 센터 드래그 중인지
        self.moving_zone = False
        self.active_index: Optional[int] = None
        self.on_geometry_changed = None

        # 픽킹 반경(픽셀 단위 → 데이터 거리로 변환해서 사용)
        self.edge_pick_radius_px = 20.0       # ← 테두리 픽킹 반경(픽셀)
        self.center_pick_radius_px = 16.0     # ← 센터 픽킹 반경(픽셀)

        # (이미 있을 수 있는 콜백 그대로 유지)
        self.on_geometry_changed = None

        # 시각화 스타일
        self.edge_color = "navy"
        self.fill_color = "royalblue"
        self.fill_alpha = 0.15
        self.center_size = 40

    # ---- 콜백 ----
    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback

    # ---- 내부 유틸 ----
    def _PixelsToDataDistanceAtMouse(self, event, pixel_radius: float = 24.0) -> float:
        try:
            px, py = float(event.x), float(event.y)
            inv = self.axes.transData.inverted()
            x0, y0 = inv.transform((px, py))
            x1, y1 = inv.transform((px + pixel_radius, py))
            x2, y2 = inv.transform((px, py + pixel_radius))
            return float(max(abs(x1 - x0), abs(y2 - y0)))
        except Exception:
            return 1.0
        
    def _PixelsToDataOffset(self, dx_pixels: float, dy_pixels: float):
        try:
            inv = self.axes.transData.inverted()
            x0, y0 = inv.transform((0.0, 0.0))
            x1, y1 = inv.transform((float(dx_pixels), float(dy_pixels)))
            return float(x1 - x0), float(y1 - y0)
        except Exception:
            return 0.0, 0.0

    def _PickZoneIndexByEdge(self, x: float, y: float, edge_threshold: float):
        best_index = None
        best_delta = float("inf")
        for i, z in enumerate(self.editor_state.gnss_shadow_zones):
            d = self._DistancePoint(x, y, z.x, z.y)
            delta = abs(d - float(z.radius))
            if delta <= edge_threshold and delta < best_delta:
                best_delta = delta
                best_index = i
        return best_index

    def _PickZoneIndexByCenter(self, x: float, y: float, center_threshold: float):
        best_index = None
        best_dist = float("inf")
        for i, z in enumerate(self.editor_state.gnss_shadow_zones):
            d = self._DistancePoint(x, y, z.x, z.y)
            if d <= center_threshold and d < best_dist:
                best_dist = d
                best_index = i
        return best_index

    def _DistancePoint(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5

    def _FindNearestIndex(self, x: float, y: float, accept_threshold: float) -> Optional[int]:
        best_i = None
        best_d = float("inf")
        for i, z in enumerate(self.editor_state.gnss_shadow_zones):
            d = math.hypot(z.x - x, z.y - y)
            if d < best_d:
                best_d = d
                best_i = i
        if best_i is not None and best_d <= accept_threshold:
            return best_i
        return None

    # ---- 수명주기 ----
    def OnModeEntered(self):
        print("[GnssShadowZonesEditor] Entered. Submodes: view/create/move/delete")
        self.Redraw()

    # ---- 입력 이벤트 ----
    def OnMousePress(self, event):
        submode = self.editor_state.gnss_shadow_zones_submode
        if submode == "view":
            return
        mouse_xy = self.GetMouseXY(event)
        if mouse_xy is None:
            return
        x, y = mouse_xy

        # ★ 변경: 픽셀→데이터 임계값을 테두리/센터로 분리
        edge_thr = self._PixelsToDataDistanceAtMouse(event, self.edge_pick_radius_px)
        ctr_thr  = self._PixelsToDataDistanceAtMouse(event, self.center_pick_radius_px)

        # CREATE
        if submode == "create" and event.button == 1:
            pos_std = float(self.editor_state.gnss_shadow_zones_create_pos_std)
            radius = float(self.editor_state.gnss_shadow_zones_create_radius)
            self.editor_state.gnss_shadow_zones.append(
                GNSSShadowZone(x=float(x), y=float(y), pos_std=pos_std, radius=radius)
            )
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            print(f"[GnssShadowZonesEditor] Zone created at ({x:.2f},{y:.2f}).")
            return

        # MOVE
        if submode == "move" and event.button == 1:
            # 1) 테두리 히트 → 반지름 리사이즈
            edge_index = None
            best_delta = float("inf")
            for i, z in enumerate(self.editor_state.gnss_shadow_zones):
                dx = x - float(z.x); dy = y - float(z.y)
                dist = (dx * dx + dy * dy) ** 0.5
                delta = abs(dist - float(z.radius))
                if delta <= edge_thr and delta < best_delta:
                    best_delta = delta
                    edge_index = i
            if edge_index is not None:
                self.editor_state.selected_gnss_shadow_zone_index = edge_index
                self.resizing_radius = True
                self.moving_zone = False
                self.Redraw()
                # ★ NEW: 리스트 선택도 즉시 반영되도록 (선택 변경을 알림)
                if callable(self.on_geometry_changed):
                    self.on_geometry_changed()
                return

            # 2) 센터 근처면 이동
            i = self._FindNearestIndex(x, y, ctr_thr * 1.5)
            if i is None:
                print("[GnssShadowZonesEditor] No zone near cursor.")
                return
            self.editor_state.selected_gnss_shadow_zone_index = i
            self.moving_zone = True
            self.resizing_radius = False
            self.Redraw()
            # ★ NEW: 리스트 선택도 즉시 반영되도록 (선택 변경을 알림)
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        # DELETE
        if submode == "delete" and event.button == 1:
            # 센터 기준으로 삭제(필요하면 max(edge_thr, ctr_thr)로 느슨하게)
            i = self._FindNearestIndex(x, y, ctr_thr * 1.5)
            if i is None:
                print("[GnssShadowZonesEditor] No zone near cursor.")
                return
            self.editor_state.gnss_shadow_zones.pop(i)
            if self.editor_state.selected_gnss_shadow_zone_index == i:
                self.editor_state.selected_gnss_shadow_zone_index = None
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            print("[GnssShadowZonesEditor] Zone deleted.")
            return


    def OnMouseMove(self, event):
        if self.editor_state.gnss_shadow_zones_submode != "move":
            return
        i = self.editor_state.selected_gnss_shadow_zone_index
        if i is None or not (0 <= i < len(self.editor_state.gnss_shadow_zones)):
            return
        mouse_xy = self.GetMouseXY(event)
        if mouse_xy is None:
            return
        x, y = mouse_xy
        z = self.editor_state.gnss_shadow_zones[i]

        if self.resizing_radius:
            dx = x - float(z.x)
            dy = y - float(z.y)
            new_radius = (dx * dx + dy * dy) ** 0.5
            z.radius = float(max(0.0, new_radius))
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        if not self.moving_zone:
            return
        z.x = float(x)
        z.y = float(y)
        self.Redraw()
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()

    def OnMouseRelease(self, event):
        self.moving_zone = False
        self.resizing_radius = False

    def OnKeyPress(self, event):
        # 현재 별도 단축키 없음 (필요 시 추가)
        pass

    # ---- UI hooks ----
    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "move", "delete"}
        if submode in allowed:
            self.editor_state.gnss_shadow_zones_submode = submode
            print(f"[GnssShadowZonesEditor] Submode -> {submode}")
            self.Redraw()

    def SetCreateDefaults(self, pos_std: float = None, radius: float = None):
        if pos_std is not None:
            self.editor_state.gnss_shadow_zones_create_pos_std = float(pos_std)
        if radius is not None:
            self.editor_state.gnss_shadow_zones_create_radius = float(radius)

    # def Redraw(self):
    #     # 현재 축 범위 보존
    #     xlim = self.axes.get_xlim()
    #     ylim = self.axes.get_ylim()

    #     # 기존 핸들 제거
    #     for h in self.editor_state.draw_handles.get("gnss_shadow_zones", []):
    #         try:
    #             h.remove()
    #         except Exception:
    #             pass
    #     self.editor_state.draw_handles["gnss_shadow_zones"].clear()

    #     # 그리기
    #     for idx, z in enumerate(self.editor_state.gnss_shadow_zones):
    #         # 원(섀도우 반경)
    #         circ = mpatches.Circle(
    #             (z.x, z.y),
    #             radius=z.radius,
    #             facecolor=self.fill_color,
    #             edgecolor=self.edge_color,
    #             linewidth=1.5,
    #             alpha=self.fill_alpha,
    #         )
    #         self.axes.add_patch(circ)
    #         self.editor_state.draw_handles["gnss_shadow_zones"].append(circ)

    #         # 중심점
    #         pts = self.axes.scatter([z.x], [z.y], s=self.center_size, marker="o",
    #                                 c=self.edge_color, edgecolors="black", linewidths=0.5, alpha=0.95)
    #         self.editor_state.draw_handles["gnss_shadow_zones"].append(pts)

    #         # 텍스트 (pos_std 표시)
    #         txt = self.axes.text(z.x, z.y, f"σ={z.pos_std:.2f}", fontsize=8, ha="center", va="bottom")
    #         self.editor_state.draw_handles["gnss_shadow_zones"].append(txt)

    #     # 축 복원 및 렌더
    #     self.axes.set_xlim(xlim)
    #     self.axes.set_ylim(ylim)
    #     self.figure.canvas.draw_idle()
    def Redraw(self):
        xlim = self.axes.get_xlim(); ylim = self.axes.get_ylim()
        for h in self.editor_state.draw_handles["gnss_shadow_zones"]:
            try:
                h.remove()
            except Exception:
                pass
        self.editor_state.draw_handles["gnss_shadow_zones"].clear()

        import matplotlib.patches as mpatches
        base_color = "purple"
        selected_edge_color = "red"
        base_line_width = 2.0
        selected_line_width = 3.0
        face_alpha = 0.06
        center_size = 30

        # σ 라벨을 센터 바로 위에 띄우기 위한 데이터 오프셋
        try:
            inv = self.axes.transData.inverted()
            x0, y0 = inv.transform((0.0, 0.0))
            x1, y1 = inv.transform((0.0, 14.0))  # 14 px 위
            dy_label = float(y1 - y0)
        except Exception:
            dy_label = 0.0

        selected_index = getattr(self.editor_state, "selected_gnss_shadow_zone_index", None)
        is_move_mode = (self.editor_state.gnss_shadow_zones_submode == "move")

        for i, z in enumerate(self.editor_state.gnss_shadow_zones):
            is_selected = (is_move_mode and selected_index is not None and i == selected_index)
            edge_color = selected_edge_color if is_selected else base_color
            line_width = selected_line_width if is_selected else base_line_width

            circ = mpatches.Circle(
                (float(z.x), float(z.y)),
                radius=max(0.0, float(z.radius)),
                facecolor=edge_color,
                edgecolor=edge_color,
                linewidth=line_width,
                alpha=face_alpha,
            )
            self.axes.add_patch(circ)
            self.editor_state.draw_handles["gnss_shadow_zones"].append(circ)

            ctr = self.axes.scatter(
                [float(z.x)], [float(z.y)],
                s=center_size, c=edge_color, edgecolors="black", linewidths=0.4
            )
            self.editor_state.draw_handles["gnss_shadow_zones"].append(ctr)

            # σ 라벨 (선택 시 빨강 강조)
            label_text = f"σ={float(z.pos_std):.2f}"
            txt = self.axes.text(
                float(z.x), float(z.y) + dy_label, label_text,
                ha="center", va="bottom", fontsize=9,
                color=edge_color,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.7)
            )
            self.editor_state.draw_handles["gnss_shadow_zones"].append(txt)

        self.axes.set_xlim(xlim); self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()

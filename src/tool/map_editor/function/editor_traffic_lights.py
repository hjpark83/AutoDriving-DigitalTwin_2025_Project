# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_traffic_lights.py
import math
from editor_base import EditorBase
from editor_state import EditorState, TrafficLightItem
from typing import Optional, List

STATE_CODE_TO_NAME = {0: "red", 1: "yellow", 2: "green"}

class TrafficLightsEditor(EditorBase):
    def __init__(self, editor_state: EditorState, figure, axes):
        super().__init__(editor_state, figure, axes)
        self.current_state_code = 0  # 0:red, 1:yellow, 2:green
        self.current_z = 5.0
        self.moving_light = False
        self.on_geometry_changed = None

        # Undo 스택
        self.undo_stack: List[list] = []
        self.max_undo: int = 50

    # ----- external hooks -----
    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback

    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "modify", "delete"}
        if submode in allowed:
            self.editor_state.traffic_lights_submode = submode
            print(f"[TrafficLightsEditor] Submode -> {submode}")
            self.Redraw()

    def SetCreateDefaults(self, state_code: int = None, z: float = None):
        if state_code is not None:
            self.current_state_code = int(state_code)
        if z is not None:
            self.current_z = float(z)
        print(f"[TrafficLightsEditor] Create defaults -> state={self.current_state_code}, z={self.current_z}")

    def OnModeEntered(self):
        print("[TrafficLightsEditor] Modes: view / create / modify / delete")
        print("[TrafficLightsEditor] Left click: create or start move. Right click: delete nearest.")
        self.Redraw()

    # ----- helpers -----
    def _PixelsToDataDistanceAtMouse(self, event, pixel_radius: float = 28.0) -> float:
        try:
            px, py = event.x, event.y
            inv = self.axes.transData.inverted()
            x0, y0 = inv.transform((px, py))
            x1, y1 = inv.transform((px + pixel_radius, py))
            x2, y2 = inv.transform((px, py + pixel_radius))
            return float(max(abs(x1 - x0), abs(y2 - y0)))
        except Exception:
            return 1.0

    def _FindNearest(self, x: float, y: float, threshold: float):
        best_i = None; best_d = float("inf")
        for i, t in enumerate(self.editor_state.traffic_lights):
            d = math.hypot(t.x - x, t.y - y)
            if d < best_d and d <= threshold:
                best_d = d; best_i = i
        return best_i
    

    # ----- undo helpers -----
    def _Snapshot(self) -> list:
        return [TrafficLightItem(x=traffic_light.x, 
                                 y=traffic_light.y, 
                                 z=traffic_light.z,
                                 state=traffic_light.state)
                for traffic_light in self.editor_state.traffic_lights] 

    def _PushUndoSnapshot(self):
        try:
            snap = self._Snapshot()
            self.undo_stack.append(snap)
            if len(self.undo_stack) > self.max_undo:
                self.undo_stack.pop(0)
        except Exception:
            pass

    def _RestoreFromSnapshot(self, snapshot: list):
        self.editor_state.traffic_lights.clear()
        for traffic_light in snapshot:
            self.editor_state.traffic_lights.append(
                TrafficLightItem(x=traffic_light.x, 
                                 y=traffic_light.y, 
                                 z=traffic_light.z,
                                 state=traffic_light.state)
            )

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
                print("[TrafficLightsEditor] Undo.")
            return

    # ----- mouse -----
    def OnMousePress(self, event):
        mode = getattr(self.editor_state, "traffic_lights_submode", "view")
        if mode == "view":
            return
        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse
        thr = self._PixelsToDataDistanceAtMouse(event, 28.0)
        thr2 = thr * 2.5

        if event.button == 1:
            if mode == "create":
                self._PushUndoSnapshot()
                self.editor_state.traffic_lights.append(TrafficLightItem(
                    x=x, y=y, z=self.current_z, state=int(self.current_state_code)
                ))
                self.editor_state.selected_traffic_light_index = len(self.editor_state.traffic_lights) - 1
                self.Redraw()
                if callable(self.on_geometry_changed): self.on_geometry_changed()
                return
            if mode == "modify":
                idx = self._FindNearest(x, y, thr) or self._FindNearest(x, y, thr2)
                if idx is not None:
                    self._PushUndoSnapshot()
                    self.editor_state.selected_traffic_light_index = idx
                    self.moving_light = True
                    print(f"[TrafficLightsEditor] Move start: index={idx}")
                else:
                    print(f"[TrafficLightsEditor] No light within {thr:.3f} (expanded {thr2:.3f}).")
                return
            if mode == "delete":
                idx = self._FindNearest(x, y, thr) or self._FindNearest(x, y, thr2)
                if idx is not None:
                    self._PushUndoSnapshot()
                    self.editor_state.traffic_lights.pop(idx)
                    if self.editor_state.selected_traffic_light_index is not None:
                        if idx == self.editor_state.selected_traffic_light_index:
                            self.editor_state.selected_traffic_light_index = None
                        elif idx < self.editor_state.selected_traffic_light_index:
                            self.editor_state.selected_traffic_light_index -= 1
                    self.Redraw()
                    if callable(self.on_geometry_changed): self.on_geometry_changed()
                else:
                    print(f"[TrafficLightsEditor] No light within {thr:.3f} (expanded {thr2:.3f}).")

        elif event.button == 3:
            # 우클릭 삭제(보조)
            idx = self._FindNearest(x, y, thr) or self._FindNearest(x, y, thr2)
            if idx is not None:
                self._PushUndoSnapshot()
                self.editor_state.traffic_lights.pop(idx)
                self.Redraw()
                if callable(self.on_geometry_changed): self.on_geometry_changed()

    def OnMouseMove(self, event):
        if not self.moving_light:
            return
        mouse = self.GetMouseXY(event)
        if mouse is None:
            return
        x, y = mouse
        idx = self.editor_state.selected_traffic_light_index
        if idx is None or not (0 <= idx < len(self.editor_state.traffic_lights)):
            return
        t = self.editor_state.traffic_lights[idx]
        t.x = float(x); t.y = float(y)
        self.Redraw()
        if callable(self.on_geometry_changed): self.on_geometry_changed()

    def OnMouseRelease(self, event):
        if self.moving_light:
            self.moving_light = False
            print("[TrafficLightsEditor] Move end.")

    # ----- draw -----
    def Redraw(self):
        # 축 범위 보존
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()

        # 기존 핸들 제거
        for handle in self.editor_state.draw_handles["traffic_lights"]:
            try:
                handle.remove()
            except Exception:
                pass
        self.editor_state.draw_handles["traffic_lights"].clear()

        items = self.editor_state.traffic_lights
        if items:
            # ----- 픽셀 → 데이터 단위 변환 -----
            rect_width_px = 56.0
            rect_height_px = 22.0
            pad_px = 3.0
            circle_radius_px = 6.0
            anchor_offset_px = 30.0  # 기준점에서 직사각형 중심까지 위로 띄우는 양(픽셀)

            inv = self.axes.transData.inverted()
            # 가로 변환
            x0, _ = inv.transform((0.0, 0.0))
            xw, _ = inv.transform((rect_width_px, 0.0))
            xp, _ = inv.transform((pad_px, 0.0))
            xr, _ = inv.transform((circle_radius_px, 0.0))
            rect_width_data = abs(xw - x0)
            pad_data_x = abs(xp - x0)
            circle_radius_data = abs(xr - x0)
            # 세로 변환
            _, y0 = inv.transform((0.0, 0.0))
            _, yh = inv.transform((0.0, rect_height_px))
            _, yo = inv.transform((0.0, anchor_offset_px))
            _, yp = inv.transform((0.0, pad_px))
            rect_height_data = abs(yh - y0)
            anchor_offset_data = abs(yo - y0)
            pad_data_y = abs(yp - y0)

            # 내부 영역 고려하여 반지름 보정
            inner_w = max(1e-9, rect_width_data - 2.0 * pad_data_x)
            inner_h = max(1e-9, rect_height_data - 2.0 * pad_data_y)
            circle_radius_data = min(circle_radius_data, inner_h * 0.5, inner_w * 0.12)

            # 색상
            state_to_color = {0: "red", 1: "yellow", 2: "green"}
            off_face = "#c8c8c8"

            from matplotlib import patches as mpatches

            mode = getattr(self.editor_state, "traffic_lights_submode", "view")
            selected_index = self.editor_state.selected_traffic_light_index

            for index, t in enumerate(items):
                cx = float(t.x)
                cy = float(t.y)
                state_code = int(getattr(t, "state", 0))

                # 이 항목이 'Modify'에서 선택 상태인지
                is_selected_in_modify = (mode == "modify" and
                                        selected_index is not None and
                                        index == selected_index)

                # 사각형 위치(가로형, 기준점 위로 띄우기)
                rect_center_y = cy + anchor_offset_data
                rect_x = cx - rect_width_data * 0.5
                rect_y = rect_center_y - rect_height_data * 0.5

                # 본체 사각형(선택되었을 때만 빨간 테두리)
                rect = mpatches.Rectangle(
                    (rect_x, rect_y),
                    rect_width_data,
                    rect_height_data,
                    linewidth=2.0 if is_selected_in_modify else 1.2,
                    edgecolor=("red" if is_selected_in_modify else "black"),
                    facecolor="#eeeeee",
                    alpha=0.9,
                    zorder=2.0,
                )
                self.axes.add_patch(rect)
                self.editor_state.draw_handles["traffic_lights"].append(rect)

                # 램프(좌-중-우) — 원 외곽선은 항상 검정
                left_center_x = rect_x + pad_data_x + circle_radius_data
                right_center_x = rect_x + rect_width_data - pad_data_x - circle_radius_data
                mid_center_x = 0.5 * (left_center_x + right_center_x)
                center_y = rect_center_y
                centers = [left_center_x, mid_center_x, right_center_x]

                for k, cx_lamp in enumerate(centers):
                    is_on = (k == state_code)
                    face = state_to_color.get(k, off_face) if is_on else off_face
                    lamp = mpatches.Circle(
                        (cx_lamp, center_y),
                        radius=circle_radius_data,
                        facecolor=face,
                        edgecolor="black",
                        linewidth=1.0,
                        alpha=0.95 if is_on else 0.65,
                        zorder=2.5,
                    )
                    self.axes.add_patch(lamp)
                    self.editor_state.draw_handles["traffic_lights"].append(lamp)

                # 기준점 X 마커 + 점선 연결(기준점 → 사각형 아래변)
                x_marker = self.axes.scatter([cx], [cy], marker="x", s=120, linewidths=2.0, c="black", zorder=3.0)
                self.editor_state.draw_handles["traffic_lights"].append(x_marker)

                line_end_y = rect_y  # 사각형 아래변
                dash, = self.axes.plot([cx, cx], [cy, line_end_y],
                                    linestyle=":", linewidth=1.4, color="black", zorder=2.2)
                self.editor_state.draw_handles["traffic_lights"].append(dash)

        # 축 범위 복구
        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()
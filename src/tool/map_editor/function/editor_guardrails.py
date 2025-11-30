# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_guardrails.py
import math
from typing import Optional, List

from editor_base import EditorBase
from editor_state import EditorState, GuardrailItem


def _DegToRad(deg: float) -> float:
    return math.radians(deg)


def _RadToDeg(rad: float) -> float:
    return math.degrees(rad)


class GuardrailsEditor(EditorBase):
    """
    기능 요약
      - Submode: view / create / modify / delete
      - create: 클릭 지점에 (cx, cy)로 새 가드레일 생성 (길이/각도는 Create Settings 사용)
      - modify:
          * 클릭 시 가장 가까운 가드레일 자동 선택
          * 중심(작은 반경) 드래그 → 평행이동
          * 끝점(큰 반경) 드래그 → length + yaw_deg 동시 갱신
      - delete: 선택된 가드레일 삭제
      - Ctrl+Z (또는 Cmd+Z) → 직전 상태로 Undo
      - 시각화: 중앙 선 + 직사각형 외곽 + 끝점(윤곽 원) + 중심(검은 사각형)
    """

    def __init__(self, editor_state: EditorState, figure, axes):
        super().__init__(editor_state, figure, axes)

        # 드래그 상태
        self.is_moving_center: bool = False
        self.active_endpoint_index: Optional[int] = None  # 0 또는 1

        # 픽킹 반경 (픽셀)
        self.center_pick_radius_px: float = 18.0   # 센터는 더 작게
        self.endpoint_pick_radius_px: float = 28.0 # 끝점은 좀 더 크게

        # 직사각형 반폭(데이터 단위)
        self.rect_half_width: float = 0.25

        # Undo 스택
        self.undo_stack: List[list] = []
        self.max_undo: int = 50

        # 외부 콜백
        self.on_geometry_changed = None

    # ---------- UI hooks ----------
    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "modify", "delete"}
        if submode in allowed:
            self.editor_state.guardrails_submode = submode
            print(f"[GuardrailsEditor] Submode -> {submode}")
            self.Redraw()

    def SetCreateDefaults(self, length: float = None, yaw_deg: float = None):
        if length is not None:
            self.editor_state.guardrails_create_length = float(length)
        if yaw_deg is not None:
            self.editor_state.guardrails_create_yaw_deg = float(yaw_deg)
        print(
            f"[GuardrailsEditor] Create defaults -> "
            f"length={getattr(self.editor_state, 'guardrails_create_length', None)}, "
            f"yaw={getattr(self.editor_state, 'guardrails_create_yaw_deg', None)}"
        )

    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback

    # ---------- helpers ----------
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

    def _DistancePoint(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def _ComputeEndpoints(self, cx: float, cy: float, length: float, yaw_deg: float):
        dx = 0.5 * length * math.cos(_DegToRad(yaw_deg))
        dy = 0.5 * length * math.sin(_DegToRad(yaw_deg))
        return (cx - dx, cy - dy), (cx + dx, cy + dy)

    def _ComputeRectangleCorners(self, cx: float, cy: float, length: float, yaw_deg: float, half_width: float):
        ux = math.cos(_DegToRad(yaw_deg))
        uy = math.sin(_DegToRad(yaw_deg))
        nx = -uy
        ny = ux
        hx = 0.5 * length * ux
        hy = 0.5 * length * uy
        wx = half_width * nx
        wy = half_width * ny
        p1 = (cx - hx + wx, cy - hy + wy)
        p2 = (cx + hx + wx, cy + hy + wy)
        p3 = (cx + hx - wx, cy + hy - wy)
        p4 = (cx - hx - wx, cy - hy - wy)
        return [p1, p2, p3, p4]

    def _WhichEndpoint(self, cx: float, cy: float, length: float, yaw_deg: float,
                       x: float, y: float, threshold: float) -> Optional[int]:
        (x1, y1), (x2, y2) = self._ComputeEndpoints(cx, cy, length, yaw_deg)
        d1 = self._DistancePoint(x, y, x1, y1)
        d2 = self._DistancePoint(x, y, x2, y2)
        if d1 <= threshold or d2 <= threshold:
            return 0 if d1 <= d2 else 1
        return None

    def _PickNearestGuardrailIndex(self, x: float, y: float, accept_threshold: float) -> Optional[int]:
        best_i = None
        best_d = float("inf")
        for i, g in enumerate(self.editor_state.guardrails):
            (x1, y1), (x2, y2) = self._ComputeEndpoints(g.cx, g.cy, g.length, g.yaw_deg)
            d = min(
                self._DistancePoint(x, y, g.cx, g.cy),
                self._DistancePoint(x, y, x1, y1),
                self._DistancePoint(x, y, x2, y2),
            )
            if d < best_d:
                best_d = d
                best_i = i
        return best_i if (best_i is not None and best_d <= accept_threshold) else None

    # ---------- undo helpers ----------
    def _Snapshot(self) -> list:
        return [GuardrailItem(cx=g.cx, cy=g.cy, length=g.length, yaw_deg=g.yaw_deg)
                for g in self.editor_state.guardrails]

    def _PushUndoSnapshot(self):
        try:
            snap = self._Snapshot()
            self.undo_stack.append(snap)
            if len(self.undo_stack) > self.max_undo:
                self.undo_stack.pop(0)
        except Exception:
            pass

    def _RestoreFromSnapshot(self, snapshot: list):
        self.editor_state.guardrails.clear()
        for g in snapshot:
            self.editor_state.guardrails.append(
                GuardrailItem(cx=float(g.cx), cy=float(g.cy),
                              length=float(g.length), yaw_deg=float(g.yaw_deg))
            )

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

    # ---------- mode hooks ----------
    def OnModeEntered(self):
        print("[GuardrailsEditor] View/Create/Modify/Delete. Modify: click near center/endpoints to move or scale/rotate. Ctrl+Z to undo.")
        self.Redraw()

    # ---------- events ----------
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
                print("[GuardrailsEditor] Undo.")
            return

    def OnMousePress(self, event):
        submode = getattr(self.editor_state, "guardrails_submode", "view")
        if submode == "view":
            return

        mouse_xy = self.GetMouseXY(event)
        if mouse_xy is None:
            return
        x, y = mouse_xy

        center_thr = self._PixelsToDataDistanceAtMouse(event, self.center_pick_radius_px)
        end_thr = self._PixelsToDataDistanceAtMouse(event, self.endpoint_pick_radius_px)
        end_thr_expand = end_thr * 2.0

        # ----- create -----
        if submode == "create" and event.button == 1:
            self._PushUndoSnapshot()
            length = float(getattr(self.editor_state, "guardrails_create_length", 2.0))
            yaw_deg = float(getattr(self.editor_state, "guardrails_create_yaw_deg", 0.0))
            self.editor_state.guardrails.append(
                GuardrailItem(cx=float(x), cy=float(y), length=length, yaw_deg=yaw_deg)
            )
            self.editor_state.selected_guardrail_index = len(self.editor_state.guardrails) - 1
            print(f"[GuardrailsEditor] Guardrail created. Total={len(self.editor_state.guardrails)}")
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        # ----- delete -----
        if submode == "delete" and event.button == 1:
            sel = self.editor_state.selected_guardrail_index
            if sel is not None and 0 <= sel < len(self.editor_state.guardrails):
                self._PushUndoSnapshot()
                self.editor_state.guardrails.pop(sel)
                self.editor_state.selected_guardrail_index = None
                print("[GuardrailsEditor] Guardrail deleted.")
                self.Redraw()
                if callable(self.on_geometry_changed):
                    self.on_geometry_changed()
            else:
                print("[GuardrailsEditor] Select a guardrail in the right panel first.")
            return

        # ----- modify -----
        if submode == "modify" and event.button == 1:
            # 1) 항상 '가까운 가드레일' 한 번 먼저 픽 → 선택 갱신
            #    (이미 다른게 선택되어 있어도, 더 가까운 걸 찍었으면 그걸로 바꿔줌)
            pick_thr = max(end_thr * 3.5, center_thr * 4.0)  # 픽킹은 관대하게
            picked = self._PickNearestGuardrailIndex(x, y, accept_threshold=pick_thr)
            if picked is None:
                print("[GuardrailsEditor] No guardrail near cursor.")
                return

            if self.editor_state.selected_guardrail_index != picked:
                self.editor_state.selected_guardrail_index = picked
                self.Redraw()  # 하이라이트 즉시 반영

            g = self.editor_state.guardrails[picked]

            # 2) 드래그 시작 판정 (센터 → 끝점 → 선분 근처)
            #    센터는 더 작은 반경으로만 잡힘(끝점 먼저 잡히도록)
            if self._DistancePoint(x, y, g.cx, g.cy) <= center_thr * 0.9:
                self._PushUndoSnapshot()
                self.is_moving_center = True
                return

            which = self._WhichEndpoint(g.cx, g.cy, g.length, g.yaw_deg, x, y, end_thr)
            if which is None:
                which = self._WhichEndpoint(g.cx, g.cy, g.length, g.yaw_deg, x, y, end_thr_expand)

            if which is not None:
                self._PushUndoSnapshot()
                self.active_endpoint_index = which
                return

            # 3) 선분에 가까우면 센터 이동으로 간주(옵션)
            (x1, y1), (x2, y2) = self._ComputeEndpoints(g.cx, g.cy, g.length, g.yaw_deg)
            seg_len = self._DistancePoint(x1, y1, x2, y2)
            if seg_len > 1e-9:
                t = max(0.0, min(1.0, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / (seg_len ** 2)))
                px = x1 + t * (x2 - x1)
                py = y1 + t * (y2 - y1)
                if self._DistancePoint(x, y, px, py) <= end_thr * 1.3:
                    self._PushUndoSnapshot()
                    self.is_moving_center = True
                    return

    def OnMouseMove(self, event):
        submode = getattr(self.editor_state, "guardrails_submode", "view")
        if submode != "modify":
            return
        mouse_xy = self.GetMouseXY(event)
        if mouse_xy is None:
            return
        x, y = mouse_xy
        sel = self.editor_state.selected_guardrail_index
        if sel is None or not (0 <= sel < len(self.editor_state.guardrails)):
            return
        g = self.editor_state.guardrails[sel]

        # 센터 이동
        if self.is_moving_center:
            g.cx = float(x)
            g.cy = float(y)
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

        # 끝점 드래그: 길이/각도 동시 갱신
        if self.active_endpoint_index is not None:
            vx = x - g.cx
            vy = y - g.cy
            new_length = 2.0 * math.hypot(vx, vy)
            if new_length < 1e-9:
                return
            new_yaw_deg = _RadToDeg(math.atan2(vy, vx))
            g.length = float(new_length)
            g.yaw_deg = float(new_yaw_deg)
            self.Redraw()
            if callable(self.on_geometry_changed):
                self.on_geometry_changed()
            return

    def OnMouseRelease(self, event):
        self.is_moving_center = False
        self.active_endpoint_index = None

    # ---------- draw ----------
    def Redraw(self):
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()

        for h in self.editor_state.draw_handles["guardrails"]:
            try:
                h.remove()
            except Exception:
                pass
        self.editor_state.draw_handles["guardrails"].clear()

        sel = self.editor_state.selected_guardrail_index
        is_modify = (getattr(self.editor_state, "guardrails_submode", "view") == "modify")

        for idx, g in enumerate(self.editor_state.guardrails):
            (x1, y1), (x2, y2) = self._ComputeEndpoints(g.cx, g.cy, g.length, g.yaw_deg)
            corners = self._ComputeRectangleCorners(g.cx, g.cy, g.length, g.yaw_deg, self.rect_half_width)

            # 직사각형(외곽)
            from matplotlib.patches import Polygon
            edge_color = "red" if (is_modify and idx == sel) else "black"
            rect = Polygon(corners, closed=True, fill=False, edgecolor=edge_color, linewidth=1.5)
            self.axes.add_patch(rect)
            self.editor_state.draw_handles["guardrails"].append(rect)

            # 중심 라인
            hl, = self.axes.plot([x1, x2], [y1, y2], linewidth=2.5, color="black")
            self.editor_state.draw_handles["guardrails"].append(hl)

            # 마커: 끝점(윤곽 원), 중심(검은 사각형)
            ends = self.axes.scatter([x1, x2], [y1, y2], s=70, marker="o",
                                     facecolors="none", edgecolors=edge_color, linewidths=1.5)
            ctr = self.axes.scatter([g.cx], [g.cy], s=60, marker="s", c="black")
            self.editor_state.draw_handles["guardrails"].extend([ends, ctr])

        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()

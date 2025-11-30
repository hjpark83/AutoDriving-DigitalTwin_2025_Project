# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_signs.py
import math
from editor_base import EditorBase
from editor_state import EditorState, SignItem
from typing import Optional, List
# ---- enum name ↔ code ----
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

def ToSignClassCode(value) -> int:
    """문자열/숫자 어떤 형태든 0~14 정수 코드로 안전 변환."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, str):
        s = value.strip()
        if s.isdigit():
            return int(s)
        s_up = s.upper()
        if s_up in SIGN_NAME_TO_CODE:
            return SIGN_NAME_TO_CODE[s_up]
        if not s_up.startswith("CLASS_"):
            alt = "CLASS_" + s_up
            return SIGN_NAME_TO_CODE.get(alt, 0)
    return 0


class SignsEditor(EditorBase):
    def __init__(self, editor_state: EditorState, figure, axes):
        super().__init__(editor_state, figure, axes)
        # Create Settings 기본값
        self.current_sign_class = 0   # CLASS_UNKNOWN
        self.current_sign_z = 1.0
        self.current_sign_yaw = 90.0

        # 드래그 이동 상태
        self.moving_sign = False

        # 지오메트리 변경 콜백
        self.on_geometry_changed = None

        # 픽킹 반경(픽셀)
        self.pick_radius_px = 24.0

        # Undo 스택
        self.undo_stack: List[list] = []
        self.max_undo: int = 50

    # ---------- 콜백 ----------
    def SetOnGeometryChanged(self, callback):
        self.on_geometry_changed = callback

    # ---------- 서브모드 ----------
    def SetSubmode(self, submode: str):
        allowed = {"view", "create", "move", "delete"}
        if submode in allowed:
            self.editor_state.signs_submode = submode
            print(f"[SignsEditor] Submode -> {submode}")
            self.Redraw()

    def SetCreateDefaults(self, sign_class: int = None, z: float = None, yaw: float = None):
        if sign_class is not None:
            try:
                self.current_sign_class = int(sign_class)
            except Exception:
                pass
        if z is not None:
            self.current_sign_z = float(z)
        if yaw is not None:
            self.current_sign_yaw = float(yaw)
        print(f"[SignsEditor] Create defaults -> class={self.current_sign_class}, z={self.current_sign_z}, yaw={self.current_sign_yaw}")

    def OnModeEntered(self):
        print("[SignsEditor] Modes: view / create / move / delete")
        print("[SignsEditor] Create: left-click to add.")
        print("[SignsEditor] Move: left-click nearest to select and drag; release to finish.")
        print("[SignsEditor] Delete: left-click nearest to delete.")
        self.Redraw()

    # ---- undo helpers
    def _Snapshot(self) -> list:
        return [SignItem(x=sign.x, 
                         y=sign.y, 
                         z=sign.z,
                         yaw=sign.yaw, 
                         sign_class=sign.sign_class)
                for sign in self.editor_state.signs] 

    def _PushUndoSnapshot(self):
        try:
            snap = self._Snapshot()
            self.undo_stack.append(snap)
            if len(self.undo_stack) > self.max_undo:
                self.undo_stack.pop(0)
        except Exception:
            pass

    def _RestoreFromSnapshot(self, snapshot: list):
        self.editor_state.signs.clear()
        for sign in snapshot:
            self.editor_state.signs.append(
                SignItem(x=sign.x, 
                         y=sign.y, 
                         z=sign.z,
                         yaw=sign.yaw, 
                         sign_class=sign.sign_class)
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


    # ---------- 헬퍼 ----------
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

    def _FindNearestSign(self, x: float, y: float, threshold: float):
        best_index = None
        best_dist = float("inf")
        for i, s in enumerate(self.editor_state.signs):
            d = math.hypot(s.x - x, s.y - y)
            if d < best_dist and d <= threshold:
                best_dist = d
                best_index = i
        return best_index

    # ---------- 마우스 ----------
    def OnMousePress(self, event):
        submode = getattr(self.editor_state, "signs_submode", "view")
        if submode == "view":
            return

        mouse_xy = self.GetMouseXY(event)
        if mouse_xy is None:
            return
        x, y = mouse_xy

        thr = self._PixelsToDataDistanceAtMouse(event, self.pick_radius_px)
        thr2 = thr * 2.0  # 확장 허용

        if event.button == 1:
            # CREATE
            if submode == "create":
                self._PushUndoSnapshot()
                self.editor_state.signs.append(SignItem(
                    x=x, y=y,
                    z=self.current_sign_z,
                    yaw=self.current_sign_yaw,
                    sign_class=ToSignClassCode(self.current_sign_class),
                ))
                self.editor_state.selected_sign_index = len(self.editor_state.signs) - 1
                self.Redraw()
                if callable(self.on_geometry_changed):
                    self.on_geometry_changed()
                return

            # MOVE (가까운 객체 자동 선택 + 드래그 시작)
            if submode == "move":
                idx = self._FindNearestSign(x, y, thr) or self._FindNearestSign(x, y, thr2)
                if idx is not None:
                    self._PushUndoSnapshot()
                    self.editor_state.selected_sign_index = idx
                    self.moving_sign = True
                    print(f"[SignsEditor] Move start: sign_index={idx}")
                    # 선택만 반영해도 우측 패널 동기화를 위해 다시 그리기
                    self.Redraw()
                    if callable(self.on_geometry_changed):
                        self.on_geometry_changed()
                else:
                    print(f"[SignsEditor] No sign within {thr:.3f} (expanded {thr2:.3f}).")
                return

            # DELETE (가까운 객체 자동 선택 + 삭제)
            if submode == "delete":
                idx = self._FindNearestSign(x, y, thr) or self._FindNearestSign(x, y, thr2)
                if idx is not None:
                    self._PushUndoSnapshot()
                    self.editor_state.signs.pop(idx)
                    # 선택 인덱스 정리
                    if self.editor_state.selected_sign_index is not None:
                        if idx == self.editor_state.selected_sign_index:
                            self.editor_state.selected_sign_index = None
                        elif idx < self.editor_state.selected_sign_index:
                            self.editor_state.selected_sign_index -= 1
                    print("[SignsEditor] Sign deleted.")
                    self.Redraw()
                    if callable(self.on_geometry_changed):
                        self.on_geometry_changed()
                else:
                    print(f"[SignsEditor] No sign within {thr:.3f} (expanded {thr2:.3f}).")

    def OnMouseMove(self, event):
        if not self.moving_sign:
            return
        mouse_xy = self.GetMouseXY(event)
        if mouse_xy is None:
            return
        x, y = mouse_xy
        idx = self.editor_state.selected_sign_index
        if idx is None or not (0 <= idx < len(self.editor_state.signs)):
            return
        s = self.editor_state.signs[idx]
        s.x = float(x); s.y = float(y)
        self.Redraw()
        if callable(self.on_geometry_changed):
            self.on_geometry_changed()

    def OnMouseRelease(self, event):
        if self.moving_sign:
            self.moving_sign = False
            print("[SignsEditor] Move end.")

    # ---------- 키보드 (옵션) ----------
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
                print("[SignsEditor] Undo.")
            return

    # ---------- 드로잉 ----------
    def Redraw(self):
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()

        # 기존 핸들 제거
        for handle in self.editor_state.draw_handles["signs"]:
            try:
                handle.remove()
            except Exception:
                pass
        self.editor_state.draw_handles["signs"].clear()

        items = self.editor_state.signs
        if items:
            # 픽셀 → 데이터 길이 변환(크기 느낌 일정)
            triangle_side_px = 34.0
            anchor_offset_px = 30.0
            pad_px = 2.0

            inv = self.axes.transData.inverted()

            # 수평 변환
            x0, _ = inv.transform((0.0, 0.0))
            xw, _ = inv.transform((triangle_side_px, 0.0))
            xp, _ = inv.transform((pad_px, 0.0))
            triangle_side_data = abs(xw - x0)
            pad_data_x = abs(xp - x0)

            # 수직 변환
            _, y0 = inv.transform((0.0, 0.0))
            _, yo = inv.transform((0.0, anchor_offset_px))
            anchor_offset_data = abs(yo - y0)

            triangle_height_data = (math.sqrt(3.0) / 2.0) * triangle_side_data

            mode = getattr(self.editor_state, "signs_submode", "view")
            selected_index = self.editor_state.selected_sign_index

            from matplotlib import patches as mpatches

            for index, s in enumerate(items):
                center_x = float(s.x)
                center_y = float(s.y)
                sign_code = ToSignClassCode(getattr(s, "sign_class", 0))

                is_selected_in_move = (mode == "move" and
                                       selected_index is not None and
                                       index == selected_index)

                # 삼각형 배치 계산
                tri_center_y = center_y + anchor_offset_data
                base_y = tri_center_y - (triangle_height_data / 3.0)
                apex_y = base_y + triangle_height_data
                half_side = triangle_side_data * 0.5
                left_x = center_x - half_side
                right_x = center_x + half_side
                vertices = [(left_x, base_y), (right_x, base_y), (center_x, apex_y)]

                # 삼각형 패치(선택 시 붉은 테두리)
                triangle = mpatches.Polygon(
                    vertices,
                    closed=True,
                    facecolor="#ffffff",
                    edgecolor=("red" if is_selected_in_move else "black"),
                    linewidth=(2.0 if is_selected_in_move else 1.2),
                    alpha=0.9,
                    zorder=2.0,
                )
                self.axes.add_patch(triangle)
                self.editor_state.draw_handles["signs"].append(triangle)

                # 기준점 X
                x_marker = self.axes.scatter(
                    [center_x], [center_y],
                    marker="x", s=120, linewidths=2.0,
                    c="black", zorder=3.0
                )
                self.editor_state.draw_handles["signs"].append(x_marker)

                # 기준점 → 삼각형 밑변 중심 점선
                dashed_line, = self.axes.plot(
                    [center_x, center_x], [center_y, base_y],
                    linestyle=":", linewidth=1.4, color="black", zorder=2.2
                )
                self.editor_state.draw_handles["signs"].append(dashed_line)

                # 삼각형 내부 숫자( sign_class )
                text_obj = self.axes.text(
                    center_x, tri_center_y, f"{sign_code}",
                    ha="center", va="center",
                    fontsize=9, color="black", zorder=3.0,
                    bbox=dict(boxstyle="round,pad=0.2", fc="none", ec="none")
                )
                self.editor_state.draw_handles["signs"].append(text_obj)

        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)
        self.figure.canvas.draw_idle()

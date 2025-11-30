from dataclasses import asdict
from typing import Optional, Dict, Any, List, Tuple, Union
import math

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons, TextBox
from matplotlib.backend_bases import MouseEvent, KeyEvent

from .structures import MapData
from .io_utils import SaveMapBundle, LoadMapBundle
from .plotting import PlotMap


# ----------------------------
# 유틸
# ----------------------------
def Distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def Clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# 파일 상단 유틸/클래스 근처에 추가
class EditorUI:
    """Matplotlib 위젯 객체를 보관해서 GC로 이벤트가 끊기지 않도록 한다."""
    def __init__(self):
        self.mode_radio = None
        self.save_button = None
        self.cancel_button = None
        self.property_textboxes = {}   # 필요 시: {"x": TextBox, ...}

# ----------------------------
# 에디터 상태
# ----------------------------
class EditorState:
    def __init__(self, map_data: MapData, bundle_path: str):
        self.map_data = map_data
        self.bundle_path = bundle_path
        self.ui = EditorUI()  # ← 추가
        # 뷰 상태
        self.view_range_x = 50.0
        self.view_range_y = 35.0

        # 현재 화면 범위 저장(사용자 줌/팬을 반영)
        self.view_x_limits = (-self.view_range_x, self.view_range_x)
        self.view_y_limits = (-self.view_range_y, self.view_range_y)

        # 모드
        self.current_mode = "pointer"       # pointer / lanes / buildings / guardrails / signs / traffic_lights

        # lanes 그리기 버퍼
        self.current_lane_id = self._SuggestNextLaneId()
        self.current_lane_type = "driving"
        self.current_lane_points: List[Dict[str, Any]] = []  # 진행 중인 차선 점들

        # 선택 상태
        # kind: "lane_point" | "building" | "guardrail" | "sign" | "traffic_light" | None
        # lane_point: (lane_id, point_index)
        self.selected_kind: Optional[str] = None
        self.selected_reference: Optional[Union[Tuple[int, int], int]] = None  # lane:(lane_id, idx) / others:int index

        # 드래그 상태
        self.drag_active = False
        self.drag_role: Optional[str] = None   # "move", "guardrail_end_handle", "building_height", "rotate_guardrail"
        self.drag_start_screen: Optional[Tuple[float, float]] = None
        self.drag_start_value: Optional[Any] = None

        # guardrail 기본값
        self.guardrail_length_default = 10.0
        self.guardrail_yaw_deg_default = 0.0

        # sign / traffic light 기본값
        self.current_sign_class = "unknown"
        self.current_traffic_light_state = "red"

    def _SuggestNextLaneId(self) -> int:
        if not self.map_data.lanes:
            return 0
        return max([int(r["lane_id"]) for r in self.map_data.lanes]) + 1

    # 선택 해제
    def ClearSelection(self) -> None:
        self.selected_kind = None
        self.selected_reference = None

def StartViewerOrEditor(bundle_path: str, block: bool = True) -> None:
    """
    뷰어 창을 열고, 'Edit' 버튼으로 StartEditor를 띄우는 래퍼.
    런처를 닫지 않으려면 block=False로 호출.
    """
    map_data = LoadMapBundle(bundle_path)

    figure, axes = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.12)

    # 0,0이 중앙이 되도록 초기 뷰 설정
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlim(-50.0, 50.0)
    axes.set_ylim(-35.0, 35.0)
    axes.axhline(0.0, color="0.6", linewidth=1, alpha=0.6)
    axes.axvline(0.0, color="0.6", linewidth=1, alpha=0.6)

    PlotMap(axes, asdict(map_data))
    axes.set_title("Map Viewer —  Click 'Edit' to enter editing mode")

    edit_button_axes = figure.add_axes([0.12, 0.02, 0.15, 0.06])
    edit_button = Button(edit_button_axes, "Edit")

    save_button_axes = figure.add_axes([0.30, 0.02, 0.15, 0.06])
    save_button = Button(save_button_axes, "Save")

    def OnEditClicked(event) -> None:
        # 뷰어는 그대로 두고, 에디터를 별도 창으로 띄운다
        StartEditor(bundle_path)

    def OnSaveClicked(event) -> None:
        SaveMapBundle(map_data, bundle_path)
        print(f"[OK] Saved current bundle at: {bundle_path}")

    edit_button.on_clicked(OnEditClicked)
    save_button.on_clicked(OnSaveClicked)

    plt.show(block=block)


def SafeFloatSet(row: Dict[str, Any], key: str, value: str, redraw_callback, minimum: Optional[float] = None) -> None:
    try:
        number_value = float(value)
        if minimum is not None:
            number_value = max(minimum, number_value)
        row[key] = number_value
        redraw_callback()
    except ValueError:
        pass

def SafeFloatSet(row: Dict[str, Any], key: str, value: str, redraw_callback) -> None:
    row[key] = str(value)
    redraw_callback()


# ----------------------------
# 화면/패널 빌드
# ----------------------------
def StartEditor(bundle_path: str, block: bool = True) -> None:
    """
    JOSM 스타일 편집기:
      - 좌측: 맵 캔버스
      - 우측: 도구/속성 패널
    """
    map_data = LoadMapBundle(bundle_path)
    state = EditorState(map_data, bundle_path)

    # Figure 레이아웃
    figure = plt.figure(figsize=(16, 9))
    # 좌측 캔버스
    
    # 우측 패널
    axes_panel = figure.add_axes([0.03, 0.08, 0.23, 0.88])
    axes_panel.axis("off")
    axes_map   = figure.add_axes([0.30, 0.08, 0.68, 0.88])

    # 초기 뷰: 0,0이 중앙이 되도록 고정
    axes_map.set_aspect("equal", adjustable="box")
    axes_map.set_xlim(state.view_x_limits)
    axes_map.set_ylim(state.view_y_limits)
    # axes_map.set_xlim(-state.view_range_x, state.view_range_x)
    # axes_map.set_ylim(-state.view_range_y, state.view_range_y)
    axes_map.autoscale(enable=False)  # 자동 스케일 비활성화(줌/팬은 가능)

    # 원점 십자선
    axes_map.axhline(0.0, color="0.6", linewidth=1, alpha=0.6)
    axes_map.axvline(0.0, color="0.6", linewidth=1, alpha=0.6)

    # 최초 그리기
    def Redraw() -> None:
        # 1) 현재 축 범위를 저장(사용자 줌/팬을 반영)
        state.view_x_limits = axes_map.get_xlim()
        state.view_y_limits = axes_map.get_ylim()
        PlotMap(axes_map, asdict(state.map_data))
        # 진행 중인 차선 그리기 (첫 점은 자동 (0,0))
        if state.current_mode == "lanes" and state.current_lane_points:
            xs = [p["x"] for p in state.current_lane_points]
            ys = [p["y"] for p in state.current_lane_points]
            axes_map.plot(xs, ys, marker="o", linewidth=2, markersize=4, alpha=0.9, label=f"lane new {state.current_lane_id}")
        # 선택 강조
        DrawSelectionOverlay(axes_map, state)
        # 원점 십자선 유지
        axes_map.axhline(0.0, color="0.6", linewidth=1, alpha=0.6)
        axes_map.axvline(0.0, color="0.6", linewidth=1, alpha=0.6)
        axes_map.set_xlim(state.view_x_limits)
        axes_map.set_ylim(state.view_y_limits)
        axes_map.autoscale(enable=False)

        axes_map.figure.canvas.draw_idle()

        # 패널 갱신
        # BuildRightPanel(figure, axes_panel, state, Redraw)
        BuildToolPanel(figure, axes_panel, state, Redraw)

    def DrawSelectionOverlay(axes, editor_state: EditorState) -> None:
        if editor_state.selected_kind is None:
            return

        if editor_state.selected_kind == "lane_point":
            lane_id, point_index = editor_state.selected_reference  # type: ignore
            lane_points = [r for r in editor_state.map_data.lanes if int(r["lane_id"]) == int(lane_id)]
            if 0 <= point_index < len(lane_points):
                px = lane_points[point_index]["x"]
                py = lane_points[point_index]["y"]
                axes.scatter([px], [py], s=100, facecolors="none", edgecolors="red", linewidths=2)

        elif editor_state.selected_kind == "building":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.buildings):
                bx = editor_state.map_data.buildings[i]["x"]
                by = editor_state.map_data.buildings[i]["y"]
                axes.scatter([bx], [by], s=160, facecolors="none", edgecolors="red", linewidths=2)

        elif editor_state.selected_kind == "guardrail":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.guardrails):
                cx = editor_state.map_data.guardrails[i]["cx"]
                cy = editor_state.map_data.guardrails[i]["cy"]
                length = editor_state.map_data.guardrails[i]["length"]
                yaw_deg = editor_state.map_data.guardrails[i]["yaw_deg"]
                axes.scatter([cx], [cy], s=160, facecolors="none", edgecolors="red", linewidths=2)
                # 끝점 핸들
                yaw_rad = math.radians(yaw_deg)
                ex = cx + (length / 2.0) * math.cos(yaw_rad)
                ey = cy + (length / 2.0) * math.sin(yaw_rad)
                axes.scatter([ex], [ey], s=60, c="red")

        elif editor_state.selected_kind == "sign":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.signs):
                sx = editor_state.map_data.signs[i]["x"]
                sy = editor_state.map_data.signs[i]["y"]
                axes.scatter([sx], [sy], s=160, facecolors="none", edgecolors="red", linewidths=2)

        elif editor_state.selected_kind == "traffic_light":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.traffic_lights):
                tx = editor_state.map_data.traffic_lights[i]["x"]
                ty = editor_state.map_data.traffic_lights[i]["y"]
                axes.scatter([tx], [ty], s=160, facecolors="none", edgecolors="red", linewidths=2)


    # 패널에 들어갈 서브 위젯 축들을 재구성하는 헬퍼
    def BuildToolPanel(figure, panel_axes, editor_state: EditorState, redraw_callback) -> None:
        """왼쪽 툴 패널을 다시 그리며, 위젯 객체는 editor_state.ui에 보관한다."""
        panel_axes.clear()
        panel_axes.axis("off")

        # 1) 모드 선택 라디오 (왼쪽 상단)
        radio_axes = figure.add_axes([0.04, 0.84, 0.20, 0.10])
        mode_labels = ["pointer", "lanes", "buildings", "guardrails", "signs", "traffic_lights"]
        editor_state.ui.mode_radio = RadioButtons(
            radio_axes,
            mode_labels,
            active=mode_labels.index(editor_state.current_mode)
        )

        def OnModeChanged(label: str) -> None:
            editor_state.current_mode = label
            if label == "lanes" and not editor_state.current_lane_points:
                editor_state.current_lane_points = [{
                    "lane_id": editor_state.current_lane_id,
                    "x": 0.0, "y": 0.0, "z": 0.0, "lane_type": editor_state.current_lane_type
                }]
            redraw_callback()

        editor_state.ui.mode_radio.on_clicked(OnModeChanged)

        # 2) Save / Cancel
        editor_state.ui.save_button = Button(figure.add_axes([0.04, 0.78, 0.09, 0.05]), "Save")
        editor_state.ui.cancel_button = Button(figure.add_axes([0.15, 0.78, 0.09, 0.05]), "Cancel")

        def OnSaveClicked(event) -> None:
            SaveMapBundle(editor_state.map_data, editor_state.bundle_path)
            print(f"[OK] Saved: {editor_state.bundle_path}")

        def OnCancelSelection(event) -> None:
            editor_state.ClearSelection()
            redraw_callback()

        editor_state.ui.save_button.on_clicked(OnSaveClicked)
        editor_state.ui.cancel_button.on_clicked(OnCancelSelection)

        # 3) 진행 중 레인 컨트롤 (필요 시)
        y_cursor = 0.72

        def AddButton(label: str, callback, x: float = 0.04, w: float = 0.20) -> Button:
            nonlocal y_cursor
            button_axes = figure.add_axes([x, y_cursor, w, 0.05])
            button = Button(button_axes, label)
            button.on_clicked(callback)
            y_cursor -= 0.06
            return button

        if editor_state.current_mode == "lanes":
            AddButton("Finish Lane (Enter)", lambda e: FinishLane(editor_state, redraw_callback))
            AddButton("Undo Last Point (U)", lambda e: UndoLanePoint(editor_state, redraw_callback))

        # 4) 선택 객체 속성 편집(TextBox들)
        #    TextBox도 참조를 보관해야 클릭/입력 이벤트가 살아있다.
        editor_state.ui.property_textboxes = {}
        y_prop = 0.62

        def AddLabeledTextBox(label: str, initial: str, on_submit, width: float = 0.12):
            nonlocal y_prop
            label_axes = figure.add_axes([0.04, y_prop, 0.08, 0.045]); label_axes.axis("off")
            label_axes.text(0.0, 0.5, label, va="center", fontsize=9)
            textbox_axes = figure.add_axes([0.13, y_prop, width, 0.045])
            tb = TextBox(textbox_axes, "", initial=initial)
            tb.on_submit(lambda v: on_submit(v))
            editor_state.ui.property_textboxes[label] = tb
            y_prop -= 0.06

        # 선택된 엔티티별 폼
        if editor_state.selected_kind == "lane_point" and isinstance(editor_state.selected_reference, tuple):
            lane_id, point_index = editor_state.selected_reference
            lane_points = [r for r in editor_state.map_data.lanes if int(r["lane_id"]) == int(lane_id)]
            if 0 <= point_index < len(lane_points):
                row = lane_points[point_index]
                AddLabeledTextBox("lane_id", str(row["lane_id"]), lambda v: None)
                AddLabeledTextBox("x", f'{row["x"]}', lambda v: SafeFloatSet(row, "x", v, redraw_callback))
                AddLabeledTextBox("y", f'{row["y"]}', lambda v: SafeFloatSet(row, "y", v, redraw_callback))
                AddLabeledTextBox("z", f'{row["z"]}', lambda v: SafeFloatSet(row, "z", v, redraw_callback))
                AddLabeledTextBox("lane_type", str(row["lane_type"]), lambda v: SafeFloatSet(row, "lane_type", v, redraw_callback))

        elif editor_state.selected_kind == "building" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference; row = editor_state.map_data.buildings[i]
            AddLabeledTextBox("id", str(row["id"]), lambda v: None)
            AddLabeledTextBox("x", f'{row["x"]}', lambda v: SafeFloatSet(row, "x", v, redraw_callback))
            AddLabeledTextBox("y", f'{row["y"]}', lambda v: SafeFloatSet(row, "y", v, redraw_callback))
            AddLabeledTextBox("h", f'{row["h"]}', lambda v: SafeFloatSet(row, "h", v, redraw_callback))

        elif editor_state.selected_kind == "guardrail" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference; row = editor_state.map_data.guardrails[i]
            AddLabeledTextBox("cx", f'{row["cx"]}', lambda v: SafeFloatSet(row, "cx", v, redraw_callback))
            AddLabeledTextBox("cy", f'{row["cy"]}', lambda v: SafeFloatSet(row, "cy", v, redraw_callback))
            AddLabeledTextBox("length", f'{row["length"]}', lambda v: SafeFloatSet(row, "length", v, redraw_callback, minimum=0.0))
            AddLabeledTextBox("yaw_deg", f'{row["yaw_deg"]}', lambda v: SafeFloatSet(row, "yaw_deg", v, redraw_callback))

        elif editor_state.selected_kind == "sign" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference; row = editor_state.map_data.signs[i]
            AddLabeledTextBox("x", f'{row["x"]}', lambda v: SafeFloatSet(row, "x", v, redraw_callback))
            AddLabeledTextBox("y", f'{row["y"]}', lambda v: SafeFloatSet(row, "y", v, redraw_callback))
            AddLabeledTextBox("z", f'{row["z"]}', lambda v: SafeFloatSet(row, "z", v, redraw_callback))
            AddLabeledTextBox("yaw", f'{row["yaw"]}', lambda v: SafeFloatSet(row, "yaw", v, redraw_callback))
            AddLabeledTextBox("sign_class", str(row["sign_class"]), lambda v: SafeFloatSet(row, "sign_class", v, redraw_callback))

        elif editor_state.selected_kind == "traffic_light" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference; row = editor_state.map_data.traffic_lights[i]
            AddLabeledTextBox("x", f'{row["x"]}', lambda v: SafeFloatSet(row, "x", v, redraw_callback))
            AddLabeledTextBox("y", f'{row["y"]}', lambda v: SafeFloatSet(row, "y", v, redraw_callback))
            AddLabeledTextBox("z", f'{row["z"]}', lambda v: SafeFloatSet(row, "z", v, redraw_callback))
            AddLabeledTextBox("state", str(row["state"]), lambda v: SafeFloatSet(row, "state", v, redraw_callback))



    def BuildRightPanel(figure, panel_axes, editor_state: EditorState, redraw_callback) -> None:
        panel_axes.clear()
        panel_axes.axis("off")

        # 1) 모드 선택 라디오
        radio_axes = figure.add_axes([0.71, 0.84, 0.23, 0.10])
        mode_labels = ["pointer", "lanes", "buildings", "guardrails", "signs", "traffic_lights"]
        radio_buttons = RadioButtons(radio_axes, mode_labels, active=mode_labels.index(editor_state.current_mode))
        def OnModeChanged(label: str) -> None:
            editor_state.current_mode = label
            # lanes 모드 진입 시, 첫 점을 자동으로 (0,0)에 추가(진행 중 버퍼가 비어있을 때만)
            if label == "lanes" and not editor_state.current_lane_points:
                editor_state.current_lane_points = [{
                    "lane_id": editor_state.current_lane_id, "x": 0.0, "y": 0.0, "z": 0.0, "lane_type": editor_state.current_lane_type
                }]
            redraw_callback()
        radio_buttons.on_clicked(OnModeChanged)

        # 2) 공통 버튼들
        save_button_axes = figure.add_axes([0.71, 0.78, 0.11, 0.05])
        save_button = Button(save_button_axes, "Save")

        cancel_select_button_axes = figure.add_axes([0.83, 0.78, 0.11, 0.05])
        cancel_select_button = Button(cancel_select_button_axes, "Cancel")

        def OnSaveClicked(event) -> None:
            SaveMapBundle(editor_state.map_data, editor_state.bundle_path)
            print(f"[OK] Saved: {editor_state.bundle_path}")

        def OnCancelSelection(event) -> None:
            editor_state.ClearSelection()
            redraw_callback()

        save_button.on_clicked(OnSaveClicked)
        cancel_select_button.on_clicked(OnCancelSelection)

        # 3) 선택된 객체 속성 편집기
        #    선택 종류에 따라 다른 폼을 그림
        y_cursor = 0.74

        def AddTextBox(label: str, initial: str, on_submit, width: float = 0.16) -> TextBox:
            nonlocal y_cursor
            label_axes = figure.add_axes([0.71, y_cursor, 0.10, 0.045]); label_axes.axis("off")
            label_axes.text(0.0, 0.5, label, va="center", fontsize=9)
            textbox_axes = figure.add_axes([0.82, y_cursor, width, 0.045])
            textbox = TextBox(textbox_axes, "", initial=initial)
            textbox.on_submit(lambda v: on_submit(v))
            y_cursor -= 0.06
            return textbox

        def AddButton(label: str, callback, x: float = 0.71, w: float = 0.23) -> Button:
            nonlocal y_cursor
            button_axes = figure.add_axes([x, y_cursor, w, 0.05])
            button = Button(button_axes, label)
            button.on_clicked(callback)
            y_cursor -= 0.06
            return button

        # lanes 진행 중일 때 보조 버튼
        if editor_state.current_mode == "lanes":
            AddButton("Finish Lane (Enter)", lambda e: FinishLane(editor_state, redraw_callback))
            AddButton("Undo Last Point (U)", lambda e: UndoLanePoint(editor_state, redraw_callback))

        # 선택된 객체에 따른 속성 창
        if editor_state.selected_kind == "lane_point" and isinstance(editor_state.selected_reference, tuple):
            lane_id, point_index = editor_state.selected_reference
            lane_points = [r for r in editor_state.map_data.lanes if int(r["lane_id"]) == int(lane_id)]
            if 0 <= point_index < len(lane_points):
                point_row = lane_points[point_index]

                def SubmitX(value: str):
                    try:
                        point_row["x"] = float(value); redraw_callback()
                    except ValueError: pass
                def SubmitY(value: str):
                    try:
                        point_row["y"] = float(value); redraw_callback()
                    except ValueError: pass
                def SubmitZ(value: str):
                    try:
                        point_row["z"] = float(value); redraw_callback()
                    except ValueError: pass
                def SubmitType(value: str):
                    point_row["lane_type"] = str(value); redraw_callback()

                AddTextBox("lane_id", str(point_row["lane_id"]), lambda v: None)
                AddTextBox("x", f'{point_row["x"]}', SubmitX)
                AddTextBox("y", f'{point_row["y"]}', SubmitY)
                AddTextBox("z", f'{point_row["z"]}', SubmitZ)
                AddTextBox("lane_type", str(point_row["lane_type"]), SubmitType)

        elif editor_state.selected_kind == "building" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference
            row = editor_state.map_data.buildings[i]

            def SubmitX(value: str):
                try: row["x"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitY(value: str):
                try: row["y"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitH(value: str):
                try: row["h"] = float(value); redraw_callback()
                except ValueError: pass

            AddTextBox("id", str(row["id"]), lambda v: None)
            AddTextBox("x", f'{row["x"]}', SubmitX)
            AddTextBox("y", f'{row["y"]}', SubmitY)
            AddTextBox("h", f'{row["h"]}', SubmitH)

        elif editor_state.selected_kind == "guardrail" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference
            row = editor_state.map_data.guardrails[i]

            def SubmitCx(value: str):
                try: row["cx"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitCy(value: str):
                try: row["cy"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitLen(value: str):
                try: row["length"] = max(0.0, float(value)); redraw_callback()
                except ValueError: pass
            def SubmitYaw(value: str):
                try: row["yaw_deg"] = float(value); redraw_callback()
                except ValueError: pass

            AddTextBox("cx", f'{row["cx"]}', SubmitCx)
            AddTextBox("cy", f'{row["cy"]}', SubmitCy)
            AddTextBox("length", f'{row["length"]}', SubmitLen)
            AddTextBox("yaw_deg", f'{row["yaw_deg"]}', SubmitYaw)

        elif editor_state.selected_kind == "sign" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference
            row = editor_state.map_data.signs[i]

            def SubmitX(value: str):
                try: row["x"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitY(value: str):
                try: row["y"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitZ(value: str):
                try: row["z"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitYaw(value: str):
                try: row["yaw"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitClass(value: str):
                row["sign_class"] = str(value); redraw_callback()

            AddTextBox("x", f'{row["x"]}', SubmitX)
            AddTextBox("y", f'{row["y"]}', SubmitY)
            AddTextBox("z", f'{row["z"]}', SubmitZ)
            AddTextBox("yaw", f'{row["yaw"]}', SubmitYaw)
            AddTextBox("sign_class", str(row["sign_class"]), SubmitClass)

        elif editor_state.selected_kind == "traffic_light" and isinstance(editor_state.selected_reference, int):
            i = editor_state.selected_reference
            row = editor_state.map_data.traffic_lights[i]

            def SubmitX(value: str):
                try: row["x"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitY(value: str):
                try: row["y"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitZ(value: str):
                try: row["z"] = float(value); redraw_callback()
                except ValueError: pass
            def SubmitState(value: str):
                row["state"] = str(value); redraw_callback()

            AddTextBox("x", f'{row["x"]}', SubmitX)
            AddTextBox("y", f'{row["y"]}', SubmitY)
            AddTextBox("z", f'{row["z"]}', SubmitZ)
            AddTextBox("state", str(row["state"]), SubmitState)

        # 삭제 버튼(선택 시)
        if editor_state.selected_kind is not None:
            AddButton("Delete (Del)", lambda e: DeleteSelected(editor_state, redraw_callback))

    # ----------------------------
    # 동작 도우미
    # ----------------------------
    def HitTestOnCanvas(x: float, y: float) -> Tuple[Optional[str], Optional[Union[Tuple[int, int], int]]]:
        """캔버스 좌표에서 가장 가까운 엔티티를 찾는다."""
        threshold = 0.8  # 선택 허용 반경

        # lanes: 각 점
        # lane_id별 순서가 보장된다고 가정 (입력 순서)
        # 모든 점 중 최단 거리 점 탐색
        best_kind = None
        best_ref = None
        best_dist = float("inf")
        # lanes
        # 그룹핑
        by_lane: Dict[int, List[Dict[str, Any]]] = {}
        for r in state.map_data.lanes:
            by_lane.setdefault(int(r["lane_id"]), []).append(r)
        for lane_id, points in by_lane.items():
            for idx, pt in enumerate(points):
                d = Distance((x, y), (pt["x"], pt["y"]))
                if d < best_dist and d <= threshold:
                    best_dist = d
                    best_kind = "lane_point"
                    best_ref = (lane_id, idx)

        # buildings
        for i, row in enumerate(state.map_data.buildings):
            d = Distance((x, y), (row["x"], row["y"]))
            if d < best_dist and d <= threshold:
                best_dist = d
                best_kind = "building"
                best_ref = i

        # guardrails (중심점 기준 + 끝점 핸들 별도 체크는 드래그 시작에서 처리)
        for i, row in enumerate(state.map_data.guardrails):
            d = Distance((x, y), (row["cx"], row["cy"]))
            if d < best_dist and d <= threshold:
                best_dist = d
                best_kind = "guardrail"
                best_ref = i

        # signs
        for i, row in enumerate(state.map_data.signs):
            d = Distance((x, y), (row["x"], row["y"]))
            if d < best_dist and d <= threshold:
                best_dist = d
                best_kind = "sign"
                best_ref = i

        # traffic lights
        for i, row in enumerate(state.map_data.traffic_lights):
            d = Distance((x, y), (row["x"], row["y"]))
            if d < best_dist and d <= threshold:
                best_dist = d
                best_kind = "traffic_light"
                best_ref = i

        return best_kind, best_ref

    def FinishLane(editor_state: EditorState, redraw_callback) -> None:
        if len(editor_state.current_lane_points) >= 2:
            # 진행 중 버퍼를 실제 lanes에 append
            for p in editor_state.current_lane_points:
                editor_state.map_data.lanes.append({
                    "lane_id": editor_state.current_lane_id,
                    "x": p["x"], "y": p["y"], "z": p["z"], "lane_type": editor_state.current_lane_type
                })
            print(f"[OK] Lane saved: lane_id={editor_state.current_lane_id} points={len(editor_state.current_lane_points)}")
            editor_state.current_lane_id += 1
            editor_state.current_lane_points = []
            editor_state.ClearSelection()
            redraw_callback()
        else:
            print("[INFO] Lane not finished (need >= 2 points).")

    def UndoLanePoint(editor_state: EditorState, redraw_callback) -> None:
        if editor_state.current_lane_points:
            editor_state.current_lane_points.pop()
            redraw_callback()

    def DeleteSelected(editor_state: EditorState, redraw_callback) -> None:
        if editor_state.selected_kind is None:
            return
        if editor_state.selected_kind == "lane_point":
            lane_id, point_index = editor_state.selected_reference  # type: ignore
            # lane_id에 해당하는 모든 점들
            lane_points = [r for r in editor_state.map_data.lanes if int(r["lane_id"]) == int(lane_id)]
            if 0 <= point_index < len(lane_points):
                # 실제 데이터에서 이 점만 제거
                target_row = lane_points[point_index]
                # 동일 참조 제거
                editor_state.map_data.lanes.remove(target_row)
        elif editor_state.selected_kind == "building":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.buildings):
                editor_state.map_data.buildings.pop(i)
        elif editor_state.selected_kind == "guardrail":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.guardrails):
                editor_state.map_data.guardrails.pop(i)
        elif editor_state.selected_kind == "sign":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.signs):
                editor_state.map_data.signs.pop(i)
        elif editor_state.selected_kind == "traffic_light":
            i = editor_state.selected_reference  # type: ignore
            if 0 <= i < len(editor_state.map_data.traffic_lights):
                editor_state.map_data.traffic_lights.pop(i)
        editor_state.ClearSelection()
        redraw_callback()

    # ----------------------------
    # 이벤트 핸들러
    # ----------------------------
    def OnKeyPress(event: KeyEvent) -> None:
        if event.key in ("q", "Q"):
            plt.close(figure)
        elif event.key in ("s", "S"):
            SaveMapBundle(state.map_data, state.bundle_path)
            print(f"[OK] Saved: {state.bundle_path}")
        elif event.key == "enter":
            if state.current_mode == "lanes":
                FinishLane(state, Redraw)
        elif event.key in ("u", "U"):
            if state.current_mode == "lanes":
                UndoLanePoint(state, Redraw)
        elif event.key == "escape":
            state.ClearSelection(); Redraw()
        elif event.key == "delete":
            DeleteSelected(state, Redraw)

    def OnMousePress(event: MouseEvent) -> None:
        if event.inaxes != axes_map or event.button != 1:
            return
        click_x = float(event.xdata)
        click_y = float(event.ydata)

        if state.current_mode == "pointer":
            kind, ref = HitTestOnCanvas(click_x, click_y)
            state.selected_kind = kind
            state.selected_reference = ref
            # 드래그 모드 결정
            if kind is not None:
                state.drag_active = True
                state.drag_role = "move"
                state.drag_start_screen = (click_x, click_y)
                state.drag_start_value = ref
            Redraw()
            return

        if state.current_mode == "lanes":
            # 첫 점은 자동 (0,0)
            if not state.current_lane_points:
                state.current_lane_points.append({
                    "lane_id": state.current_lane_id, "x": 0.0, "y": 0.0, "z": 0.0, "lane_type": state.current_lane_type
                })
            state.current_lane_points.append({
                "lane_id": state.current_lane_id, "x": click_x, "y": click_y, "z": 0.0, "lane_type": state.current_lane_type
            })
            Redraw()
            return

        if state.current_mode == "buildings":
            next_id = (max([b["id"] for b in state.map_data.buildings], default=-1) + 1) if state.map_data.buildings else 0
            state.map_data.buildings.append({"id": next_id, "x": click_x, "y": click_y, "h": 10.0})
            state.selected_kind = "building"
            state.selected_reference = len(state.map_data.buildings) - 1
            state.drag_active = True
            state.drag_role = "move"
            state.drag_start_screen = (click_x, click_y)
            state.drag_start_value = state.selected_reference
            Redraw()
            return

        if state.current_mode == "guardrails":
            state.map_data.guardrails.append({
                "cx": click_x, "cy": click_y,
                "length": state.guardrail_length_default, "yaw_deg": state.guardrail_yaw_deg_default
            })
            state.selected_kind = "guardrail"
            state.selected_reference = len(state.map_data.guardrails) - 1
            state.drag_active = True
            state.drag_role = "move"
            state.drag_start_screen = (click_x, click_y)
            state.drag_start_value = state.selected_reference
            Redraw()
            return

        if state.current_mode == "signs":
            state.map_data.signs.append({
                "x": click_x, "y": click_y, "z": 0.0, "yaw": 0.0, "sign_class": state.current_sign_class
            })
            state.selected_kind = "sign"
            state.selected_reference = len(state.map_data.signs) - 1
            state.drag_active = True
            state.drag_role = "move"
            state.drag_start_screen = (click_x, click_y)
            state.drag_start_value = state.selected_reference
            Redraw()
            return

        if state.current_mode == "traffic_lights":
            state.map_data.traffic_lights.append({
                "x": click_x, "y": click_y, "z": 0.0, "state": state.current_traffic_light_state
            })
            state.selected_kind = "traffic_light"
            state.selected_reference = len(state.map_data.traffic_lights) - 1
            state.drag_active = True
            state.drag_role = "move"
            state.drag_start_screen = (click_x, click_y)
            state.drag_start_value = state.selected_reference
            Redraw()
            return

    def OnMouseRelease(event: MouseEvent) -> None:
        if event.inaxes != axes_map or event.button != 1:
            return
        state.drag_active = False
        state.drag_role = None
        state.drag_start_screen = None
        state.drag_start_value = None

    def OnMouseMove(event: MouseEvent) -> None:
        if not state.drag_active or event.inaxes != axes_map:
            return
        move_x = float(event.xdata)
        move_y = float(event.ydata)

        # lanes: 점 드래그는 pointer 모드에서만 허용
        if state.drag_role == "move" and state.selected_kind == "lane_point" and isinstance(state.selected_reference, tuple):
            lane_id, point_index = state.selected_reference
            lane_points = [r for r in state.map_data.lanes if int(r["lane_id"]) == int(lane_id)]
            if 0 <= point_index < len(lane_points):
                lane_points[point_index]["x"] = move_x
                lane_points[point_index]["y"] = move_y
                Redraw()
                return

        # buildings move 또는 높이 변경(Shift)
        if state.selected_kind == "building" and isinstance(state.selected_reference, int):
            i = state.selected_reference
            row = state.map_data.buildings[i]
            if event.key == "shift":  # 높이 h 조절
                delta_y = move_y - state.drag_start_screen[1] if state.drag_start_screen else 0.0
                row["h"] = max(0.0, row["h"] + delta_y)
            else:  # 위치 이동
                row["x"] = move_x
                row["y"] = move_y
            Redraw()
            return

        # guardrail 이동/길이/회전
        if state.selected_kind == "guardrail" and isinstance(state.selected_reference, int):
            i = state.selected_reference
            row = state.map_data.guardrails[i]
            if event.key == "alt":  # 회전
                dx = move_x - row["cx"]; dy = move_y - row["cy"]
                row["yaw_deg"] = math.degrees(math.atan2(dy, dx))
            else:  # 위치 이동 또는 끝점 길이 조절
                if state.drag_role == "guardrail_end_handle":
                    yaw_rad = math.radians(row["yaw_deg"])
                    # 마우스 위치를 투영하여 length 업데이트
                    vx = math.cos(yaw_rad); vy = math.sin(yaw_rad)
                    proj = (move_x - row["cx"]) * vx + (move_y - row["cy"]) * vy
                    row["length"] = max(0.0, 2.0 * abs(proj))
                else:
                    row["cx"] = move_x
                    row["cy"] = move_y
            Redraw()
            return

        # sign / traffic_light 이동
        if state.selected_kind in ("sign", "traffic_light") and isinstance(state.selected_reference, int):
            if state.selected_kind == "sign":
                row = state.map_data.signs[state.selected_reference]
            else:
                row = state.map_data.traffic_lights[state.selected_reference]
            row["x"] = move_x; row["y"] = move_y
            Redraw()
            return

    def OnMouseDoubleClick(event: MouseEvent) -> None:
        # 더블클릭으로 guardrail 끝점 핸들 선택
        if event.inaxes != axes_map or event.button != 1:
            return
        click_x = float(event.xdata); click_y = float(event.ydata)
        if state.selected_kind == "guardrail" and isinstance(state.selected_reference, int):
            i = state.selected_reference
            row = state.map_data.guardrails[i]
            yaw_rad = math.radians(row["yaw_deg"])
            ex = row["cx"] + (row["length"] / 2.0) * math.cos(yaw_rad)
            ey = row["cy"] + (row["length"] / 2.0) * math.sin(yaw_rad)
            if Distance((click_x, click_y), (ex, ey)) <= 0.8:
                state.drag_active = True
                state.drag_role = "guardrail_end_handle"
                state.drag_start_screen = (click_x, click_y)
                state.drag_start_value = (ex, ey)

    # 더블클릭 이벤트 시뮬레이션: Matplotlib은 native 더블클릭 이벤트를 제공하지만
    # 백엔드에 따라 다르게 동작할 수 있어, 간단히 pick 방식 대신 위 함수는 필요 시 호출해도 됨.

    # 초기 패널 및 맵 그리기
    Redraw()

    # 이벤트 연결
    figure.canvas.mpl_connect("key_press_event", OnKeyPress)
    figure.canvas.mpl_connect("button_press_event", OnMousePress)
    figure.canvas.mpl_connect("button_release_event", OnMouseRelease)
    figure.canvas.mpl_connect("motion_notify_event", OnMouseMove)
    # 더블클릭 감지는 backends 마다 차이가 있으니 필요 시 OnMousePress에서 구현 확장 가능

    plt.show(block=block)

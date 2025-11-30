import os
import sys

import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
FUNCTION_DIR = os.path.join(CURRENT_DIR, "function")
if FUNCTION_DIR not in sys.path:
    sys.path.append(FUNCTION_DIR)

from editor_manager import EditorManager


# ★ Sign 클래스 옵션 (이름, 코드)
SIGN_CLASS_OPTIONS = [
    ("CLASS_UNKNOWN", 0),
    ("CLASS_STOP", 1),
    ("CLASS_YIELD", 2),
    ("CLASS_SPEED_LIMIT_30", 3),
    ("CLASS_SPEED_LIMIT_50", 4),
    ("CLASS_SPEED_LIMIT_60", 5),
    ("CLASS_SPEED_LIMIT_80", 6),
    ("CLASS_NO_ENTRY", 7),
    ("CLASS_NO_PARKING", 8),
    ("CLASS_PEDESTRIAN_CROSSING", 9),
    ("CLASS_SCHOOL_ZONE", 10),
    ("CLASS_CONSTRUCTION", 11),
    ("CLASS_TURN_LEFT", 12),
    ("CLASS_TURN_RIGHT", 13),
    ("CLASS_GO_STRAIGHT", 14),
]

TRAFFIC_LIGHT_STATE_OPTIONS = [
    ("red", 0),
    ("yellow", 1),
    ("green", 2),
]

# main.py (예: MapEditorWindow 위쪽에 추가)

class SaveBundleDialog(QtWidgets.QDialog):
    def __init__(self, parent, default_root, default_name, editor_state):
        super().__init__(parent)
        self.setWindowTitle("Save Map Bundle")
        self.setModal(True)

        v = QtWidgets.QVBoxLayout(self)

        # 경로 + 폴더명
        form = QtWidgets.QFormLayout()
        self.edit_dir = QtWidgets.QLineEdit(default_root, self)
        btn_browse = QtWidgets.QPushButton("Browse…", self)
        hb = QtWidgets.QHBoxLayout(); hb.addWidget(self.edit_dir); hb.addWidget(btn_browse)

        self.edit_name = QtWidgets.QLineEdit(default_name, self)

        form.addRow("Destination directory", hb)
        form.addRow("Bundle folder name", self.edit_name)
        v.addLayout(form)

        # 체크박스
        grid = QtWidgets.QGridLayout()
        counts = {
            "lanes": len(getattr(editor_state, "lanes", [])),
            "buildings": len(getattr(editor_state, "buildings", [])),
            "guardrails": len(getattr(editor_state, "guardrails", [])),
            "signs": len(getattr(editor_state, "signs", [])),
            "traffic_lights": len(getattr(editor_state, "traffic_lights", [])),
            "gnss_shadow_zones": len(getattr(editor_state, "gnss_shadow_zones", [])) if hasattr(editor_state, "gnss_shadow_zones") else 0,
            "surrounding_vehicles": len(getattr(editor_state, "surrounding_vehicles", [])) if hasattr(editor_state, "surrounding_vehicles") else 0,
        }
        labels = [
            ("Lanes", "lanes"),
            ("Buildings", "buildings"),
            ("Guardrails", "guardrails"),
            ("Signs", "signs"),
            ("Traffic Lights", "traffic_lights"),
            ("GNSS Shadow Zones", "gnss_shadow_zones"),
            ("Surrounding Vehicles", "surrounding_vehicles"),
        ]
        self.checks = {}
        for i, (label, key) in enumerate(labels):
            cb = QtWidgets.QCheckBox(f"{label}  ({counts[key]} items)")
            cb.setChecked(counts[key] > 0)  # 기본: 데이터가 있으면 체크
            self.checks[key] = cb
            grid.addWidget(cb, i // 2, i % 2)
        group = QtWidgets.QGroupBox("Select files to save")
        group.setLayout(grid)
        v.addWidget(group)

        # 버튼
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        v.addWidget(bb)

        # 시그널
        btn_browse.clicked.connect(self._on_browse)
        bb.accepted.connect(self._on_accept)
        bb.rejected.connect(self.reject)

        self._result = None

    def _on_browse(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Choose destination directory", self.edit_dir.text())
        if d:
            self.edit_dir.setText(d)

    def _on_accept(self):
        dest_dir = self.edit_dir.text().strip()
        name = self.edit_name.text().strip()
        if not dest_dir or not os.path.isdir(dest_dir):
            QtWidgets.QMessageBox.warning(self, "Invalid folder", "Please choose a valid destination directory.")
            return
        if not name:
            QtWidgets.QMessageBox.warning(self, "Missing name", "Please enter a bundle folder name.")
            return
        which = {k for k, cb in self.checks.items() if cb.isChecked()}
        if not which:
            if QtWidgets.QMessageBox.question(self, "Nothing selected",
                                              "No file types selected. Create empty folder anyway?",
                                              QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
                return
        self._result = {
            "out_dir": os.path.join(dest_dir, name),
            "which": which
        }
        self.accept()

    def result(self):
        return self._result



class MapEditorWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("DCAS Map Editor (GUI)")
        self.resize(2000, 1400) # 1500, 900

        # 우측 에디터 값 로딩 중 시그널 차단 가드
        self.is_loading_lane_fields = False
        self.is_loading_building_fields = False
        self.is_loading_sign_fields = False  # ★ 추가
        self.is_loading_traffic_fields = False  # ★ 추가
        self.is_loading_gnss_fields = False  # ★ NEW

        # ===== 레이아웃 골격 =====
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central)
        h.setContentsMargins(8, 8, 8, 8)
        h.setSpacing(8)

        # 좌측 컨트롤 패널 (Stack)
        self.left_panel = self._BuildLeftPanel()
        h.addWidget(self.left_panel, 0)

        # 중앙 플롯
        self.figure, self.axes = plt.subplots(figsize=(8, 6))
        self.axes.set_xlabel("x")
        self.axes.set_ylabel("y")
        self.axes.grid(True)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.canvas.setFocus()
        h.addWidget(self.canvas, 1)

        self.axes.set_xlim(-50, 50)
        self.axes.set_ylim(-50, 50)

        # 우측 오버뷰+에디터 (Stack)
        self.right_stack = QtWidgets.QStackedWidget(self)
        self.lanes_right = self._BuildLanesRightPanel()
        self.buildings_right = self._BuildBuildingsRightPanel()
        self.signs_right = self._BuildSignsRightPanel()   # ★ 추가
        self.traffic_right = self._BuildTrafficLightsRightPanel()   # ★ 추가
        self.guardrails_right = self._BuildGuardrailsRightPanel()
        self.gnss_right = self._BuildGnssShadowZonesRightPanel()  # ★ NEW

        self.surrounding_right = self._BuildSurroundingVehiclesRightPanel()

        self.right_stack.addWidget(self.lanes_right)      # index 0
        self.right_stack.addWidget(self.buildings_right)  # index 1
        self.right_stack.addWidget(self.signs_right)      # ★ index 2
        self.right_stack.addWidget(self.traffic_right)     # ★ index 3
        self.right_stack.addWidget(self.guardrails_right)
        self.right_stack.addWidget(self.gnss_right)               # ★ NEW
        self.right_stack.addWidget(self.surrounding_right)  # 새로운 인덱스(마지막)


        h.addWidget(self.right_stack, 0)

        # ===== 에디터 매니저 =====
        self.editor_manager = EditorManager(figure=self.figure, axes=self.axes)
        self.editor_manager.Initialize()

        # 목록 갱신 콜백
        # self.editor_manager.SetOnLanesChanged(self.RefreshLaneList)
        self.editor_manager.SetOnLanesChanged(self._OnLanesListOrGeometryChanged)

        self.editor_manager.SetOnBuildingsChanged(self.RefreshBuildingList)

        self.editor_manager.SetOnGuardrailsChanged(self.RefreshGuardrailList)

        # ★ 지오메트리 변경 콜백(마우스로 이동 시 우측 숫자 필드 즉시 동기화)
        # if hasattr(self.editor_manager, "SetOnLanesGeometryChanged"):
        #     self.editor_manager.SetOnLanesGeometryChanged(self.LoadLaneEditorFields)
        if hasattr(self.editor_manager, "SetOnLanesGeometryChanged"):
            self.editor_manager.SetOnLanesGeometryChanged(self._OnLanesListOrGeometryChanged)

        # ★ 추가: Buildings 지오메트리 변경 → 우측 패널 동기화
        if hasattr(self.editor_manager, "SetOnBuildingsGeometryChanged"):
            self.editor_manager.SetOnBuildingsGeometryChanged(self.LoadBuildingEditorFields)

        # 목록 갱신 콜백들 아래에 이어서
        if hasattr(self.editor_manager, "SetOnSignsChanged"):
            self.editor_manager.SetOnSignsChanged(self.RefreshSignList)

        # 지오메트리 변경 콜백들 아래에 이어서
        if hasattr(self.editor_manager, "SetOnSignsGeometryChanged"):
            self.editor_manager.SetOnSignsGeometryChanged(self.LoadSignEditorFields)

        # 목록 콜백들 아래에 이어서
        if hasattr(self.editor_manager, "SetOnTrafficLightsChanged"):
            self.editor_manager.SetOnTrafficLightsChanged(self.RefreshTrafficLightList)

        # 지오메트리 변경 콜백들 아래에 이어서
        if hasattr(self.editor_manager, "SetOnTrafficLightsGeometryChanged"):
            self.editor_manager.SetOnTrafficLightsGeometryChanged(self.LoadTrafficLightEditorFields)
        
        if hasattr(self.editor_manager, "SetOnGuardrailsGeometryChanged"):
            self.editor_manager.SetOnGuardrailsGeometryChanged(self.LoadGuardrailEditorFields)
        
        # (에디터 매니저 콜백 연결 부분)
        if hasattr(self.editor_manager, "SetOnGnssShadowZonesChanged"):
            self.editor_manager.SetOnGnssShadowZonesChanged(self._OnGnssListOrGeometryChanged)
        if hasattr(self.editor_manager, "SetOnGnssShadowZonesGeometryChanged"):
            self.editor_manager.SetOnGnssShadowZonesGeometryChanged(self._OnGnssListOrGeometryChanged)

        # 목록 갱신 콜백
        if hasattr(self.editor_manager, "SetOnSurroundingVehiclesChanged"):
            self.editor_manager.SetOnSurroundingVehiclesChanged(self.RefreshSurroundingVehicleList)

        # [ADD] 지오메트리 변경 콜백
        if hasattr(self.editor_manager, "SetOnSurroundingVehiclesGeometryChanged"):
            self.editor_manager.SetOnSurroundingVehiclesGeometryChanged(self.LoadSurroundingVehicleEditorFields)
        



        # 상태바
        self.status_bar = QtWidgets.QStatusBar(self)
        self.setStatusBar(self.status_bar)
        self._SetStatus("Ready.")

        # 신호 연결
        self._ConnectSignals()

        # 초기 UI 상태
        self._SwitchModeUi("lanes")
        self.UpdateLaneSubmodeButtons(self.editor_manager.GetLaneSubmode())
        self.UpdateBuildingSubmodeButtons(self.editor_manager.GetBuildingSubmode())
        
        self.RefreshLaneList()
        self.RefreshBuildingList()
        self.RefreshGuardrailList()

        self.LoadLaneEditorFields()
        self.LoadBuildingEditorFields()
        self.LoadGuardrailEditorFields()

        self.UpdateSignSubmodeButtons(getattr(self.editor_manager, "GetSignSubmode", lambda: "view")())
        self.RefreshSignList()
        self.LoadSignEditorFields()

        # 초기 UI 상태 블록에 아래 세 줄 추가
        self.UpdateTrafficLightSubmodeButtons(getattr(self.editor_manager, "GetTrafficLightsSubmode", lambda: "view")())
        self.RefreshTrafficLightList()
        self.LoadTrafficLightEditorFields()

        # (초기 UI 세팅 끝부분, 트래픽/가드레일 초기화 뒤)
        self.UpdateGnssShadowZonesSubmodeButtons(
            getattr(self.editor_manager, "GetGnssShadowZonesSubmode", lambda: "view")()
        )  # ★ NEW
        self.RefreshGnssShadowZoneList()           # ★ NEW
        self.LoadGnssShadowZoneEditorFields()      # ★ NEW
        self.ApplyGnssShadowZonesCreateDefaults()  # ★ NEW

        # ★ Create Settings 기본값을 에디터로 1회 반영
        self.ApplyTrafficLightCreateDefaults()

        # Oncomming vehicles
        self.UpdateSurroundingVehiclesSubmodeButtons(getattr(self.editor_manager, "GetSurroundingVehiclesSubmode", lambda: "view")())
        self.RefreshSurroundingVehicleList()
        self.LoadSurroundingVehicleEditorFields()
        self.editor_manager.SetSurroundingVehiclesCreateDefaults(speed_mps=float(self.spin_surrounding_speed.value()))

    # =========================
    # 좌측 패널
    # =========================
    def _BuildLeftPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(10)

        # Map Bundle
        group_bundle = QtWidgets.QGroupBox("Map Bundle", panel)
        vb = QtWidgets.QVBoxLayout(group_bundle)
        self.button_new_bundle = QtWidgets.QPushButton("New Bundle Name", group_bundle)
        self.button_open_bundle = QtWidgets.QPushButton("Open Bundle Folder…", group_bundle)
        self.button_save_bundle = QtWidgets.QPushButton("Save Bundle", group_bundle)
        vb.addWidget(self.button_new_bundle)
        vb.addWidget(self.button_open_bundle)
        vb.addWidget(self.button_save_bundle)
        v.addWidget(group_bundle)

        # View
        group_view = QtWidgets.QGroupBox("View", panel)
        vv = QtWidgets.QVBoxLayout(group_view)
        self.button_fit_view = QtWidgets.QPushButton("Fit To Data", group_view)
        self.toggle_lock_view = QtWidgets.QPushButton("Lock View", group_view)
        self.toggle_lock_view.setCheckable(True)
        self.toggle_lock_view.setChecked(True)
        self.toggle_common_view = QtWidgets.QPushButton("Common View Mode", group_view)
        self.toggle_common_view.setCheckable(True)
        vv.addWidget(self.button_fit_view)
        vv.addWidget(self.toggle_lock_view)
        vv.addWidget(self.toggle_common_view)
        v.addWidget(group_view)

        # Edit Mode
        group_mode = QtWidgets.QGroupBox("Edit Mode", panel)
        vm = QtWidgets.QVBoxLayout(group_mode)
        self.radio_mode_lanes = QtWidgets.QRadioButton("Lanes")
        self.radio_mode_buildings = QtWidgets.QRadioButton("Buildings")
        self.radio_mode_guardrails = QtWidgets.QRadioButton("Guardrails")
        self.radio_mode_signs = QtWidgets.QRadioButton("Signs")
        self.radio_mode_traffic = QtWidgets.QRadioButton("Traffic Lights")
        self.radio_mode_gnss = QtWidgets.QRadioButton("GNSS Shadow Zones")  # ★ NEW
        self.radio_mode_surrounding = QtWidgets.QRadioButton("Surrounding Vehicles")

        self.radio_mode_lanes.setChecked(True)
        for w in [self.radio_mode_lanes, self.radio_mode_buildings,
                  self.radio_mode_guardrails, self.radio_mode_signs, self.radio_mode_traffic,
                  self.radio_mode_traffic, self.radio_mode_gnss, self.radio_mode_surrounding]:
            vm.addWidget(w)
        v.addWidget(group_mode)

        # ---------- Subpanel Stack (Lanes / Buildings) ----------
        self.left_stack = QtWidgets.QStackedWidget(panel)
        v.addWidget(self.left_stack)

        # Lanes subpanel
        lanes_page = QtWidgets.QWidget(panel)
        vl = QtWidgets.QVBoxLayout(lanes_page)
        group_lanes_sub = QtWidgets.QGroupBox("Lanes Submode", lanes_page)
        gl = QtWidgets.QGridLayout(group_lanes_sub)
        self.button_lane_view = QtWidgets.QPushButton("View"); self.button_lane_view.setCheckable(True)
        self.button_lane_create = QtWidgets.QPushButton("Create"); self.button_lane_create.setCheckable(True)
        self.button_lane_move = QtWidgets.QPushButton("Modify"); self.button_lane_move.setCheckable(True)
        self.button_lane_delete = QtWidgets.QPushButton("Delete"); self.button_lane_delete.setCheckable(True)

        gl.addWidget(self.button_lane_view, 0, 0)
        gl.addWidget(self.button_lane_create, 0, 1)
        gl.addWidget(self.button_lane_move, 1, 0)
        gl.addWidget(self.button_lane_delete, 1, 1)
        vl.addWidget(group_lanes_sub)

        # Create 용 Apply 버튼 추가
        self.button_lane_apply = QtWidgets.QPushButton("Apply Current Lane")
        vl.addWidget(self.button_lane_apply)

        # --- Modify 모드 세부 옵션 ---
        group_lanes_modify_mode = QtWidgets.QGroupBox("Modify Mode")
        gm = QtWidgets.QHBoxLayout(group_lanes_modify_mode)
        self.radio_lane_modify_node = QtWidgets.QRadioButton("Node")
        self.radio_lane_modify_lane = QtWidgets.QRadioButton("Lane")
        self.radio_lane_modify_node.setChecked(True)
        gm.addWidget(self.radio_lane_modify_node)
        gm.addWidget(self.radio_lane_modify_lane)
        vl.addWidget(group_lanes_modify_mode)

        # --- Delete 모드 세부 옵션 ---
        group_lanes_delete_mode = QtWidgets.QGroupBox("Delete Mode")
        gd = QtWidgets.QHBoxLayout(group_lanes_delete_mode)
        self.radio_lane_delete_node = QtWidgets.QRadioButton("Node")
        self.radio_lane_delete_lane = QtWidgets.QRadioButton("Lane")
        self.radio_lane_delete_node.setChecked(True)
        gd.addWidget(self.radio_lane_delete_node)
        gd.addWidget(self.radio_lane_delete_lane)
        vl.addWidget(group_lanes_delete_mode)


        group_lanes_type = QtWidgets.QGroupBox("Lanes Type", lanes_page)
        gt = QtWidgets.QGridLayout(group_lanes_type)

        self.button_lane_type_unknown          = QtWidgets.QPushButton("TYPE_UNKNOWN")
        self.button_lane_type_white_solid      = QtWidgets.QPushButton("TYPE_WHITE_SOLID")
        self.button_lane_type_white_dashed     = QtWidgets.QPushButton("TYPE_WHITE_DASHED")
        self.button_lane_type_yellow_solid     = QtWidgets.QPushButton("TYPE_YELLOW_SOLID")
        self.button_lane_type_yellow_dashed    = QtWidgets.QPushButton("TYPE_YELLOW_DASHED")
        self.button_lane_type_double_yellow    = QtWidgets.QPushButton("TYPE_DOUBLE_YELLOW_SOLID")

        for i, b in enumerate([
            self.button_lane_type_unknown,
            self.button_lane_type_white_solid,
            self.button_lane_type_white_dashed,
            self.button_lane_type_yellow_solid,
            self.button_lane_type_yellow_dashed,
            self.button_lane_type_double_yellow
        ]):
            gt.addWidget(b, i // 2, i % 2)
        vl.addWidget(group_lanes_type)
        vl.addStretch(1)
        self.left_stack.addWidget(lanes_page)  # index 0

        # 클릭 핸들러
        self.button_lane_type_unknown.clicked.connect(lambda: self._OnSetLaneType("TYPE_UNKNOWN"))
        self.button_lane_type_white_solid.clicked.connect(lambda: self._OnSetLaneType("TYPE_WHITE_SOLID"))
        self.button_lane_type_white_dashed.clicked.connect(lambda: self._OnSetLaneType("TYPE_WHITE_DASHED"))
        self.button_lane_type_yellow_solid.clicked.connect(lambda: self._OnSetLaneType("TYPE_YELLOW_SOLID"))
        self.button_lane_type_yellow_dashed.clicked.connect(lambda: self._OnSetLaneType("TYPE_YELLOW_DASHED"))
        self.button_lane_type_double_yellow.clicked.connect(lambda: self._OnSetLaneType("TYPE_DOUBLE_YELLOW_SOLID"))

        # Buildings subpanel
        buildings_page = QtWidgets.QWidget(panel)
        vbld = QtWidgets.QVBoxLayout(buildings_page)

        group_bsub = QtWidgets.QGroupBox("Buildings Submode", buildings_page)
        gb = QtWidgets.QGridLayout(group_bsub)
        self.button_building_view   = QtWidgets.QPushButton("View");   self.button_building_view.setCheckable(True)
        self.button_building_create = QtWidgets.QPushButton("Create"); self.button_building_create.setCheckable(True)
        self.button_building_modify = QtWidgets.QPushButton("Modify"); self.button_building_modify.setCheckable(True)
        self.button_building_delete = QtWidgets.QPushButton("Delete"); self.button_building_delete.setCheckable(True)
        gb.addWidget(self.button_building_view, 0, 0); gb.addWidget(self.button_building_create, 0, 1)
        gb.addWidget(self.button_building_modify, 1, 0); gb.addWidget(self.button_building_delete, 1, 1)
        vbld.addWidget(group_bsub)

        group_bcreate = QtWidgets.QGroupBox("Create Settings", buildings_page)
        fb = QtWidgets.QFormLayout(group_bcreate)
        self.spin_building_vertices_count = QtWidgets.QSpinBox(group_bcreate)
        self.spin_building_vertices_count.setRange(3, 64); self.spin_building_vertices_count.setValue(4)
        self.spin_building_vertex_radius = QtWidgets.QDoubleSpinBox(group_bcreate)
        self.spin_building_vertex_radius.setDecimals(3)
        self.spin_building_vertex_radius.setRange(0.01, 1_000.0)
        self.spin_building_vertex_radius.setValue(10.0)
        self.spin_building_height = QtWidgets.QDoubleSpinBox(group_bcreate)
        self.spin_building_height.setDecimals(3); self.spin_building_height.setRange(0.01, 1_000.0); self.spin_building_height.setValue(3.0)
        fb.addRow("Number of vertices", self.spin_building_vertices_count)
        fb.addRow("Radius from center", self.spin_building_vertex_radius)
        fb.addRow("Height", self.spin_building_height)
        vbld.addWidget(group_bcreate)

        vbld.addStretch(1)
        self.left_stack.addWidget(buildings_page)  # index 1

        v.addStretch(1)

        # ★ Signs subpanel (좌측)
        signs_page = QtWidgets.QWidget(panel)
        vs = QtWidgets.QVBoxLayout(signs_page)

        group_ssub = QtWidgets.QGroupBox("Signs Submode", signs_page)
        gs = QtWidgets.QGridLayout(group_ssub)
        self.button_sign_view   = QtWidgets.QPushButton("View");   self.button_sign_view.setCheckable(True)
        self.button_sign_create = QtWidgets.QPushButton("Create"); self.button_sign_create.setCheckable(True)
        self.button_sign_move   = QtWidgets.QPushButton("Modify");   self.button_sign_move.setCheckable(True)
        self.button_sign_delete = QtWidgets.QPushButton("Delete"); self.button_sign_delete.setCheckable(True)
        gs.addWidget(self.button_sign_view, 0, 0); gs.addWidget(self.button_sign_create, 0, 1)
        gs.addWidget(self.button_sign_move, 1, 0); gs.addWidget(self.button_sign_delete, 1, 1)
        vs.addWidget(group_ssub)

        group_screate = QtWidgets.QGroupBox("Create Settings", signs_page)
        fs = QtWidgets.QFormLayout(group_screate)

        self.combo_sign_class = QtWidgets.QComboBox(group_screate)
        for name, code in SIGN_CLASS_OPTIONS:
            self.combo_sign_class.addItem(name, code)
        self.combo_sign_class.setCurrentIndex(0)  # CLASS_UNKNOWN

        self.spin_sign_z = QtWidgets.QDoubleSpinBox(group_screate)
        self.spin_sign_z.setDecimals(3); self.spin_sign_z.setRange(-1_000_000.0, 1_000_000.0)
        self.spin_sign_z.setSingleStep(0.1); self.spin_sign_z.setValue(1.0)

        self.spin_sign_yaw = QtWidgets.QDoubleSpinBox(group_screate)
        self.spin_sign_yaw.setDecimals(3); self.spin_sign_yaw.setRange(-3600.0, 3600.0)
        self.spin_sign_yaw.setSingleStep(1.0); self.spin_sign_yaw.setValue(90.0)

        fs.addRow("sign_class", self.combo_sign_class)
        fs.addRow("z (height)", self.spin_sign_z)
        fs.addRow("yaw (deg)", self.spin_sign_yaw)
        vs.addWidget(group_screate)

        vs.addStretch(1)
        self.left_stack.addWidget(signs_page)  # ★ index 2

        # ★ Traffic Lights subpanel (좌측)
        traffic_page = QtWidgets.QWidget(panel)
        vt = QtWidgets.QVBoxLayout(traffic_page)

        group_tsub = QtWidgets.QGroupBox("Traffic Lights Submode", traffic_page)
        gt = QtWidgets.QGridLayout(group_tsub)
        self.button_traffic_view   = QtWidgets.QPushButton("View");   self.button_traffic_view.setCheckable(True)
        self.button_traffic_create = QtWidgets.QPushButton("Create"); self.button_traffic_create.setCheckable(True)
        self.button_traffic_modify = QtWidgets.QPushButton("Modify"); self.button_traffic_modify.setCheckable(True)
        self.button_traffic_delete = QtWidgets.QPushButton("Delete"); self.button_traffic_delete.setCheckable(True)
        gt.addWidget(self.button_traffic_view,   0, 0); gt.addWidget(self.button_traffic_create, 0, 1)
        gt.addWidget(self.button_traffic_modify, 1, 0); gt.addWidget(self.button_traffic_delete, 1, 1)
        vt.addWidget(group_tsub)

        group_tcreate = QtWidgets.QGroupBox("Create Settings", traffic_page)
        ft = QtWidgets.QFormLayout(group_tcreate)

        self.combo_traffic_state = QtWidgets.QComboBox(group_tcreate)
        for name, code in TRAFFIC_LIGHT_STATE_OPTIONS:
            self.combo_traffic_state.addItem(name, code)
        self.combo_traffic_state.setCurrentIndex(0)  # red=0

        self.spin_traffic_z = QtWidgets.QDoubleSpinBox(group_tcreate)
        self.spin_traffic_z.setDecimals(3); self.spin_traffic_z.setRange(-1_000_000.0, 1_000_000.0)
        self.spin_traffic_z.setSingleStep(0.1); self.spin_traffic_z.setValue(5.0)   # 예시값

        ft.addRow("state", self.combo_traffic_state)
        ft.addRow("z (height)", self.spin_traffic_z)
        vt.addWidget(group_tcreate)

        vt.addStretch(1)
        self.left_stack.addWidget(traffic_page)  # ★ index 3

        # guardrails
        # --- Guardrails subpanel ---
        guardrails_page = QtWidgets.QWidget(panel)
        vg = QtWidgets.QVBoxLayout(guardrails_page)

        group_gsub = QtWidgets.QGroupBox("Guardrails Submode", guardrails_page)
        gg = QtWidgets.QGridLayout(group_gsub)
        self.button_guard_view   = QtWidgets.QPushButton("View");   self.button_guard_view.setCheckable(True)
        self.button_guard_create = QtWidgets.QPushButton("Create"); self.button_guard_create.setCheckable(True)
        self.button_guard_modify = QtWidgets.QPushButton("Modify"); self.button_guard_modify.setCheckable(True)
        self.button_guard_delete = QtWidgets.QPushButton("Delete"); self.button_guard_delete.setCheckable(True)
        gg.addWidget(self.button_guard_view,   0, 0)
        gg.addWidget(self.button_guard_create, 0, 1)
        gg.addWidget(self.button_guard_modify, 1, 0)
        gg.addWidget(self.button_guard_delete, 1, 1)
        vg.addWidget(group_gsub)

        group_gcreate = QtWidgets.QGroupBox("Create Settings", guardrails_page)
        fg = QtWidgets.QFormLayout(group_gcreate)
        self.spin_guard_length = QtWidgets.QDoubleSpinBox(group_gcreate)
        self.spin_guard_length.setDecimals(3); self.spin_guard_length.setRange(0.01, 1_000_000.0)
        self.spin_guard_length.setSingleStep(0.1); self.spin_guard_length.setValue(2.0)
        self.spin_guard_yaw = QtWidgets.QDoubleSpinBox(group_gcreate)
        self.spin_guard_yaw.setDecimals(3); self.spin_guard_yaw.setRange(-3600.0, 3600.0)
        self.spin_guard_yaw.setSingleStep(1.0); self.spin_guard_yaw.setValue(0.0)
        fg.addRow("Length", self.spin_guard_length)
        fg.addRow("Yaw (deg)", self.spin_guard_yaw)
        vg.addWidget(group_gcreate)

        vg.addStretch(1)
        self.guardrails_left = guardrails_page
        self.left_stack.addWidget(self.guardrails_left)  # ← left_stack에 페이지 등록

        # (left_stack에 각 서브패널 넣는 곳 맨 아래에 추가)
        # ★ NEW: GNSS Shadow Zones subpanel
        gnss_page = QtWidgets.QWidget(panel)
        vz = QtWidgets.QVBoxLayout(gnss_page)

        group_zsub = QtWidgets.QGroupBox("GNSS Shadow Zones Submode", gnss_page)
        gz = QtWidgets.QGridLayout(group_zsub)
        self.button_gnss_view   = QtWidgets.QPushButton("View");   self.button_gnss_view.setCheckable(True)
        self.button_gnss_create = QtWidgets.QPushButton("Create"); self.button_gnss_create.setCheckable(True)
        self.button_gnss_move   = QtWidgets.QPushButton("Modify");   self.button_gnss_move.setCheckable(True)
        self.button_gnss_delete = QtWidgets.QPushButton("Delete"); self.button_gnss_delete.setCheckable(True)
        gz.addWidget(self.button_gnss_view, 0, 0)
        gz.addWidget(self.button_gnss_create, 0, 1)
        gz.addWidget(self.button_gnss_move, 1, 0)
        gz.addWidget(self.button_gnss_delete, 1, 1)
        vz.addWidget(group_zsub)

        # ---- Surrounding Vehicles subpanel
        surrounding_page = QtWidgets.QWidget(panel)
        vo = QtWidgets.QVBoxLayout(surrounding_page)

        group_osub = QtWidgets.QGroupBox("Surrounding Vehicles Submode", surrounding_page)
        go = QtWidgets.QGridLayout(group_osub)
        self.button_surrounding_view   = QtWidgets.QPushButton("View");   self.button_surrounding_view.setCheckable(True)
        self.button_surrounding_create = QtWidgets.QPushButton("Create"); self.button_surrounding_create.setCheckable(True)
        self.button_surrounding_modify = QtWidgets.QPushButton("Modify"); self.button_surrounding_modify.setCheckable(True)
        self.button_surrounding_delete = QtWidgets.QPushButton("Delete"); self.button_surrounding_delete.setCheckable(True)
        go.addWidget(self.button_surrounding_view,   0, 0)
        go.addWidget(self.button_surrounding_create, 0, 1)
        go.addWidget(self.button_surrounding_modify, 1, 0)
        go.addWidget(self.button_surrounding_delete, 1, 1)
        vo.addWidget(group_osub)

        # Create - Apply 버튼
        self.button_surrounding_apply = QtWidgets.QPushButton("Apply Current Surrounding Vehicle")
        vo.addWidget(self.button_surrounding_apply)

        # Modify 모드 옵션
        group_surrounding_modify = QtWidgets.QGroupBox("Modify Mode", surrounding_page)
        gom = QtWidgets.QHBoxLayout(group_surrounding_modify)
        self.radio_surrounding_modify_node = QtWidgets.QRadioButton("Node")
        self.radio_surrounding_modify_vehicle = QtWidgets.QRadioButton("Trajectory")
        self.radio_surrounding_modify_node.setChecked(True)
        gom.addWidget(self.radio_surrounding_modify_node)
        gom.addWidget(self.radio_surrounding_modify_vehicle)
        vo.addWidget(group_surrounding_modify)

        # Delete 모드 옵션
        group_surrounding_delete = QtWidgets.QGroupBox("Delete Mode", surrounding_page)
        god = QtWidgets.QHBoxLayout(group_surrounding_delete)
        self.radio_surrounding_delete_node = QtWidgets.QRadioButton("Node")
        self.radio_surrounding_delete_vehicle = QtWidgets.QRadioButton("Trajectory")
        self.radio_surrounding_delete_node.setChecked(True)
        god.addWidget(self.radio_surrounding_delete_node)
        god.addWidget(self.radio_surrounding_delete_vehicle)
        vo.addWidget(group_surrounding_delete)

        # Create Settings
        group_ocreate = QtWidgets.QGroupBox("Create Settings", surrounding_page)
        fo = QtWidgets.QFormLayout(group_ocreate)
        self.spin_surrounding_speed = QtWidgets.QDoubleSpinBox(group_ocreate)
        self.spin_surrounding_speed.setDecimals(3)
        self.spin_surrounding_speed.setRange(0.0, 1_000.0)
        self.spin_surrounding_speed.setSingleStep(0.5)
        self.spin_surrounding_speed.setValue(5.0)
        fo.addRow("speed_mps", self.spin_surrounding_speed)
        vo.addWidget(group_ocreate)

        vo.addStretch(1)
        self.left_stack.addWidget(surrounding_page)  # ← 새 페이지 등록

        group_zcreate = QtWidgets.QGroupBox("Create Settings", gnss_page)
        fz = QtWidgets.QFormLayout(group_zcreate)
        self.spin_gnss_pos_std = QtWidgets.QDoubleSpinBox(group_zcreate)
        self.spin_gnss_pos_std.setDecimals(3); self.spin_gnss_pos_std.setRange(0.0, 1_000_000.0)
        self.spin_gnss_pos_std.setSingleStep(0.1); self.spin_gnss_pos_std.setValue(1.0)
        self.spin_gnss_radius = QtWidgets.QDoubleSpinBox(group_zcreate)
        self.spin_gnss_radius.setDecimals(3); self.spin_gnss_radius.setRange(0.0, 1_000_000.0)
        self.spin_gnss_radius.setSingleStep(0.1); self.spin_gnss_radius.setValue(10.0)
        fz.addRow("pos_std (m)", self.spin_gnss_pos_std)
        fz.addRow("radius (m)", self.spin_gnss_radius)
        vz.addWidget(group_zcreate)

        vz.addStretch(1)
        self.gnss_left = gnss_page
        self.left_stack.addWidget(self.gnss_left)  # ★ NEW

        self.surrounding_left = surrounding_page
        self.left_stack.addWidget(self.surrounding_left)

        return panel
    
    # =========================
    # 오른쪽 패널: Lanes
    # =========================
    def _BuildLanesRightPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(8)

        # Overview
        group_list = QtWidgets.QGroupBox("Lanes Overview", panel)
        vl = QtWidgets.QVBoxLayout(group_list)
        self.list_lanes = QtWidgets.QListWidget(group_list)
        self.list_lanes.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vl.addWidget(self.list_lanes)
        v.addWidget(group_list)

        # Editor
        group_edit = QtWidgets.QGroupBox("Selected Lane Editor (Modify mode)", panel)
        self.form_lane = QtWidgets.QFormLayout(group_edit)

        # Lane type (유지)
        self.combo_lane_type = QtWidgets.QComboBox(group_edit)
        # self.combo_lane_type.addItems(["driving", "shoulder", "bike", "bus", "parking"])
        self.combo_lane_type.addItems([
            "TYPE_UNKNOWN",
            "TYPE_WHITE_SOLID",
            "TYPE_WHITE_DASHED",
            "TYPE_YELLOW_SOLID",
            "TYPE_YELLOW_DASHED",
            "TYPE_DOUBLE_YELLOW_SOLID",
        ])
        self.form_lane.addRow("Lane Type", self.combo_lane_type)

        # ★ Nodes 동적 영역
        self.group_lane_nodes = QtWidgets.QGroupBox("Nodes")
        self.nodes_layout = QtWidgets.QFormLayout(self.group_lane_nodes)
        self.form_lane.addRow(self.group_lane_nodes)

        v.addWidget(group_edit)
        self.group_lane_editor = group_edit

        # 시그널: lane type만 유지, start/end 스핀박스 관련 연결은 모두 제거
        self.combo_lane_type.currentTextChanged.connect(self._OnLaneTypeChanged)

        v.addStretch(1)
        return panel

    # =========================
    # 오른쪽 패널: Buildings
    # =========================
    def _BuildBuildingsRightPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(8)

        group_list = QtWidgets.QGroupBox("Buildings Overview", panel)
        vb = QtWidgets.QVBoxLayout(group_list)
        self.list_buildings = QtWidgets.QListWidget(group_list)
        self.list_buildings.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vb.addWidget(self.list_buildings)
        v.addWidget(group_list)

        group_edit = QtWidgets.QGroupBox("Selected Building Editor (Modify mode)", panel)
        self.form_building = QtWidgets.QFormLayout(group_edit)

        def mkspin():
            s = QtWidgets.QDoubleSpinBox(group_edit)
            s.setDecimals(3); s.setRange(-1_000_000.0, 1_000_000.0); s.setSingleStep(0.1)
            return s
        self.spin_center_x = mkspin(); self.spin_center_y = mkspin()
        self.spin_rotation_deg = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_rotation_deg.setDecimals(3); self.spin_rotation_deg.setRange(-3600.0, 3600.0); self.spin_rotation_deg.setSingleStep(1.0)
        self.spin_height = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_height.setDecimals(3); self.spin_height.setRange(0.01, 1_000.0); self.spin_height.setSingleStep(0.1)

        self.form_building.addRow("Center x", self.spin_center_x)
        self.form_building.addRow("Center y", self.spin_center_y)
        self.form_building.addRow("Rotation r (deg)", self.spin_rotation_deg)
        self.form_building.addRow("Height h", self.spin_height)

        # 동적 꼭짓점 편집 영역
        self.vertices_container = QtWidgets.QGroupBox("Vertices")
        self.vertices_layout = QtWidgets.QFormLayout(self.vertices_container)
        self.form_building.addRow(self.vertices_container)

        v.addWidget(group_edit)
        self.group_building_editor = group_edit

        # 즉시 반영
        self.list_buildings.itemSelectionChanged.connect(self._OnBuildingSelectionChanged)
        self.spin_center_x.valueChanged.connect(self._OnBuildingFieldsChanged)
        self.spin_center_y.valueChanged.connect(self._OnBuildingFieldsChanged)
        self.spin_rotation_deg.valueChanged.connect(self._OnBuildingRotationChanged)
        self.spin_height.valueChanged.connect(self._OnBuildingHeightChanged)

        v.addStretch(1)
        return panel
    

    # =========================
    # 오른쪽 패널: Signs
    # =========================
    def _BuildSignsRightPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(8)

        group_list = QtWidgets.QGroupBox("Signs Overview", panel)
        vl = QtWidgets.QVBoxLayout(group_list)
        self.list_signs = QtWidgets.QListWidget(group_list)
        self.list_signs.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vl.addWidget(self.list_signs)
        v.addWidget(group_list)

        group_edit = QtWidgets.QGroupBox("Selected Sign Editor (Modify mode)", panel)
        fs = QtWidgets.QFormLayout(group_edit)

        def mkspin():
            s = QtWidgets.QDoubleSpinBox(group_edit)
            s.setDecimals(3); s.setRange(-1_000_000.0, 1_000_000.0); s.setSingleStep(0.1)
            return s

        self.spin_sign_x = mkspin()
        self.spin_sign_y = mkspin()
        self.spin_sign_z_edit = mkspin()

        self.spin_sign_yaw_edit = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_sign_yaw_edit.setDecimals(3); self.spin_sign_yaw_edit.setRange(-3600.0, 3600.0); self.spin_sign_yaw_edit.setSingleStep(1.0)

        self.combo_sign_class_edit = QtWidgets.QComboBox(group_edit)
        for name, code in SIGN_CLASS_OPTIONS:
            self.combo_sign_class_edit.addItem(name, code)

        fs.addRow("Center x", self.spin_sign_x)
        fs.addRow("Center y", self.spin_sign_y)
        fs.addRow("Height z", self.spin_sign_z_edit)
        fs.addRow("yaw", self.spin_sign_yaw_edit)
        fs.addRow("sign class", self.combo_sign_class_edit)
        v.addWidget(group_edit)
        self.group_sign_editor = group_edit

        # 시그널 (핸들러는 아래 9)~13)에서 추가)
        self.list_signs.itemSelectionChanged.connect(self._OnSignSelectionChanged)
        self.spin_sign_x.valueChanged.connect(self._OnSignFieldsChanged)
        self.spin_sign_y.valueChanged.connect(self._OnSignFieldsChanged)
        self.spin_sign_z_edit.valueChanged.connect(self._OnSignHeightChanged)
        self.spin_sign_yaw_edit.valueChanged.connect(self._OnSignYawChanged)
        self.combo_sign_class_edit.currentIndexChanged.connect(self._OnSignClassChanged)

        v.addStretch(1)
        return panel

    # =========================
    # 오른쪽 패널: Traffic Lights
    # =========================
    def _BuildTrafficLightsRightPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(8)

        group_list = QtWidgets.QGroupBox("Traffic Lights Overview", panel)
        vl = QtWidgets.QVBoxLayout(group_list)
        self.list_traffic = QtWidgets.QListWidget(group_list)
        self.list_traffic.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vl.addWidget(self.list_traffic)
        v.addWidget(group_list)

        group_edit = QtWidgets.QGroupBox("Selected Traffic Lights Editor (Modify mode)", panel)
        ft = QtWidgets.QFormLayout(group_edit)

        def mkspin():
            s = QtWidgets.QDoubleSpinBox(group_edit)
            s.setDecimals(3); s.setRange(-1_000_000.0, 1_000_000.0); s.setSingleStep(0.1)
            return s

        self.spin_traffic_x = mkspin()
        self.spin_traffic_y = mkspin()
        self.spin_traffic_z_edit = mkspin()

        self.combo_traffic_state_edit = QtWidgets.QComboBox(group_edit)
        for name, code in TRAFFIC_LIGHT_STATE_OPTIONS:
            self.combo_traffic_state_edit.addItem(name, code)

        ft.addRow("Center x", self.spin_traffic_x)
        ft.addRow("Center y", self.spin_traffic_y)
        ft.addRow("Height z", self.spin_traffic_z_edit)
        ft.addRow("state", self.combo_traffic_state_edit)
        v.addWidget(group_edit)
        self.group_traffic_editor = group_edit

        # 시그널 연결
        self.list_traffic.itemSelectionChanged.connect(self._OnTrafficLightSelectionChanged)
        self.spin_traffic_x.valueChanged.connect(self._OnTrafficLightCenterChanged)
        self.spin_traffic_y.valueChanged.connect(self._OnTrafficLightCenterChanged)
        self.spin_traffic_z_edit.valueChanged.connect(self._OnTrafficLightHeightChanged)
        self.combo_traffic_state_edit.currentIndexChanged.connect(self._OnTrafficLightStateChanged)

        v.addStretch(1)
        return panel
    
    # =========================
    # 오른쪽 패널: GuardRails
    # =========================
    def _BuildGuardrailsRightPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(8)

        group_list = QtWidgets.QGroupBox("Guardrails Overview", panel)
        vl = QtWidgets.QVBoxLayout(group_list)
        self.list_guardrails = QtWidgets.QListWidget(group_list)
        self.list_guardrails.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vl.addWidget(self.list_guardrails)
        v.addWidget(group_list)

        group_edit = QtWidgets.QGroupBox("Selected Guardrail Editor (Modify mode)", panel)
        fg = QtWidgets.QFormLayout(group_edit)

        def mkspin():
            s = QtWidgets.QDoubleSpinBox(group_edit)
            s.setDecimals(3); s.setRange(-1_000_000.0, 1_000_000.0); s.setSingleStep(0.1)
            return s

        self.spin_guard_cx = mkspin()
        self.spin_guard_cy = mkspin()
        self.spin_guard_length_edit = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_guard_length_edit.setDecimals(3); self.spin_guard_length_edit.setRange(0.01, 1_000_000.0); self.spin_guard_length_edit.setSingleStep(0.1)
        self.spin_guard_yaw_edit = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_guard_yaw_edit.setDecimals(3); self.spin_guard_yaw_edit.setRange(-3600.0, 3600.0); self.spin_guard_yaw_edit.setSingleStep(1.0)

        fg.addRow("Center x", self.spin_guard_cx)
        fg.addRow("Center y", self.spin_guard_cy)
        fg.addRow("Length", self.spin_guard_length_edit)
        fg.addRow("Yaw (deg)", self.spin_guard_yaw_edit)
        v.addWidget(group_edit)
        self.group_guard_editor = group_edit

        # signals
        self.list_guardrails.itemSelectionChanged.connect(self._OnGuardrailSelectionChanged)
        self.spin_guard_cx.valueChanged.connect(self._OnGuardrailCenterChanged)
        self.spin_guard_cy.valueChanged.connect(self._OnGuardrailCenterChanged)
        self.spin_guard_length_edit.valueChanged.connect(self._OnGuardrailLengthChanged)
        self.spin_guard_yaw_edit.valueChanged.connect(self._OnGuardrailYawChanged)

        v.addStretch(1)
        return panel
    
    # =========================
    # 오른쪽 패널: Surrounding Vehicles
    # =========================
    def _BuildSurroundingVehiclesRightPanel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4); v.setSpacing(8)

        group_list = QtWidgets.QGroupBox("Surrounding Vehicles Overview", panel)
        vl = QtWidgets.QVBoxLayout(group_list)
        self.list_surrounding = QtWidgets.QListWidget(group_list)
        self.list_surrounding.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vl.addWidget(self.list_surrounding)
        v.addWidget(group_list)

        group_edit = QtWidgets.QGroupBox("Selected Surrounding Vehicle Editor (Modify mode)", panel)
        fo = QtWidgets.QFormLayout(group_edit)

        # 속도 편집
        self.spin_surrounding_speed_edit = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_surrounding_speed_edit.setDecimals(3); self.spin_surrounding_speed_edit.setRange(0.0, 1_000.0)
        self.spin_surrounding_speed_edit.setSingleStep(0.5)

        fo.addRow("speed_mps", self.spin_surrounding_speed_edit)

        # 동적 노드 영역
        self.group_surrounding_nodes = QtWidgets.QGroupBox("Nodes (x, y)")
        self.surrounding_nodes_layout = QtWidgets.QFormLayout(self.group_surrounding_nodes)
        fo.addRow(self.group_surrounding_nodes)

        v.addWidget(group_edit)
        self.group_surrounding_editor = group_edit

        # 시그널 연결
        self.list_surrounding.itemSelectionChanged.connect(self._OnSurroundingVehicleSelectionChanged)
        self.spin_surrounding_speed_edit.valueChanged.connect(self._OnSurroundingVehicleSpeedChanged)

        v.addStretch(1)
        return panel


    # 클래스 메서드로 완전 신규 추가
    def _BuildGnssShadowZonesRightPanel(self) -> QtWidgets.QWidget:  # ★ NEW
        panel = QtWidgets.QWidget(self)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(8)

        group_list = QtWidgets.QGroupBox("GNSS Shadow Zones Overview", panel)
        vl = QtWidgets.QVBoxLayout(group_list)
        self.list_gnss = QtWidgets.QListWidget(group_list)
        self.list_gnss.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        vl.addWidget(self.list_gnss)
        v.addWidget(group_list)

        group_edit = QtWidgets.QGroupBox("Selected GNSS Shadow Zone Editor (Move mode)", panel)
        fz = QtWidgets.QFormLayout(group_edit)

        def mkspin():
            s = QtWidgets.QDoubleSpinBox(group_edit)
            s.setDecimals(3); s.setRange(-1_000_000.0, 1_000_000.0); s.setSingleStep(0.1)
            return s

        self.spin_gnss_x = mkspin()
        self.spin_gnss_y = mkspin()
        self.spin_gnss_pos_std_edit = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_gnss_pos_std_edit.setDecimals(3); self.spin_gnss_pos_std_edit.setRange(0.0, 1_000_000.0); self.spin_gnss_pos_std_edit.setSingleStep(0.1)
        self.spin_gnss_radius_edit = QtWidgets.QDoubleSpinBox(group_edit)
        self.spin_gnss_radius_edit.setDecimals(3); self.spin_gnss_radius_edit.setRange(0.0, 1_000_000.0); self.spin_gnss_radius_edit.setSingleStep(0.1)

        fz.addRow("Center x", self.spin_gnss_x)
        fz.addRow("Center y", self.spin_gnss_y)
        fz.addRow("pos_std (m)", self.spin_gnss_pos_std_edit)
        fz.addRow("radius (m)", self.spin_gnss_radius_edit)
        v.addWidget(group_edit)
        self.group_gnss_editor = group_edit

        # 시그널
        self.list_gnss.itemSelectionChanged.connect(self._OnGnssShadowZoneSelectionChanged)
        self.spin_gnss_x.valueChanged.connect(self._OnGnssShadowZoneCenterChanged)
        self.spin_gnss_y.valueChanged.connect(self._OnGnssShadowZoneCenterChanged)
        self.spin_gnss_pos_std_edit.valueChanged.connect(self._OnGnssShadowZonePosStdChanged)
        self.spin_gnss_radius_edit.valueChanged.connect(self._OnGnssShadowZoneRadiusChanged)

        v.addStretch(1)
        return panel

    # =========================
    # 신호 연결
    # =========================
    def _ConnectSignals(self):
        # Bundle
        self.button_new_bundle.clicked.connect(self._OnNewBundle)
        self.button_open_bundle.clicked.connect(self._OnOpenBundle)
        self.button_save_bundle.clicked.connect(self._OnSaveBundle)
        # View
        self.button_fit_view.clicked.connect(self._OnFitView)
        self.toggle_lock_view.clicked.connect(self._OnToggleLock)
        self.toggle_common_view.clicked.connect(self._OnToggleCommonViewMode)

        # Create - Apply
        self.button_lane_apply.clicked.connect(self._OnLaneApplyCreate)

        # Mode
        self.radio_mode_lanes.toggled.connect(lambda c: c and self._OnSwitchMode("lanes"))
        self.radio_mode_buildings.toggled.connect(lambda c: c and self._OnSwitchMode("buildings"))
        self.radio_mode_guardrails.toggled.connect(lambda c: c and self._OnSwitchMode("guardrails"))
        self.radio_mode_signs.toggled.connect(lambda c: c and self._OnSwitchMode("signs"))
        self.radio_mode_traffic.toggled.connect(lambda c: c and self._OnSwitchMode("traffic_lights"))
        # (Mode 라디오 토글들 근처)
        self.radio_mode_gnss.toggled.connect(lambda c: c and self._OnSwitchMode("gnss_shadow_zones"))  # ★ NEW
        self.radio_mode_surrounding.toggled.connect(lambda c: c and self._OnSwitchMode("surrounding_vehicles"))

        # Modify mode 옵션
        self.radio_lane_modify_node.toggled.connect(
            lambda c: c and self._OnSetLaneModifyMode("node")
        )
        self.radio_lane_modify_lane.toggled.connect(
            lambda c: c and self._OnSetLaneModifyMode("lane")
        )
        
        # Delete mode 옵션
        self.radio_lane_delete_node.toggled.connect(
            lambda c: c and self._OnSetLaneDeleteMode("node")
        )
        self.radio_lane_delete_lane.toggled.connect(
            lambda c: c and self._OnSetLaneDeleteMode("lane")
        )

        # Lanes submodes - Modify 문자열로 바꿈
        self.button_lane_move.clicked.connect(lambda: self._OnSetLaneSubmode("modify"))

        # Lanes submodes
        self.button_lane_view.clicked.connect(lambda: self._OnSetLaneSubmode("view"))
        self.button_lane_create.clicked.connect(lambda: self._OnSetLaneSubmode("create"))
        self.button_lane_move.clicked.connect(lambda: self._OnSetLaneSubmode("move"))
        self.button_lane_delete.clicked.connect(lambda: self._OnSetLaneSubmode("delete"))
        # Lanes type quick
        # self.button_lane_type_driving.clicked.connect(lambda: self._OnSetLaneType("driving"))
        # self.button_lane_type_shoulder.clicked.connect(lambda: self._OnSetLaneType("shoulder"))
        # self.button_lane_type_bike.clicked.connect(lambda: self._OnSetLaneType("bike"))
        # self.button_lane_type_bus.clicked.connect(lambda: self._OnSetLaneType("bus"))
        # self.button_lane_type_parking.clicked.connect(lambda: self._OnSetLaneType("parking"))
        # Lanes type quick (enum 이름으로 연결)
        self.button_lane_type_unknown.clicked.connect(lambda: self._OnSetLaneType("TYPE_UNKNOWN"))
        self.button_lane_type_white_solid.clicked.connect(lambda: self._OnSetLaneType("TYPE_WHITE_SOLID"))
        self.button_lane_type_white_dashed.clicked.connect(lambda: self._OnSetLaneType("TYPE_WHITE_DASHED"))
        self.button_lane_type_yellow_solid.clicked.connect(lambda: self._OnSetLaneType("TYPE_YELLOW_SOLID"))
        self.button_lane_type_yellow_dashed.clicked.connect(lambda: self._OnSetLaneType("TYPE_YELLOW_DASHED"))
        self.button_lane_type_double_yellow.clicked.connect(lambda: self._OnSetLaneType("TYPE_DOUBLE_YELLOW_SOLID"))

        # Buildings submodes
        self.button_building_view.clicked.connect(lambda: self._OnSetBuildingSubmode("view"))
        self.button_building_create.clicked.connect(lambda: self._OnSetBuildingSubmode("create"))
        self.button_building_modify.clicked.connect(lambda: self._OnSetBuildingSubmode("modify"))
        self.button_building_delete.clicked.connect(lambda: self._OnSetBuildingSubmode("delete"))
        # Buildings create defaults
        self.spin_building_vertices_count.valueChanged.connect(lambda v: self.editor_manager.SetBuildingCreateDefaults(count=int(v)))
        self.spin_building_vertex_radius.valueChanged.connect(lambda v: self.editor_manager.SetBuildingCreateDefaults(radius=float(v)))
        self.spin_building_height.valueChanged.connect(lambda v: self.editor_manager.SetBuildingCreateDefaults(height=float(v)))

        # Lists
        self.list_lanes.itemSelectionChanged.connect(self._OnLaneSelectionChanged)


        # ★ Signs submodes
        self.button_sign_view.clicked.connect(lambda: self._OnSetSignSubmode("view"))
        self.button_sign_create.clicked.connect(lambda: self._OnSetSignSubmode("create"))
        self.button_sign_move.clicked.connect(lambda: self._OnSetSignSubmode("move"))
        self.button_sign_delete.clicked.connect(lambda: self._OnSetSignSubmode("delete"))

        # ★ Signs create defaults (에디터 매니저에 해당 API 없으면 직접 signs 에디터 필드에 세팅)
        def _apply_sign_create_defaults():
            # editor_signs.py에 current_sign_* 속성이 있으므로 직접 주입 (원본 파일 그대로여도 OK)
            signs_editor = self.editor_manager.editors.get("signs")
            if signs_editor is not None:
                signs_editor.current_sign_yaw = float(self.spin_sign_yaw.value())
                signs_editor.current_sign_z = float(self.spin_sign_z.value())
                # 기본은 이름 문자열로도 동작(원본 editor_signs는 문자열 사용). 코드값만 쓰고 싶으면 여기서 매핑해도 됨.
                data = self.combo_sign_class.currentData()
                signs_editor.current_sign_class = int(data) if data is not None else 0

        self.combo_sign_class.currentIndexChanged.connect(lambda _: _apply_sign_create_defaults())
        self.spin_sign_z.valueChanged.connect(lambda _: _apply_sign_create_defaults())
        self.spin_sign_yaw.valueChanged.connect(lambda _: _apply_sign_create_defaults())

        # ★ Traffic Lights submodes
        self.button_traffic_view.clicked.connect(lambda: self._OnSetTrafficLightsSubmode("view"))
        self.button_traffic_create.clicked.connect(lambda: self._OnSetTrafficLightsSubmode("create"))
        self.button_traffic_modify.clicked.connect(lambda: self._OnSetTrafficLightsSubmode("modify"))
        self.button_traffic_delete.clicked.connect(lambda: self._OnSetTrafficLightsSubmode("delete"))

        # ★ Traffic Lights Create Settings → 에디터에 바로 반영
        self.combo_traffic_state.currentIndexChanged.connect(self.ApplyTrafficLightCreateDefaults)
        self.spin_traffic_z.valueChanged.connect(self.ApplyTrafficLightCreateDefaults)

        # Guardrails submodes
        self.button_guard_view.clicked.connect(lambda: self._OnSetGuardSubmode("view"))
        self.button_guard_create.clicked.connect(lambda: self._OnSetGuardSubmode("create"))
        self.button_guard_modify.clicked.connect(lambda: self._OnSetGuardSubmode("modify"))
        self.button_guard_delete.clicked.connect(lambda: self._OnSetGuardSubmode("delete"))

        # Guardrails create defaults
        self.spin_guard_length.valueChanged.connect(lambda v: self.editor_manager.SetGuardrailsCreateDefaults(length=float(v)))
        self.spin_guard_yaw.valueChanged.connect(lambda v: self.editor_manager.SetGuardrailsCreateDefaults(yaw_deg=float(v)))

        # (GNSS 좌측 서브패널 버튼/스핀박스 시그널)
        self.button_gnss_view.clicked.connect(lambda: self._OnSetGnssShadowZonesSubmode("view"))     # ★ NEW
        self.button_gnss_create.clicked.connect(lambda: self._OnSetGnssShadowZonesSubmode("create")) # ★ NEW
        self.button_gnss_move.clicked.connect(lambda: self._OnSetGnssShadowZonesSubmode("move"))     # ★ NEW
        self.button_gnss_delete.clicked.connect(lambda: self._OnSetGnssShadowZonesSubmode("delete")) # ★ NEW
        self.spin_gnss_pos_std.valueChanged.connect(self.ApplyGnssShadowZonesCreateDefaults)          # ★ NEW
        self.spin_gnss_radius.valueChanged.connect(self.ApplyGnssShadowZonesCreateDefaults)           # ★ NEW

        # On comming vehicles
        self.button_surrounding_view.clicked.connect(lambda: self._OnSetSurroundingVehiclesSubmode("view"))
        self.button_surrounding_create.clicked.connect(lambda: self._OnSetSurroundingVehiclesSubmode("create"))
        self.button_surrounding_modify.clicked.connect(lambda: self._OnSetSurroundingVehiclesSubmode("modify"))
        self.button_surrounding_delete.clicked.connect(lambda: self._OnSetSurroundingVehiclesSubmode("delete"))
        # Apply
        self.button_surrounding_apply.clicked.connect(self._OnSurroundingVehicleApplyCreate)
        # Modify/Delete 라디오
        self.radio_surrounding_modify_node.toggled.connect(
            lambda c: c and self.editor_manager.SetSurroundingVehiclesModifyMode("node"))
        self.radio_surrounding_modify_vehicle.toggled.connect(
            lambda c: c and self.editor_manager.SetSurroundingVehiclesModifyMode("vehicle"))

        self.radio_surrounding_delete_node.toggled.connect(
            lambda c: c and self.editor_manager.SetSurroundingVehiclesDeleteMode("node"))
        self.radio_surrounding_delete_vehicle.toggled.connect(
            lambda c: c and self.editor_manager.SetSurroundingVehiclesDeleteMode("vehicle"))
        # Create Defaults
        self.spin_surrounding_speed.valueChanged.connect(lambda v: self.editor_manager.SetSurroundingVehiclesCreateDefaults(speed_mps=float(v)))



    # =========================
    # 좌측/우측 패널 스위치
    # =========================
    def _SwitchModeUi(self, mode: str):
        if mode == "lanes":
            self.left_stack.setCurrentIndex(0)
            self.right_stack.setCurrentIndex(0)
        elif mode == "buildings":
            self.left_stack.setCurrentIndex(1)
            self.right_stack.setCurrentIndex(1)
        elif mode == "signs":
            self.left_stack.setCurrentIndex(2)
            self.right_stack.setCurrentIndex(2)
        elif mode == "traffic_lights":  # ★ 추가
            self.left_stack.setCurrentIndex(3)
            self.right_stack.setCurrentIndex(3)
        elif mode == "guardrails":
            self.left_stack.setCurrentWidget(self.guardrails_left)
            self.right_stack.setCurrentWidget(self.guardrails_right)
        elif mode == "gnss_shadow_zones":                              # ★ NEW
            self.left_stack.setCurrentWidget(self.gnss_left)
            self.right_stack.setCurrentWidget(self.gnss_right)
        elif mode == "surrounding_vehicles":
            # 좌/우 모두 surrounding 페이지로
            # 좌측은 추가된 surrounding_page가 left_stack의 '마지막' 인덱스이므로 index 계산 필요 시 find 사용 or 직접 인덱스 추적
            # 여기서는 간단히:
            self.left_stack.setCurrentWidget(self.surrounding_left)
            self.right_stack.setCurrentWidget(self.surrounding_right)
        else:
            # 다른 모드는 아직 공통 UI만 표시
            self.left_stack.setCurrentIndex(0)
            self.right_stack.setCurrentIndex(0)
        

    # =========================
    # Lanes: 목록 / 폼
    # =========================
    def RefreshLaneList(self):
        prev_focus = self.focusWidget()
        prev_row = self.list_lanes.currentRow()
        prev_scroll = self.list_lanes.verticalScrollBar().value()

        summaries = self.editor_manager.GetLanesSummary()  # (lane_index, lane_id, lane_type, num_nodes)
        self.list_lanes.blockSignals(True)
        self.list_lanes.clear()
        for lane_index, lane_id, lane_type, num_nodes in summaries:
            txt = f"[{lane_id}] {lane_type}: {num_nodes} nodes"
            it = QtWidgets.QListWidgetItem(txt)
            it.setData(QtCore.Qt.UserRole, lane_index)
            self.list_lanes.addItem(it)
        self.list_lanes.blockSignals(False)

        if 0 <= prev_row < self.list_lanes.count():
            self.list_lanes.setCurrentRow(prev_row)
        self.list_lanes.verticalScrollBar().setValue(prev_scroll)

        if prev_focus is not None:
            prev_focus.setFocus(QtCore.Qt.OtherFocusReason)
    
    def _OnLaneNodeChanged(self, node_index, spin_x, spin_y):
        if self.is_loading_lane_fields:
            return
        ok, msg = self.editor_manager.UpdateSelectedLaneNode(node_index, spin_x.value(), spin_y.value())
        for ed in self.editor_manager.editors.values():
            ed.Redraw()
        self.canvas.draw_idle()
        if msg:
            self._SetStatus(msg)
    
    def _RebuildLaneNodeEditors(self, points_xyz):
        # 기존 제거
        while self.nodes_layout.rowCount():
            self.nodes_layout.removeRow(0)
        self.lane_node_spins = []  # [(sx, sy), ...]

        for i, (px, py, _pz) in enumerate(points_xyz):
            spin_x = QtWidgets.QDoubleSpinBox(self.group_lane_nodes)
            spin_x.setDecimals(3); spin_x.setRange(-1_000_000.0, 1_000_000.0); spin_x.setSingleStep(0.1); spin_x.setValue(float(px))
            spin_y = QtWidgets.QDoubleSpinBox(self.group_lane_nodes)
            spin_y.setDecimals(3); spin_y.setRange(-1_000_000.0, 1_000_000.0); spin_y.setSingleStep(0.1); spin_y.setValue(float(py))

            # 값 변경 → 노드 좌표 갱신
            spin_x.valueChanged.connect(lambda _v, idx=i, sx=spin_x, sy=spin_y: self._OnLaneNodeChanged(idx, sx, sy))
            spin_y.valueChanged.connect(lambda _v, idx=i, sx=spin_x, sy=spin_y: self._OnLaneNodeChanged(idx, sx, sy))

            roww = QtWidgets.QWidget(self.group_lane_nodes)
            rowl = QtWidgets.QHBoxLayout(roww); rowl.setContentsMargins(0,0,0,0)
            rowl.addWidget(spin_x); rowl.addWidget(spin_y)
            self.nodes_layout.addRow(f"node {i}", roww)
            self.lane_node_spins.append((spin_x, spin_y))

    def LoadLaneEditorFields(self):
        self.is_loading_lane_fields = True
        details = self.editor_manager.GetSelectedLaneDetails()

        # 선택된 lane이 없을 때: 초기화
        if not details:
            # 노드 에디터 비우기
            if hasattr(self, "_RebuildLaneNodeEditors"):
                self._RebuildLaneNodeEditors([])
            # 타입 콤보 초기화 (없으면 0번째로)
            if hasattr(self, "combo_lane_type"):
                idx_unknown = self.combo_lane_type.findText("TYPE_UNKNOWN")
                if idx_unknown >= 0:
                    self.combo_lane_type.setCurrentIndex(idx_unknown)
                else:
                    self.combo_lane_type.setCurrentIndex(0)
            self._UpdateLaneEditorEnabled()
            self.is_loading_lane_fields = False
            return

        # lane 타입 이름은 lane_type_name(신규) 또는 lane_type(구버전 문자열) 키로 올 수 있음
        lane_type_name = details.get("lane_type_name") or details.get("lane_type") or "TYPE_UNKNOWN"
        if hasattr(self, "combo_lane_type"):
            idx = self.combo_lane_type.findText(lane_type_name)
            if idx >= 0:
                self.combo_lane_type.setCurrentIndex(idx)

        # 모든 노드 스핀박스 재구성 (매니저에 함수가 있다면 활용)
        points = []
        if hasattr(self.editor_manager, "GetSelectedLaneAllPoints"):
            points = self.editor_manager.GetSelectedLaneAllPoints()
        if hasattr(self, "_RebuildLaneNodeEditors"):
            self._RebuildLaneNodeEditors(points)

        self._UpdateLaneEditorEnabled()
        self.is_loading_lane_fields = False

    def _OnLaneSpinChanged(self, *_):
        if self.is_loading_lane_fields:
            return
        is_move = (self.editor_manager.editor_state.mode == "lanes" and
                   self.editor_manager.GetLaneSubmode() == "modify" and
                   self.editor_manager.GetSelectedLaneIndex() is not None)
        if not is_move: return
        start_xy = (self.spin_start_x.value(), self.spin_start_y.value())
        end_xy   = (self.spin_end_x.value(),   self.spin_end_y.value())
        ok, msg = self.editor_manager.UpdateSelectedLanePoints(start_xy, end_xy)
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if ok:
            self.editor_manager.TriggerLanesChanged()
        if msg: self._SetStatus(msg)
        sender = self.sender()
        if isinstance(sender, QtWidgets.QAbstractSpinBox):
            sender.setFocus(QtCore.Qt.OtherFocusReason)

    # def _OnLaneTypeChanged(self, text: str):
    #     if self.is_loading_lane_fields:
    #         return
    #     is_move = (self.editor_manager.editor_state.mode == "lanes" and
    #                self.editor_manager.GetLaneSubmode() == "move" and
    #                self.editor_manager.GetSelectedLaneIndex() is not None)
    #     if not is_move: return
    #     ok, msg = self.editor_manager.UpdateSelectedLaneType(text)
    #     for ed in self.editor_manager.editors.values(): ed.Redraw()
    #     self.canvas.draw_idle()
    #     if ok:
    #         self.editor_manager.TriggerLanesChanged()
    #     if msg: self._SetStatus(msg)
    #     sender = self.sender()
    #     if isinstance(sender, QtWidgets.QComboBox):
    #         sender.setFocus(QtCore.Qt.OtherFocusReason)
    # _OnLaneTypeChanged()는 그대로 '문자열 이름'을 넘기면 됨
    def _OnLaneTypeChanged(self, text: str):
        if self.is_loading_lane_fields:
            return
        is_modify = (self.editor_manager.editor_state.mode == "lanes" and
                    self.editor_manager.GetLaneSubmode() == "modify" and
                    self.editor_manager.GetSelectedLaneIndex() is not None)
        if not is_modify: return
        ok, msg = self.editor_manager.UpdateSelectedLaneType(text)  # ← 이름 문자열
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if ok:
            self.editor_manager.TriggerLanesChanged()
        if msg: self._SetStatus(msg)

    def _OnLaneSelectionChanged(self):
        items = self.list_lanes.selectedItems()
        if not items:
            self.editor_manager.SelectLaneByIndex(None)
        else:
            lane_index = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.SelectLaneByIndex(int(lane_index))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadLaneEditorFields()
    
    #===== Lane 적용/모드 핸들러 메서드 추가
    def _OnLaneApplyCreate(self):
        ok, msg = self.editor_manager.CommitCurrentLaneCreation()
        for ed in self.editor_manager.editors.values():
            ed.Redraw()
        self.canvas.draw_idle()
        self.RefreshLaneList()
        if msg:
            self._SetStatus(msg)
    
    def _OnLanesListOrGeometryChanged(self):
        self.RefreshLaneList()
        self.LoadLaneEditorFields()

    def _OnSetLaneModifyMode(self, which: str):
        self.editor_manager.SetLaneModifyMode(which)
        self._SetStatus(f"Modify mode -> {which}")

    def _OnSetLaneDeleteMode(self, which: str):
        self.editor_manager.SetLaneDeleteMode(which)
        self._SetStatus(f"Delete mode -> {which}")
    #=====

    def _UpdateLaneEditorEnabled(self):
        has_sel = self.editor_manager.GetSelectedLaneIndex() is not None
        is_move = (self.editor_manager.editor_state.mode == "lanes" and
                   self.editor_manager.GetLaneSubmode() == "modify")
        self.group_lane_editor.setEnabled(bool(has_sel and is_move))

    # =========================
    # Buildings: 목록 / 폼
    # =========================
    def RefreshBuildingList(self):
        self.list_buildings.blockSignals(True)
        self.list_buildings.clear()
        for index, bid, cnt, h in self.editor_manager.GetBuildingsSummary():
            txt = f"[{bid}] {cnt} points, h = {h:.2f} building"
            it = QtWidgets.QListWidgetItem(txt)
            it.setData(QtCore.Qt.UserRole, index)
            self.list_buildings.addItem(it)
        self.list_buildings.blockSignals(False)
        sel = self.editor_manager.GetSelectedBuildingIndex()
        if sel is not None:
            for i in range(self.list_buildings.count()):
                if self.list_buildings.item(i).data(QtCore.Qt.UserRole) == sel:
                    self.list_buildings.setCurrentRow(i)
                    break

    def LoadBuildingEditorFields(self):
        self.is_loading_building_fields = True
        det = self.editor_manager.GetSelectedBuildingDetails()
        if not det:
            self._RebuildBuildingVertexEditors([])
            self._UpdateBuildingEditorEnabled()
            self.is_loading_building_fields = False
            return
        cx, cy = det["center_xy"]
        self.spin_center_x.setValue(float(cx)); self.spin_center_y.setValue(float(cy))
        self.spin_rotation_deg.setValue(float(det["rotation_deg"]))
        self.spin_height.setValue(float(det["height"]))
        self._RebuildBuildingVertexEditors(det["vertices_xy"])
        self._UpdateBuildingEditorEnabled()
        self.is_loading_building_fields = False
    

    def _RebuildBuildingVertexEditors(self, verts):
        # 기존 제거
        while self.vertices_layout.rowCount():
            self.vertices_layout.removeRow(0)
        self.vertex_spins = []
        for i, (vx, vy) in enumerate(verts):
            spin_x = QtWidgets.QDoubleSpinBox(); spin_x.setDecimals(3); spin_x.setRange(-1_000_000.0, 1_000_000.0); spin_x.setSingleStep(0.1); spin_x.setValue(float(vx))
            spin_y = QtWidgets.QDoubleSpinBox(); spin_y.setDecimals(3); spin_y.setRange(-1_000_000.0, 1_000_000.0); spin_y.setSingleStep(0.1); spin_y.setValue(float(vy))
            spin_x.valueChanged.connect(lambda _v, idx=i, sx=spin_x, sy=spin_y: self._OnBuildingVertexChanged(idx, sx, sy))
            spin_y.valueChanged.connect(lambda _v, idx=i, sx=spin_x, sy=spin_y: self._OnBuildingVertexChanged(idx, sx, sy))
            roww = QtWidgets.QWidget()
            rowl = QtWidgets.QHBoxLayout(roww); rowl.setContentsMargins(0,0,0,0)
            rowl.addWidget(spin_x); rowl.addWidget(spin_y)
            self.vertices_layout.addRow(f"v{i}", roww)
            self.vertex_spins.append((spin_x, spin_y))

    def _OnBuildingSelectionChanged(self):
        items = self.list_buildings.selectedItems()
        if not items:
            self.editor_manager.SelectBuildingByIndex(None)
        else:
            idx = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.SelectBuildingByIndex(int(idx))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadBuildingEditorFields()

    def _OnBuildingFieldsChanged(self, *_):
        if self.is_loading_building_fields:
            return
        det = self.editor_manager.GetSelectedBuildingDetails()
        if not det: return
        cx = self.spin_center_x.value(); cy = self.spin_center_y.value()
        self.editor_manager.UpdateSelectedBuildingCenter((cx, cy))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnBuildingRotationChanged(self, *_):
        if self.is_loading_building_fields:
            return
        det = self.editor_manager.GetSelectedBuildingDetails()
        if not det: return
        self.editor_manager.UpdateSelectedBuildingRotation(self.spin_rotation_deg.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnBuildingHeightChanged(self, *_):
        if self.is_loading_building_fields:
            return
        det = self.editor_manager.GetSelectedBuildingDetails()
        if not det: return
        self.editor_manager.UpdateSelectedBuildingHeight(self.spin_height.value())
        self.RefreshBuildingList()

    def _OnBuildingVertexChanged(self, vertex_index, sx, sy):
        if self.is_loading_building_fields:
            return
        det = self.editor_manager.GetSelectedBuildingDetails()
        if not det: return
        self.editor_manager.UpdateSelectedBuildingVertex(vertex_index, (sx.value(), sy.value()))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _UpdateBuildingEditorEnabled(self):
        has_sel = self.editor_manager.GetSelectedBuildingIndex() is not None
        is_modify = (self.editor_manager.editor_state.mode == "buildings" and
                     self.editor_manager.GetBuildingSubmode() == "modify")
        self.group_building_editor.setEnabled(bool(has_sel and is_modify))

    # =========================    # Signs: 목록 / 폼
    def RefreshSignList(self):
        # editor_manager에 전용 API가 없어도 editor_state 직접 읽으면 됨
        signs = getattr(self.editor_manager.editor_state, "signs", [])
        self.list_signs.blockSignals(True)
        self.list_signs.clear()
        for i, s in enumerate(signs):
            txt = f"({s.x:.2f},{s.y:.2f}), z={s.z:.2f}, yaw={s.yaw:.1f}, class={getattr(s, 'sign_class', '?')}"
            it = QtWidgets.QListWidgetItem(txt)
            it.setData(QtCore.Qt.UserRole, i)
            self.list_signs.addItem(it)
        self.list_signs.blockSignals(False)

    def LoadSignEditorFields(self):
        self.is_loading_sign_fields = True
        idx = getattr(self.editor_manager.editor_state, "selected_sign_index", None)
        signs = getattr(self.editor_manager.editor_state, "signs", [])
        if idx is None or not (0 <= idx < len(signs)):
            for s in [self.spin_sign_x, self.spin_sign_y, self.spin_sign_z_edit, self.spin_sign_yaw_edit]:
                s.setValue(0.0)
            self.combo_sign_class_edit.setCurrentIndex(0)
            self._UpdateSignEditorEnabled()
            self.is_loading_sign_fields = False
            return
        s = signs[idx]
        self.spin_sign_x.setValue(float(s.x))
        self.spin_sign_y.setValue(float(s.y))
        self.spin_sign_z_edit.setValue(float(s.z))
        self.spin_sign_yaw_edit.setValue(float(s.yaw))
        # class 콤보는 문자열/정수 둘 다 커버 (원본은 문자열임)
        target = s.sign_class
        # 찾기: 이름과 코드 둘 다 시도
        found = False
        for i in range(self.combo_sign_class_edit.count()):
            if self.combo_sign_class_edit.itemText(i) == str(target) or self.combo_sign_class_edit.itemData(i) == target:
                self.combo_sign_class_edit.setCurrentIndex(i)
                found = True
                break
        if not found:
            self.combo_sign_class_edit.setCurrentIndex(0)
        self._UpdateSignEditorEnabled()
        self.is_loading_sign_fields = False

    def _OnSignSelectionChanged(self):
        items = self.list_signs.selectedItems()
        if not items:
            self.editor_manager.editor_state.selected_sign_index = None
        else:
            idx = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.editor_state.selected_sign_index = int(idx)
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadSignEditorFields()
    
    def _OnSignFieldsChanged(self, *_):
        if self.is_loading_sign_fields: return
        is_modify = (self.editor_manager.editor_state.mode == "signs" and
                    self.editor_manager.editor_state.signs_submode == "move" and
                    self.editor_manager.editor_state.selected_sign_index is not None)
        if not is_modify: return
        idx = self.editor_manager.editor_state.selected_sign_index
        s = self.editor_manager.editor_state.signs[idx]
        s.x = float(self.spin_sign_x.value())
        s.y = float(self.spin_sign_y.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnSignHeightChanged(self, *_):
        if self.is_loading_sign_fields: return
        idx = self.editor_manager.editor_state.selected_sign_index
        if idx is None: return
        self.editor_manager.editor_state.signs[idx].z = float(self.spin_sign_z_edit.value())

    def _OnSignYawChanged(self, *_):
        if self.is_loading_sign_fields: return
        idx = self.editor_manager.editor_state.selected_sign_index
        if idx is None: return
        self.editor_manager.editor_state.signs[idx].yaw = float(self.spin_sign_yaw_edit.value())

    def _OnSignClassChanged(self, *_):
        if self.is_loading_sign_fields: return
        idx = self.editor_manager.editor_state.selected_sign_index
        if idx is None: return
        # 원본 editor_signs.py는 문자열 클래스도 허용하므로 이름을 기본 저장
        self.editor_manager.editor_state.signs[idx].sign_class = self.combo_sign_class_edit.currentText()
    
    def _UpdateSignEditorEnabled(self):
        has_sel = self.editor_manager.GetSelectedSignIndex() is not None
        is_modify = (self.editor_manager.editor_state.mode == "signs" and
                     self.editor_manager.GetSignSubmode() == "move")
        self.group_sign_editor.setEnabled(bool(has_sel and is_modify))
    


    # =========================    # Traffic Lights: 목록 / 폼
    # =========================
    def RefreshTrafficLightList(self):
        traffic = getattr(self.editor_manager.editor_state, "traffic_lights", [])
        self.list_traffic.blockSignals(True)
        self.list_traffic.clear()
        for i, t in enumerate(traffic):
            txt = f"({t.x:.2f},{t.y:.2f}), z={t.z:.2f}, state={int(t.state)}"
            it = QtWidgets.QListWidgetItem(txt)
            it.setData(QtCore.Qt.UserRole, i)
            self.list_traffic.addItem(it)
        self.list_traffic.blockSignals(False)

    def LoadTrafficLightEditorFields(self):
        self.is_loading_traffic_fields = True
        idx = getattr(self.editor_manager.editor_state, "selected_traffic_light_index", None)
        items = getattr(self.editor_manager.editor_state, "traffic_lights", [])
        if idx is None or not (0 <= idx < len(items)):
            for s in [self.spin_traffic_x, self.spin_traffic_y, self.spin_traffic_z_edit]:
                s.setValue(0.0)
            self.combo_traffic_state_edit.setCurrentIndex(0)
            self._UpdateTrafficLightEditorEnabled()
            self.is_loading_traffic_fields = False
            return
        t = items[idx]
        self.spin_traffic_x.setValue(float(t.x))
        self.spin_traffic_y.setValue(float(t.y))
        self.spin_traffic_z_edit.setValue(float(t.z))
        target_code = int(t.state)
        pos = self.combo_traffic_state_edit.findData(target_code)
        self.combo_traffic_state_edit.setCurrentIndex(pos if pos >= 0 else 0)
        self._UpdateTrafficLightEditorEnabled()
        self.is_loading_traffic_fields = False

    def _OnTrafficLightSelectionChanged(self):
        items = self.list_traffic.selectedItems()
        if not items:
            self.editor_manager.editor_state.selected_traffic_light_index = None
        else:
            idx = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.editor_state.selected_traffic_light_index = int(idx)
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadTrafficLightEditorFields()

    def _OnTrafficLightCenterChanged(self, *_):
        if self.is_loading_traffic_fields: return
        is_modify = (self.editor_manager.editor_state.mode == "traffic_lights" and
                    getattr(self.editor_manager.editor_state, "traffic_lights_submode", "view") == "modify" and
                    self.editor_manager.editor_state.selected_traffic_light_index is not None)
        if not is_modify: return
        idx = self.editor_manager.editor_state.selected_traffic_light_index
        items = self.editor_manager.editor_state.traffic_lights
        if not (0 <= idx < len(items)): return
        t = items[idx]
        t.x = float(self.spin_traffic_x.value())
        t.y = float(self.spin_traffic_y.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnTrafficLightHeightChanged(self, *_):
        if self.is_loading_traffic_fields: return
        idx = self.editor_manager.editor_state.selected_traffic_light_index
        if idx is None: return
        items = self.editor_manager.editor_state.traffic_lights
        if not (0 <= idx < len(items)): return
        items[idx].z = float(self.spin_traffic_z_edit.value())

    def _OnTrafficLightStateChanged(self, *_):
        if self.is_loading_traffic_fields: return
        idx = self.editor_manager.editor_state.selected_traffic_light_index
        if idx is None: return
        items = self.editor_manager.editor_state.traffic_lights
        if not (0 <= idx < len(items)): return
        code = int(self.combo_traffic_state_edit.currentData())
        items[idx].state = code

    def _UpdateTrafficLightEditorEnabled(self):
        has_sel = getattr(self.editor_manager.editor_state, "selected_traffic_light_index", None) is not None
        is_modify = (self.editor_manager.editor_state.mode == "traffic_lights" and
                    getattr(self.editor_manager.editor_state, "traffic_lights_submode", "view") == "modify")
        self.group_traffic_editor.setEnabled(bool(has_sel and is_modify))

    def _OnSetTrafficLightsSubmode(self, submode: str):
        if hasattr(self.editor_manager, "SetTrafficLightsSubmode"):
            self.editor_manager.SetTrafficLightsSubmode(submode)
        else:
            self.editor_manager.editor_state.traffic_lights_submode = submode
            self.editor_manager.editors["traffic_lights"].Redraw()
        self.UpdateTrafficLightSubmodeButtons(submode)
        self._UpdateTrafficLightEditorEnabled()
        self._SetStatus(f"Traffic Lights submode -> {submode}")

    def UpdateTrafficLightSubmodeButtons(self, submode: str):
        m = {"view": self.button_traffic_view, "create": self.button_traffic_create,
            "modify": self.button_traffic_modify, "delete": self.button_traffic_delete}
        for k, b in m.items():
            b.setChecked(k == submode)

    def ApplyTrafficLightCreateDefaults(self):
        # 중첩함수 대신 클래스 메서드로 제공
        editor_obj = self.editor_manager.editors.get("traffic_lights")
        if editor_obj is not None:
            if hasattr(editor_obj, "SetCreateDefaults"):
                editor_obj.SetCreateDefaults(
                    state_code=int(self.combo_traffic_state.currentData()),
                    z=float(self.spin_traffic_z.value()),
                )
            else:
                # 구형 에디터 호환: 필드 직접 주입
                if hasattr(editor_obj, "current_state"):
                    editor_obj.current_state = {0:"red", 1:"yellow", 2:"green"}[int(self.combo_traffic_state.currentData())]
                if hasattr(editor_obj, "current_z"):
                    editor_obj.current_z = float(self.spin_traffic_z.value())

    # =========================    # Guard Lights: 목록 / 폼
    # =========================
    def UpdateGuardSubmodeButtons(self, submode: str):
        m = {"view": self.button_guard_view, "create": self.button_guard_create,
            "modify": self.button_guard_modify, "delete": self.button_guard_delete}
        for k, b in m.items():
            b.setChecked(k == submode)

    def _OnSetGuardSubmode(self, submode: str):
        self.editor_manager.SetGuardrailsSubmode(submode)
        self.UpdateGuardSubmodeButtons(submode)
        self._UpdateGuardrailEditorEnabled()
        self._SetStatus(f"Guardrails submode -> {submode}")
        self.canvas.setFocus()

    

    def ApplyGnssShadowZonesCreateDefaults(self):  # ★ NEW
        editor_obj = self.editor_manager.editors.get("gnss_shadow_zones")
        if editor_obj is not None and hasattr(editor_obj, "SetCreateDefaults"):
            editor_obj.SetCreateDefaults(
                pos_std=float(self.spin_gnss_pos_std.value()),
                radius=float(self.spin_gnss_radius.value()),
            )

    # =========================    # Traffic Lights: 목록 / 폼
    # =========================
    def RefreshGuardrailList(self):
        self.list_guardrails.blockSignals(True)
        self.list_guardrails.clear()
        for index, center_xy, length, yaw_deg in self.editor_manager.GetGuardrailsSummary():
            txt = f"[{index}] center=({center_xy[0]:.2f},{center_xy[1]:.2f})  L={length:.2f}  r={yaw_deg:.1f}"
            it = QtWidgets.QListWidgetItem(txt)
            it.setData(QtCore.Qt.UserRole, index)
            self.list_guardrails.addItem(it)
        self.list_guardrails.blockSignals(False)

        sel = self.editor_manager.GetSelectedGuardrailIndex()
        if sel is not None:
            for i in range(self.list_guardrails.count()):
                if self.list_guardrails.item(i).data(QtCore.Qt.UserRole) == sel:
                    self.list_guardrails.setCurrentRow(i)
                    break

    def LoadGuardrailEditorFields(self):
        det = self.editor_manager.GetSelectedGuardrailDetails()
        # 로딩 중 플래그(불필요하면 삭제해도 무방)
        self.is_loading_guard_fields = True
        if not det:
            for s in [self.spin_guard_cx, self.spin_guard_cy, self.spin_guard_length_edit, self.spin_guard_yaw_edit]:
                s.blockSignals(True); s.setValue(0.0); s.blockSignals(False)
            self._UpdateGuardrailEditorEnabled()
            self.is_loading_guard_fields = False
            return

        (cx, cy) = det["center_xy"]
        self.spin_guard_cx.blockSignals(True); self.spin_guard_cx.setValue(float(cx)); self.spin_guard_cx.blockSignals(False)
        self.spin_guard_cy.blockSignals(True); self.spin_guard_cy.setValue(float(cy)); self.spin_guard_cy.blockSignals(False)
        self.spin_guard_length_edit.blockSignals(True); self.spin_guard_length_edit.setValue(float(det["length"])); self.spin_guard_length_edit.blockSignals(False)
        self.spin_guard_yaw_edit.blockSignals(True); self.spin_guard_yaw_edit.setValue(float(det["yaw_deg"])); self.spin_guard_yaw_edit.blockSignals(False)
        self._UpdateGuardrailEditorEnabled()
        self.is_loading_guard_fields = False

    def _UpdateGuardrailEditorEnabled(self):
        has_sel = self.editor_manager.GetSelectedGuardrailIndex() is not None
        is_modify = (self.editor_manager.editor_state.mode == "guardrails" and
                    self.editor_manager.GetGuardrailsSubmode() == "modify")
        self.group_guard_editor.setEnabled(bool(has_sel and is_modify))

    def _OnGuardrailSelectionChanged(self):
        items = self.list_guardrails.selectedItems()
        if not items:
            self.editor_manager.SelectGuardrailByIndex(None)
        else:
            idx = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.SelectGuardrailByIndex(int(idx))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadGuardrailEditorFields()

    def _OnGuardrailCenterChanged(self, *_):
        if getattr(self, "is_loading_guard_fields", False):
            return
        det = self.editor_manager.GetSelectedGuardrailDetails()
        if not det: return
        ok, msg = self.editor_manager.UpdateSelectedGuardrailCenter(self.spin_guard_cx.value(), self.spin_guard_cy.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if ok and hasattr(self.editor_manager, "TriggerGuardrailsChanged"):
            self.editor_manager.TriggerGuardrailsChanged()
        if msg: self._SetStatus(msg)

    def _OnGuardrailLengthChanged(self, *_):
        if getattr(self, "is_loading_guard_fields", False):
            return
        det = self.editor_manager.GetSelectedGuardrailDetails()
        if not det: return
        ok, msg = self.editor_manager.UpdateSelectedGuardrailLength(self.spin_guard_length_edit.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if ok and hasattr(self.editor_manager, "TriggerGuardrailsChanged"):
            self.editor_manager.TriggerGuardrailsChanged()
        if msg: self._SetStatus(msg)

    def _OnGuardrailYawChanged(self, *_):
        if getattr(self, "is_loading_guard_fields", False):
            return
        det = self.editor_manager.GetSelectedGuardrailDetails()
        if not det: return
        ok, msg = self.editor_manager.UpdateSelectedGuardrailYaw(self.spin_guard_yaw_edit.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if ok and hasattr(self.editor_manager, "TriggerGuardrailsChanged"):
            self.editor_manager.TriggerGuardrailsChanged()
        if msg: self._SetStatus(msg)


    # =========================
    # GNSS
    # =========================
    def RefreshGnssShadowZoneList(self):  # ★ NEW
        items = getattr(self.editor_manager.editor_state, "gnss_shadow_zones", [])
        self.list_gnss.blockSignals(True)
        self.list_gnss.clear()
        for i, z in enumerate(items):
            txt = f"[{i}] ({z.x:.2f},{z.y:.2f}), σ={z.pos_std:.2f}, r={z.radius:.2f}"
            it = QtWidgets.QListWidgetItem(txt); it.setData(QtCore.Qt.UserRole, i)
            self.list_gnss.addItem(it)
        self.list_gnss.blockSignals(False)

    def LoadGnssShadowZoneEditorFields(self):  # ★ NEW
        self.is_loading_gnss_fields = True
        idx = getattr(self.editor_manager.editor_state, "selected_gnss_shadow_zone_index", None)
        items = getattr(self.editor_manager.editor_state, "gnss_shadow_zones", [])
        if idx is None or not (0 <= idx < len(items)):
            for s in [self.spin_gnss_x, self.spin_gnss_y, self.spin_gnss_pos_std_edit, self.spin_gnss_radius_edit]:
                s.setValue(0.0)
            self._UpdateGnssShadowZoneEditorEnabled()
            self.is_loading_gnss_fields = False
            return
        z = items[idx]
        self.spin_gnss_x.setValue(float(z.x))
        self.spin_gnss_y.setValue(float(z.y))
        self.spin_gnss_pos_std_edit.setValue(float(z.pos_std))
        self.spin_gnss_radius_edit.setValue(float(z.radius))
        self._UpdateGnssShadowZoneEditorEnabled()
        self.is_loading_gnss_fields = False

    def _OnGnssListOrGeometryChanged(self):
        self.RefreshGnssShadowZoneList()
        self.LoadGnssShadowZoneEditorFields()

    def _OnGnssShadowZoneSelectionChanged(self):  # ★ NEW
        items = self.list_gnss.selectedItems()
        if not items:
            self.editor_manager.SelectGnssShadowZoneByIndex(None)
        else:
            idx = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.SelectGnssShadowZoneByIndex(int(idx))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadGnssShadowZoneEditorFields()

    def _OnGnssShadowZoneCenterChanged(self, *_):  # ★ NEW
        if self.is_loading_gnss_fields: return
        is_move = (self.editor_manager.editor_state.mode == "gnss_shadow_zones" and
                self.editor_manager.GetGnssShadowZonesSubmode() == "move" and
                self.editor_manager.GetSelectedGnssShadowZoneIndex() is not None)
        if not is_move: return
        idx = self.editor_manager.GetSelectedGnssShadowZoneIndex()
        items = getattr(self.editor_manager.editor_state, "gnss_shadow_zones", [])
        if not (0 <= idx < len(items)): return
        z = items[idx]
        z.x = float(self.spin_gnss_x.value())
        z.y = float(self.spin_gnss_y.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnGnssShadowZonePosStdChanged(self, *_):  # ★ NEW
        if self.is_loading_gnss_fields: return
        idx = self.editor_manager.GetSelectedGnssShadowZoneIndex()
        if idx is None: return
        items = getattr(self.editor_manager.editor_state, "gnss_shadow_zones", [])
        if not (0 <= idx < len(items)): return
        items[idx].pos_std = float(self.spin_gnss_pos_std_edit.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnGnssShadowZoneRadiusChanged(self, *_):  # ★ NEW
        if self.is_loading_gnss_fields: return
        idx = self.editor_manager.GetSelectedGnssShadowZoneIndex()
        if idx is None: return
        items = getattr(self.editor_manager.editor_state, "gnss_shadow_zones", [])
        if not (0 <= idx < len(items)): return
        items[idx].radius = float(self.spin_gnss_radius_edit.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _UpdateGnssShadowZoneEditorEnabled(self):  # ★ NEW
        has_sel = self.editor_manager.GetSelectedGnssShadowZoneIndex() is not None
        is_move = (self.editor_manager.editor_state.mode == "gnss_shadow_zones" and
                self.editor_manager.GetGnssShadowZonesSubmode() == "move")
        self.group_gnss_editor.setEnabled(bool(has_sel and is_move))

    # =========================
    # Surrounding Vehicles: 목록 / 폼
    # =========================
    def RefreshSurroundingVehicleList(self):
        items = getattr(self.editor_manager, "GetSurroundingVehiclesSummary", lambda: [])()
        prev_row = self.list_surrounding.currentRow()
        prev_scroll = self.list_surrounding.verticalScrollBar().value()

        self.list_surrounding.blockSignals(True)
        self.list_surrounding.clear()
        for vehicle_index, vehicle_id, node_count, speed_mps in items:
            txt = f"[{vehicle_id}] {node_count} points, v={float(speed_mps):.2f} mps"
            it = QtWidgets.QListWidgetItem(txt)
            it.setData(QtCore.Qt.UserRole, vehicle_index)
            self.list_surrounding.addItem(it)
        self.list_surrounding.blockSignals(False)

        if 0 <= prev_row < self.list_surrounding.count():
            self.list_surrounding.setCurrentRow(prev_row)
        self.list_surrounding.verticalScrollBar().setValue(prev_scroll)

    def _RebuildSurroundingVehicleNodeEditors(self, points_xy):
        while self.surrounding_nodes_layout.rowCount():
            self.surrounding_nodes_layout.removeRow(0)
        self.surrounding_node_spins = []
        for i, (px, py) in enumerate(points_xy):
            spin_x = QtWidgets.QDoubleSpinBox(self.group_surrounding_nodes)
            spin_x.setDecimals(3); spin_x.setRange(-1_000_000.0, 1_000_000.0); spin_x.setSingleStep(0.1); spin_x.setValue(float(px))
            spin_y = QtWidgets.QDoubleSpinBox(self.group_surrounding_nodes)
            spin_y.setDecimals(3); spin_y.setRange(-1_000_000.0, 1_000_000.0); spin_y.setSingleStep(0.1); spin_y.setValue(float(py))
            spin_x.valueChanged.connect(lambda _v, idx=i, sx=spin_x, sy=spin_y: self._OnSurroundingVehicleNodeChanged(idx, sx, sy))
            spin_y.valueChanged.connect(lambda _v, idx=i, sx=spin_x, sy=spin_y: self._OnSurroundingVehicleNodeChanged(idx, sx, sy))
            roww = QtWidgets.QWidget(self.group_surrounding_nodes)
            rowl = QtWidgets.QHBoxLayout(roww); rowl.setContentsMargins(0,0,0,0)
            rowl.addWidget(spin_x); rowl.addWidget(spin_y)
            self.surrounding_nodes_layout.addRow(f"node {i}", roww)
            self.surrounding_node_spins.append((spin_x, spin_y))

    def LoadSurroundingVehicleEditorFields(self):
        det = getattr(self.editor_manager, "GetSelectedSurroundingVehicleDetails", lambda: None)()
        if not det:
            self.spin_surrounding_speed_edit.blockSignals(True)
            self.spin_surrounding_speed_edit.setValue(0.0)
            self.spin_surrounding_speed_edit.blockSignals(False)
            if hasattr(self, "_RebuildSurroundingVehicleNodeEditors"):
                self._RebuildSurroundingVehicleNodeEditors([])
            self._UpdateSurroundingVehicleEditorEnabled()
            return

        self.spin_surrounding_speed_edit.blockSignals(True)
        self.spin_surrounding_speed_edit.setValue(float(det["speed_mps"]))
        self.spin_surrounding_speed_edit.blockSignals(False)

        if hasattr(self, "_RebuildSurroundingVehicleNodeEditors"):
            self._RebuildSurroundingVehicleNodeEditors(det["points_xy"])

        self._UpdateSurroundingVehicleEditorEnabled()

    def _OnSurroundingVehicleSelectionChanged(self):
        items = self.list_surrounding.selectedItems()
        if not items:
            self.editor_manager.SelectSurroundingVehicleByIndex(None)
        else:
            idx = items[0].data(QtCore.Qt.UserRole)
            self.editor_manager.SelectSurroundingVehicleByIndex(int(idx))
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.LoadSurroundingVehicleEditorFields()

    def _OnSurroundingVehicleNodeChanged(self, node_index, spin_x, spin_y):
        ok, msg = self.editor_manager.UpdateSelectedSurroundingVehicleNode(node_index, spin_x.value(), spin_y.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if msg: self._SetStatus(msg)
        # 리스트에도 반영
        self.RefreshSurroundingVehicleList()

    def _OnSurroundingVehicleSpeedChanged(self, *_):
        ok, msg = self.editor_manager.UpdateSelectedSurroundingVehicleSpeed(self.spin_surrounding_speed_edit.value())
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        if msg: self._SetStatus(msg)
        self.RefreshSurroundingVehicleList()

    def _OnSurroundingVehicleApplyCreate(self):
        ok, msg = self.editor_manager.CommitCurrentSurroundingVehicleCreation()
        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()
        self.RefreshSurroundingVehicleList()
        if msg: self._SetStatus(msg)

    def _OnSetSurroundingVehiclesSubmode(self, submode: str):
        self.editor_manager.SetSurroundingVehiclesSubmode(submode)
        self.UpdateSurroundingVehiclesSubmodeButtons(submode)
        self._UpdateSurroundingVehicleEditorEnabled()
        self._SetStatus(f"Surrounding Vehicles submode -> {submode}")
        if submode == "create":
            self.canvas.setFocus()

    def UpdateSurroundingVehiclesSubmodeButtons(self, submode: str):
        m = {
            "view": self.button_surrounding_view,
            "create": self.button_surrounding_create,
            "modify": self.button_surrounding_modify,
            "delete": self.button_surrounding_delete
        }
        for k, b in m.items():
            b.setChecked(k == submode)

    def _UpdateSurroundingVehicleEditorEnabled(self):
        has_sel = getattr(self.editor_manager, "GetSelectedSurroundingVehicleIndex", lambda: None)() is not None
        is_modify = (self.editor_manager.editor_state.mode == "surrounding_vehicles" and
                    self.editor_manager.GetSurroundingVehiclesSubmode() == "modify")
        self.group_surrounding_editor.setEnabled(bool(has_sel and is_modify))

    # =========================
    # Bundle / View / Mode 핸들러
    # =========================
    def _OnNewBundle(self):
        self.editor_manager.NewBundleName()
        self._SetStatus("New bundle name reserved.")

    def _OnOpenBundle(self):
        dlg = QtWidgets.QFileDialog(self, "Select existing map bundle folder")
        dlg.setFileMode(QtWidgets.QFileDialog.Directory)
        dlg.setOption(QtWidgets.QFileDialog.ShowDirsOnly, True)
        if dlg.exec_():
            sel = dlg.selectedFiles()
            if sel:
                folder = sel[0]
                ok, msg = self.editor_manager.OpenBundle(folder)
                self._SetStatus(msg)
                for ed in self.editor_manager.editors.values(): ed.Redraw()
                self.canvas.draw_idle()
                self.RefreshLaneList()
                self.RefreshBuildingList()
                self.RefreshGnssShadowZoneList()      # ★ NEW
                self.LoadLaneEditorFields()
                self.LoadBuildingEditorFields()
                self.LoadGnssShadowZoneEditorFields() # ★ NEW

    def _OnSaveBundle(self):
        # 다이얼로그 띄워서 경로/파일타입 선택
        default_root = os.path.dirname(os.path.join(self.editor_manager.editor_state.results_root_dir,
                                                    self.editor_manager.editor_state.active_map_folder_name))
        if not os.path.isdir(default_root):
            default_root = self.editor_manager.editor_state.results_root_dir
        dlg = SaveBundleDialog(
            self,
            default_root=default_root,
            default_name=self.editor_manager.editor_state.active_map_folder_name,
            editor_state=self.editor_manager.editor_state
        )
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            info = dlg.result()
            out_dir = info["out_dir"]
            which = info["which"]
            ok, msg = self.editor_manager.SaveBundle(folder_path=out_dir, which=which)
            self._SetStatus(msg)
        else:
            self._SetStatus("Save cancelled.")
        self.canvas.draw_idle()

    def _OnFitView(self):
        self.editor_manager.FitView()
        self.canvas.draw_idle()
        self._SetStatus("Fitted view to data.")

    def _OnToggleLock(self):
        self.editor_manager.ToggleViewLock(bool(self.toggle_lock_view.isChecked()))
        self._SetStatus("View locked." if self.toggle_lock_view.isChecked() else "View unlocked.")

    def _OnToggleCommonViewMode(self):
        is_on = bool(self.toggle_common_view.isChecked())
        self.editor_manager.SetCommonViewMode(is_on)
        self._SetStatus(f"Common View Mode {'ON' if is_on else 'OFF'}.")

    def _OnSwitchMode(self, mode: str):
        self.editor_manager.SwitchMode(mode)
        self._SwitchModeUi(mode)
        self._SetStatus(f"Switched mode to {mode}.")
        if mode == "lanes":
            self.UpdateLaneSubmodeButtons(self.editor_manager.GetLaneSubmode())
            self._UpdateLaneEditorEnabled()
        elif mode == "buildings":
            self.UpdateBuildingSubmodeButtons(self.editor_manager.GetBuildingSubmode())
            self._UpdateBuildingEditorEnabled()
        elif mode == "guardrails":
            self.left_stack.setCurrentWidget(self.guardrails_left)
            self.right_stack.setCurrentWidget(self.guardrails_right)
            self.UpdateGuardSubmodeButtons(self.editor_manager.GetGuardrailsSubmode())
            self._UpdateGuardrailEditorEnabled()
        elif mode == "gnss_shadow_zones":                                               # ★ NEW
            self.left_stack.setCurrentWidget(self.gnss_left)
            self.right_stack.setCurrentWidget(self.gnss_right)
            self.UpdateGnssShadowZonesSubmodeButtons(self.editor_manager.GetGnssShadowZonesSubmode())
            self._UpdateGnssShadowZoneEditorEnabled()
        elif mode == "surrounding_vehicles":
            self.UpdateSurroundingVehiclesSubmodeButtons(self.editor_manager.GetSurroundingVehiclesSubmode())
            self._UpdateSurroundingVehicleEditorEnabled()

        for ed in self.editor_manager.editors.values(): ed.Redraw()
        self.canvas.draw_idle()

    def _OnSetLaneSubmode(self, submode: str):
        self.editor_manager.SetLaneSubmode(submode)
        self.UpdateLaneSubmodeButtons(submode)
        self._UpdateLaneEditorEnabled()
        self._SetStatus(f"Lanes submode -> {submode}")
        if submode == "create":
            self.canvas.setFocus()

    def _OnSetLaneType(self, lane_type: str):
        self.editor_manager.SetLaneType(lane_type)
        self._SetStatus(f"Lanes type -> {lane_type}")
        idx = self.combo_lane_type.findText(lane_type)
        if idx >= 0:
            self.combo_lane_type.setCurrentIndex(idx)

    def _OnSetBuildingSubmode(self, submode: str):
        self.editor_manager.SetBuildingSubmode(submode)
        self.UpdateBuildingSubmodeButtons(submode)
        self._UpdateBuildingEditorEnabled()
        self._SetStatus(f"Buildings submode -> {submode}")
    
    def _OnSetSignSubmode(self, submode: str):
        # editor_manager에 API 없을 때는 직접 상태값만 바꿔도 동작
        if hasattr(self.editor_manager, "SetSignSubmode"):
            self.editor_manager.SetSignSubmode(submode)
        else:
            self.editor_manager.editor_state.signs_submode = submode
            self.editor_manager.editors["signs"].Redraw()
        self.UpdateSignSubmodeButtons(submode)
        self._UpdateSignEditorEnabled()
        self._SetStatus(f"Signs submode -> {submode}")

    def _OnSetGnssShadowZonesSubmode(self, submode: str):  # ★ NEW
        if hasattr(self.editor_manager, "SetGnssShadowZonesSubmode"):
            self.editor_manager.SetGnssShadowZonesSubmode(submode)
        else:
            self.editor_manager.editor_state.gnss_shadow_zones_submode = submode
            self.editor_manager.editors["gnss_shadow_zones"].Redraw()
        self.UpdateGnssShadowZonesSubmodeButtons(submode)
        self._UpdateGnssShadowZoneEditorEnabled()
        self._SetStatus(f"GNSS Shadow Zones submode -> {submode}")

    def UpdateLaneSubmodeButtons(self, submode: str):
        m = {"view": self.button_lane_view, "create": self.button_lane_create,
             "move": self.button_lane_move, "delete": self.button_lane_delete}
        for k, b in m.items():
            b.setChecked(k == submode)

    def UpdateBuildingSubmodeButtons(self, submode: str):
        m = {"view": self.button_building_view, "create": self.button_building_create,
             "modify": self.button_building_modify, "delete": self.button_building_delete}
        for k, b in m.items():
            b.setChecked(k == submode)
    
    def UpdateSignSubmodeButtons(self, submode: str):
        m = {"view": self.button_sign_view, "create": self.button_sign_create,
            "move": self.button_sign_move, "delete": self.button_sign_delete}
        for k, b in m.items():
            b.setChecked(k == submode)
    
    def UpdateGnssShadowZonesSubmodeButtons(self, submode: str):  # ★ NEW
        m = {"view": self.button_gnss_view, "create": self.button_gnss_create,
            "move": self.button_gnss_move, "delete": self.button_gnss_delete}
        for k, b in m.items():
            b.setChecked(k == submode)

    # =========================
    # 유틸
    # =========================
    def _SetStatus(self, text: str):
        if self.status_bar:
            self.status_bar.showMessage(text, 3000)


def RunMapEditor():
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    w = MapEditorWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    RunMapEditor()
    

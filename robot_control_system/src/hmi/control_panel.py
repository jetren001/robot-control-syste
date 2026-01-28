"""
控制面板模块
实现HMI控制面板的各种控件和交互逻辑

作者: Cursor AI
日期: 2026-01-28
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QPushButton, QLabel, QSlider, QSpinBox,
    QDoubleSpinBox, QTabWidget, QFrame, QProgressBar,
    QTableWidget, QTableWidgetItem, QHeaderView, QComboBox,
    QLineEdit, QTextEdit, QSplitter, QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette

from typing import List, Callable, Optional
from dataclasses import dataclass


class StatusIndicator(QLabel):
    """状态指示器"""
    
    def __init__(self, text: str = "", parent=None):
        super().__init__(text, parent)
        self.setMinimumWidth(100)
        self.setAlignment(Qt.AlignCenter)
        self.setAutoFillBackground(True)
        self._set_color("gray")
    
    def _set_color(self, color: str):
        colors = {
            "green": "#4CAF50",
            "red": "#F44336",
            "yellow": "#FF9800",
            "blue": "#2196F3",
            "gray": "#9E9E9E"
        }
        bg_color = colors.get(color, colors["gray"])
        self.setStyleSheet(f"""
            QLabel {{
                background-color: {bg_color};
                color: white;
                border-radius: 5px;
                padding: 5px;
                font-weight: bold;
            }}
        """)
    
    def set_status(self, status: str, color: str = "gray"):
        self.setText(status)
        self._set_color(color)


class JointControl(QWidget):
    """单关节控制组件"""
    
    value_changed = pyqtSignal(int, float)  # (joint_id, value)
    jog_pressed = pyqtSignal(int, int)      # (joint_id, direction: +1/-1)
    jog_released = pyqtSignal(int)          # (joint_id)
    
    def __init__(self, joint_id: int, name: str, 
                 min_val: float, max_val: float, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self.min_val = min_val
        self.max_val = max_val
        
        self._setup_ui(name)
    
    def _setup_ui(self, name: str):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        
        # 关节名称
        self.label = QLabel(name)
        self.label.setMinimumWidth(30)
        self.label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(self.label)
        
        # 负方向点动按钮
        self.btn_minus = QPushButton("-")
        self.btn_minus.setFixedSize(30, 30)
        self.btn_minus.pressed.connect(lambda: self.jog_pressed.emit(self.joint_id, -1))
        self.btn_minus.released.connect(lambda: self.jog_released.emit(self.joint_id))
        layout.addWidget(self.btn_minus)
        
        # 滑块
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(self.min_val * 10))
        self.slider.setMaximum(int(self.max_val * 10))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider, 1)
        
        # 正方向点动按钮
        self.btn_plus = QPushButton("+")
        self.btn_plus.setFixedSize(30, 30)
        self.btn_plus.pressed.connect(lambda: self.jog_pressed.emit(self.joint_id, 1))
        self.btn_plus.released.connect(lambda: self.jog_released.emit(self.joint_id))
        layout.addWidget(self.btn_plus)
        
        # 数值显示/输入
        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(self.min_val, self.max_val)
        self.spinbox.setDecimals(2)
        self.spinbox.setSuffix("°")
        self.spinbox.setMinimumWidth(80)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)
        layout.addWidget(self.spinbox)
    
    def _on_slider_changed(self, value):
        real_value = value / 10.0
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(real_value)
        self.spinbox.blockSignals(False)
        self.value_changed.emit(self.joint_id, real_value)
    
    def _on_spinbox_changed(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(int(value * 10))
        self.slider.blockSignals(False)
        self.value_changed.emit(self.joint_id, value)
    
    def set_value(self, value: float):
        """设置当前值"""
        self.slider.blockSignals(True)
        self.spinbox.blockSignals(True)
        self.slider.setValue(int(value * 10))
        self.spinbox.setValue(value)
        self.slider.blockSignals(False)
        self.spinbox.blockSignals(False)
    
    def get_value(self) -> float:
        """获取当前值"""
        return self.spinbox.value()
    
    def set_enabled(self, enabled: bool):
        """设置启用状态"""
        self.slider.setEnabled(enabled)
        self.spinbox.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)
        self.btn_minus.setEnabled(enabled)


class ControlPanel(QWidget):
    """
    主控制面板
    包含所有控制元素
    """
    
    # 信号定义
    servo_enable_clicked = pyqtSignal()
    servo_disable_clicked = pyqtSignal()
    home_clicked = pyqtSignal()
    estop_clicked = pyqtSignal()
    mode_changed = pyqtSignal(str)  # "manual" or "auto"
    virtual_real_changed = pyqtSignal(str)  # "virtual" or "real"
    speed_changed = pyqtSignal(int)
    joint_value_changed = pyqtSignal(int, float)
    joint_jog_pressed = pyqtSignal(int, int)
    joint_jog_released = pyqtSignal(int)
    tcp_jog_pressed = pyqtSignal(str, int)  # (axis: "X"/"Y"/"Z"/"Rx"/"Ry"/"Rz", direction: +1/-1)
    tcp_jog_released = pyqtSignal(str)  # (axis)
    tcp_position_changed = pyqtSignal(list, list)  # (position [X,Y,Z], orientation [Rx,Ry,Rz])
    program_start_clicked = pyqtSignal()
    program_stop_clicked = pyqtSignal()
    program_pause_clicked = pyqtSignal()
    joints_zero_clicked = pyqtSignal()  # 全部归零信号
    record_point_clicked = pyqtSignal(list)  # 记录点位信号，传递当前关节角度
    teach_pick_clicked = pyqtSignal()  # 示教取料点
    teach_place_clicked = pyqtSignal()  # 示教放料点
    drag_teaching_changed = pyqtSignal(bool)  # 拖动示教开关
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
        self._connect_signals()
    
    def _setup_ui(self):
        """设置UI"""
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        
        # 创建选项卡
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # 基本控制选项卡
        self._create_basic_control_tab()
        
        # 关节控制选项卡
        self._create_joint_control_tab()
        
        # TCP控制选项卡
        self._create_tcp_control_tab()
        
        # 码垛程序选项卡
        self._create_program_tab()
        
        # 底部状态区域
        self._create_status_area(main_layout)
    
    def _create_basic_control_tab(self):
        """创建基本控制选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 模式切换组
        mode_group = QGroupBox("模式切换")
        mode_layout = QGridLayout(mode_group)
        
        # 手动/自动切换
        mode_layout.addWidget(QLabel("运行模式:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["手动", "自动"])
        mode_layout.addWidget(self.mode_combo, 0, 1)
        
        # 虚拟/实机切换
        mode_layout.addWidget(QLabel("连接模式:"), 1, 0)
        self.virtual_combo = QComboBox()
        self.virtual_combo.addItems(["虚拟仿真", "实机连接"])
        mode_layout.addWidget(self.virtual_combo, 1, 1)
        
        layout.addWidget(mode_group)
        
        # 控制按钮组
        button_group = QGroupBox("控制按钮")
        button_layout = QGridLayout(button_group)
        
        # 伺服使能
        self.btn_enable = QPushButton("伺服使能")
        self.btn_enable.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
        self.btn_enable.setMinimumHeight(40)
        button_layout.addWidget(self.btn_enable, 0, 0)
        
        # 伺服禁止
        self.btn_disable = QPushButton("伺服禁止")
        self.btn_disable.setStyleSheet("QPushButton { background-color: #FF9800; color: white; }")
        self.btn_disable.setMinimumHeight(40)
        button_layout.addWidget(self.btn_disable, 0, 1)
        
        # 回原点
        self.btn_home = QPushButton("回原点")
        self.btn_home.setStyleSheet("QPushButton { background-color: #2196F3; color: white; }")
        self.btn_home.setMinimumHeight(40)
        button_layout.addWidget(self.btn_home, 1, 0)
        
        # 急停
        self.btn_estop = QPushButton("急 停")
        self.btn_estop.setStyleSheet("""
            QPushButton { 
                background-color: #F44336; 
                color: white; 
                font-size: 18px;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #D32F2F;
            }
        """)
        self.btn_estop.setMinimumHeight(60)
        button_layout.addWidget(self.btn_estop, 1, 1)
        
        layout.addWidget(button_group)
        
        # 速度控制
        speed_group = QGroupBox("速度控制")
        speed_layout = QHBoxLayout(speed_group)
        
        speed_layout.addWidget(QLabel("速度:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        speed_layout.addWidget(self.speed_slider, 1)
        
        self.speed_label = QLabel("50%")
        self.speed_label.setMinimumWidth(50)
        speed_layout.addWidget(self.speed_label)
        
        layout.addWidget(speed_group)
        
        layout.addStretch()
        
        self.tab_widget.addTab(tab, "基本控制")
    
    def _create_joint_control_tab(self):
        """创建关节控制选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 关节配置
        joint_configs = [
            ("J1", -170, 170),
            ("J2", -120, 120),
            ("J3", -170, 170),
            ("J4", -120, 120),
            ("J5", -170, 170),
            ("J6", -360, 360),
        ]
        
        # 创建关节控制组件
        self.joint_controls: List[JointControl] = []
        
        joints_group = QGroupBox("关节控制")
        joints_layout = QVBoxLayout(joints_group)
        
        for i, (name, min_val, max_val) in enumerate(joint_configs):
            joint_ctrl = JointControl(i, name, min_val, max_val)
            self.joint_controls.append(joint_ctrl)
            joints_layout.addWidget(joint_ctrl)
        
        layout.addWidget(joints_group)
        
        # 快捷按钮
        quick_group = QGroupBox("快捷操作")
        quick_layout = QHBoxLayout(quick_group)
        
        self.btn_zero = QPushButton("全部归零")
        quick_layout.addWidget(self.btn_zero)
        
        self.btn_record = QPushButton("记录点位")
        quick_layout.addWidget(self.btn_record)
        
        layout.addWidget(quick_group)
        
        self.tab_widget.addTab(tab, "关节控制")
    
    def _create_tcp_control_tab(self):
        """创建TCP控制选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # TCP位置显示
        pos_group = QGroupBox("TCP位置")
        pos_layout = QGridLayout(pos_group)
        
        self.tcp_displays = {}
        coords = [("X", "mm"), ("Y", "mm"), ("Z", "mm"), 
                  ("Rx", "°"), ("Ry", "°"), ("Rz", "°")]
        
        for i, (name, unit) in enumerate(coords):
            row = i // 3
            col = (i % 3) * 2
            
            pos_layout.addWidget(QLabel(f"{name}:"), row, col)
            
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-9999, 9999)
            spinbox.setDecimals(2)
            spinbox.setSuffix(f" {unit}")
            self.tcp_displays[name] = spinbox
            pos_layout.addWidget(spinbox, row, col + 1)
        
        layout.addWidget(pos_group)
        
        # TCP点动按钮
        jog_group = QGroupBox("TCP点动")
        jog_layout = QGridLayout(jog_group)
        
        directions = ["X+", "X-", "Y+", "Y-", "Z+", "Z-", 
                      "Rx+", "Rx-", "Ry+", "Ry-", "Rz+", "Rz-"]
        
        self.tcp_jog_buttons = {}
        for i, name in enumerate(directions):
            btn = QPushButton(name)
            btn.setMinimumHeight(35)
            self.tcp_jog_buttons[name] = btn
            row = i // 4
            col = i % 4
            jog_layout.addWidget(btn, row, col)
        
        layout.addWidget(jog_group)
        
        # 拖动示教
        drag_group = QGroupBox("拖动示教")
        drag_layout = QVBoxLayout(drag_group)
        
        # 第一行：启用按钮
        row1 = QHBoxLayout()
        self.btn_drag_teaching = QPushButton("启用点击示教")
        self.btn_drag_teaching.setCheckable(True)
        self.btn_drag_teaching.setStyleSheet("""
            QPushButton { background-color: #607D8B; color: white; }
            QPushButton:checked { background-color: #4CAF50; }
        """)
        self.btn_drag_teaching.setMinimumHeight(40)
        row1.addWidget(self.btn_drag_teaching)
        row1.addWidget(QLabel("点击3D场景设置XY目标"))
        drag_layout.addLayout(row1)
        
        # 第二行：Z高度控制
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("目标Z高度:"))
        self.spin_target_z = QDoubleSpinBox()
        self.spin_target_z.setRange(0, 1000)
        self.spin_target_z.setValue(500)
        self.spin_target_z.setSuffix(" mm")
        row2.addWidget(self.spin_target_z)
        
        self.btn_z_up = QPushButton("Z+50")
        self.btn_z_up.setMaximumWidth(60)
        row2.addWidget(self.btn_z_up)
        
        self.btn_z_down = QPushButton("Z-50")
        self.btn_z_down.setMaximumWidth(60)
        row2.addWidget(self.btn_z_down)
        drag_layout.addLayout(row2)
        
        # 连接Z高度按钮
        self.btn_z_up.clicked.connect(lambda: self.spin_target_z.setValue(self.spin_target_z.value() + 50))
        self.btn_z_down.clicked.connect(lambda: self.spin_target_z.setValue(self.spin_target_z.value() - 50))
        
        layout.addWidget(drag_group)
        layout.addStretch()
        
        self.tab_widget.addTab(tab, "TCP控制")
    
    def _create_program_tab(self):
        """创建码垛程序选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 示教点位
        teach_group = QGroupBox("示教点位")
        teach_layout = QGridLayout(teach_group)
        
        # 取料点
        teach_layout.addWidget(QLabel("取料点:"), 0, 0)
        self.label_pick_point = QLabel("未设置")
        self.label_pick_point.setStyleSheet("color: gray;")
        teach_layout.addWidget(self.label_pick_point, 0, 1)
        self.btn_teach_pick = QPushButton("示教")
        self.btn_teach_pick.setMaximumWidth(60)
        teach_layout.addWidget(self.btn_teach_pick, 0, 2)
        
        # 放料起始点
        teach_layout.addWidget(QLabel("放料点:"), 1, 0)
        self.label_place_point = QLabel("未设置")
        self.label_place_point.setStyleSheet("color: gray;")
        teach_layout.addWidget(self.label_place_point, 1, 1)
        self.btn_teach_place = QPushButton("示教")
        self.btn_teach_place.setMaximumWidth(60)
        teach_layout.addWidget(self.btn_teach_place, 1, 2)
        
        # 间距设置
        teach_layout.addWidget(QLabel("X间距(mm):"), 2, 0)
        self.spin_x_spacing = QSpinBox()
        self.spin_x_spacing.setRange(10, 200)
        self.spin_x_spacing.setValue(50)
        teach_layout.addWidget(self.spin_x_spacing, 2, 1)
        
        teach_layout.addWidget(QLabel("Y间距(mm):"), 2, 2)
        self.spin_y_spacing = QSpinBox()
        self.spin_y_spacing.setRange(10, 200)
        self.spin_y_spacing.setValue(50)
        teach_layout.addWidget(self.spin_y_spacing, 2, 3)
        
        teach_layout.addWidget(QLabel("Z间距(mm):"), 3, 0)
        self.spin_z_spacing = QSpinBox()
        self.spin_z_spacing.setRange(10, 200)
        self.spin_z_spacing.setValue(30)
        teach_layout.addWidget(self.spin_z_spacing, 3, 1)
        
        layout.addWidget(teach_group)
        
        # 码垛配置
        config_group = QGroupBox("码垛配置")
        config_layout = QGridLayout(config_group)
        
        config_layout.addWidget(QLabel("行数:"), 0, 0)
        self.spin_rows = QSpinBox()
        self.spin_rows.setRange(1, 10)
        self.spin_rows.setValue(3)
        config_layout.addWidget(self.spin_rows, 0, 1)
        
        config_layout.addWidget(QLabel("列数:"), 0, 2)
        self.spin_cols = QSpinBox()
        self.spin_cols.setRange(1, 10)
        self.spin_cols.setValue(3)
        config_layout.addWidget(self.spin_cols, 0, 3)
        
        config_layout.addWidget(QLabel("层数:"), 1, 0)
        self.spin_layers = QSpinBox()
        self.spin_layers.setRange(1, 5)
        self.spin_layers.setValue(2)
        config_layout.addWidget(self.spin_layers, 1, 1)
        
        config_layout.addWidget(QLabel("总数:"), 1, 2)
        self.label_total = QLabel("18")
        self.label_total.setFont(QFont("Arial", 12, QFont.Bold))
        config_layout.addWidget(self.label_total, 1, 3)
        
        layout.addWidget(config_group)
        
        # 程序控制
        control_group = QGroupBox("程序控制")
        control_layout = QHBoxLayout(control_group)
        
        self.btn_start = QPushButton("启动")
        self.btn_start.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
        self.btn_start.setMinimumHeight(40)
        control_layout.addWidget(self.btn_start)
        
        self.btn_pause = QPushButton("暂停")
        self.btn_pause.setStyleSheet("QPushButton { background-color: #FF9800; color: white; }")
        self.btn_pause.setMinimumHeight(40)
        control_layout.addWidget(self.btn_pause)
        
        self.btn_stop = QPushButton("停止")
        self.btn_stop.setStyleSheet("QPushButton { background-color: #F44336; color: white; }")
        self.btn_stop.setMinimumHeight(40)
        control_layout.addWidget(self.btn_stop)
        
        layout.addWidget(control_group)
        
        # 进度显示
        progress_group = QGroupBox("运行进度")
        progress_layout = QVBoxLayout(progress_group)
        
        info_layout = QHBoxLayout()
        info_layout.addWidget(QLabel("当前循环:"))
        self.label_current = QLabel("0")
        info_layout.addWidget(self.label_current)
        info_layout.addWidget(QLabel("/"))
        self.label_total_cycles = QLabel("0")
        info_layout.addWidget(self.label_total_cycles)
        info_layout.addStretch()
        progress_layout.addLayout(info_layout)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        progress_layout.addWidget(self.progress_bar)
        
        layout.addWidget(progress_group)
        layout.addStretch()
        
        self.tab_widget.addTab(tab, "码垛程序")
    
    def _create_status_area(self, main_layout):
        """创建状态显示区域"""
        status_group = QGroupBox("系统状态")
        status_layout = QGridLayout(status_group)
        
        # 状态指示器
        status_layout.addWidget(QLabel("连接状态:"), 0, 0)
        self.status_connection = StatusIndicator("断开")
        status_layout.addWidget(self.status_connection, 0, 1)
        
        status_layout.addWidget(QLabel("运行模式:"), 0, 2)
        self.status_mode = StatusIndicator("手动")
        status_layout.addWidget(self.status_mode, 0, 3)
        
        status_layout.addWidget(QLabel("状态机:"), 1, 0)
        self.status_state = StatusIndicator("IDLE")
        status_layout.addWidget(self.status_state, 1, 1)
        
        status_layout.addWidget(QLabel("伺服状态:"), 1, 2)
        self.status_servo = StatusIndicator("禁止")
        status_layout.addWidget(self.status_servo, 1, 3)
        
        main_layout.addWidget(status_group)
    
    def _connect_signals(self):
        """连接信号"""
        # 按钮信号
        self.btn_enable.clicked.connect(self.servo_enable_clicked.emit)
        self.btn_disable.clicked.connect(self.servo_disable_clicked.emit)
        self.btn_home.clicked.connect(self.home_clicked.emit)
        self.btn_estop.clicked.connect(self.estop_clicked.emit)
        
        # 模式切换
        self.mode_combo.currentTextChanged.connect(
            lambda t: self.mode_changed.emit("manual" if t == "手动" else "auto"))
        self.virtual_combo.currentTextChanged.connect(
            lambda t: self.virtual_real_changed.emit("virtual" if "虚拟" in t else "real"))
        
        # 速度滑块
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        
        # 关节控制
        for jc in self.joint_controls:
            jc.value_changed.connect(self.joint_value_changed.emit)
            jc.jog_pressed.connect(self.joint_jog_pressed.emit)
            jc.jog_released.connect(self.joint_jog_released.emit)
        
        # 归零按钮
        self.btn_zero.clicked.connect(self._on_zero_clicked)
        self.btn_record.clicked.connect(self._on_record_clicked)
        
        # TCP点动按钮 - 使用clicked信号
        for name, btn in self.tcp_jog_buttons.items():
            axis = name[:-1]  # "X+", "X-" -> "X"
            direction = 1 if name.endswith('+') else -1
            # 使用functools.partial避免lambda闭包问题
            btn.clicked.connect(self._make_tcp_jog_handler(axis, direction))
        
        # TCP位置输入框 - 按回车或失去焦点时触发运动
        for name, spinbox in self.tcp_displays.items():
            spinbox.editingFinished.connect(self._on_tcp_input_finished)
        
        # 程序控制
        self.btn_start.clicked.connect(self.program_start_clicked.emit)
        self.btn_stop.clicked.connect(self.program_stop_clicked.emit)
        self.btn_pause.clicked.connect(self.program_pause_clicked.emit)
        
        # 示教点位
        self.btn_teach_pick.clicked.connect(self.teach_pick_clicked.emit)
        self.btn_teach_place.clicked.connect(self.teach_place_clicked.emit)
        
        # 拖动示教
        self.btn_drag_teaching.toggled.connect(self.drag_teaching_changed.emit)
        
        # 码垛配置变化
        self.spin_rows.valueChanged.connect(self._update_total)
        self.spin_cols.valueChanged.connect(self._update_total)
        self.spin_layers.valueChanged.connect(self._update_total)
    
    def _make_tcp_jog_handler(self, axis, direction):
        """创建TCP点动处理函数"""
        def handler():
            print(f"[ControlPanel] TCP点动信号: axis={axis}, direction={direction}")
            self.tcp_jog_pressed.emit(axis, direction)
        return handler
    
    def _on_tcp_input_finished(self):
        """TCP位置输入完成，发送运动指令"""
        position = [
            self.tcp_displays["X"].value(),
            self.tcp_displays["Y"].value(),
            self.tcp_displays["Z"].value()
        ]
        orientation = [
            self.tcp_displays["Rx"].value(),
            self.tcp_displays["Ry"].value(),
            self.tcp_displays["Rz"].value()
        ]
        print(f"[ControlPanel] TCP位置输入: pos={position}, ori={orientation}")
        self.tcp_position_changed.emit(position, orientation)
    
    def _on_speed_changed(self, value):
        self.speed_label.setText(f"{value}%")
        self.speed_changed.emit(value)
    
    def _on_zero_clicked(self):
        """全部归零按钮点击"""
        # 更新UI显示
        for jc in self.joint_controls:
            jc.set_value(0)
        # 发送信号让机器人移动到零位
        self.joints_zero_clicked.emit()
    
    def _on_record_clicked(self):
        """记录点位按钮点击"""
        # 获取当前关节角度
        current_joints = [jc.get_value() for jc in self.joint_controls]
        print(f"[ControlPanel] 记录点位: {current_joints}")
        self.record_point_clicked.emit(current_joints)
    
    def _update_total(self):
        total = self.spin_rows.value() * self.spin_cols.value() * self.spin_layers.value()
        self.label_total.setText(str(total))
        self.label_total_cycles.setText(str(total))
    
    # ==================== 公共接口 ====================
    
    def set_joint_values(self, values: List[float]):
        """设置关节值显示"""
        for i, value in enumerate(values):
            if i < len(self.joint_controls):
                self.joint_controls[i].set_value(value)
    
    def get_joint_values(self) -> List[float]:
        """获取关节值"""
        return [jc.get_value() for jc in self.joint_controls]
    
    def set_tcp_values(self, position: List[float], orientation: List[float]):
        """设置TCP显示值"""
        coords = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        values = position + orientation
        for name, value in zip(coords, values):
            if name in self.tcp_displays:
                self.tcp_displays[name].setValue(value)
    
    def set_connection_status(self, connected: bool):
        """设置连接状态"""
        if connected:
            self.status_connection.set_status("已连接", "green")
        else:
            self.status_connection.set_status("断开", "red")
    
    def set_state_machine_status(self, state: str):
        """设置状态机状态"""
        color_map = {
            "IDLE": "gray",
            "SCANNING": "blue",
            "READY": "blue",
            "ENABLED": "green",
            "HOMING": "yellow",
            "STANDBY": "green",
            "JOG": "green",
            "AUTO_RUN": "green",
            "PAUSED": "yellow",
            "FAULT": "red",
            "EMERGENCY_STOP": "red"
        }
        self.status_state.set_status(state, color_map.get(state, "gray"))
    
    def set_servo_status(self, enabled: bool):
        """设置伺服状态"""
        if enabled:
            self.status_servo.set_status("使能", "green")
        else:
            self.status_servo.set_status("禁止", "gray")
    
    def set_mode_status(self, mode: str):
        """设置模式状态"""
        if mode == "auto":
            self.status_mode.set_status("自动", "green")
        else:
            self.status_mode.set_status("手动", "blue")
    
    def set_progress(self, current: int, total: int):
        """设置进度"""
        self.label_current.setText(str(current))
        self.label_total_cycles.setText(str(total))
        if total > 0:
            self.progress_bar.setValue(int(current / total * 100))
        else:
            self.progress_bar.setValue(0)
    
    def set_pick_point(self, tcp_pos):
        """设置取料点显示"""
        self.label_pick_point.setText(f"X:{tcp_pos[0]:.1f} Y:{tcp_pos[1]:.1f} Z:{tcp_pos[2]:.1f}")
        self.label_pick_point.setStyleSheet("color: green; font-weight: bold;")
    
    def set_place_point(self, tcp_pos):
        """设置放料点显示"""
        self.label_place_point.setText(f"X:{tcp_pos[0]:.1f} Y:{tcp_pos[1]:.1f} Z:{tcp_pos[2]:.1f}")
        self.label_place_point.setStyleSheet("color: green; font-weight: bold;")
    
    def get_palletizing_config(self):
        """获取码垛配置"""
        return {
            'rows': self.spin_rows.value(),
            'cols': self.spin_cols.value(),
            'layers': self.spin_layers.value(),
            'x_spacing': self.spin_x_spacing.value(),
            'y_spacing': self.spin_y_spacing.value(),
            'z_spacing': self.spin_z_spacing.value()
        }
    
    def enable_controls(self, enabled: bool):
        """启用/禁用控件"""
        for jc in self.joint_controls:
            jc.set_enabled(enabled)
        
        self.btn_enable.setEnabled(not enabled if enabled else True)
        self.btn_home.setEnabled(enabled)


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    panel = ControlPanel()
    panel.setMinimumSize(400, 600)
    panel.show()
    
    # 测试设置值
    panel.set_joint_values([10, -20, 30, 0, 45, 90])
    panel.set_tcp_values([400, 100, 300], [180, 0, 0])
    panel.set_connection_status(True)
    panel.set_state_machine_status("STANDBY")
    panel.set_servo_status(True)
    
    sys.exit(app.exec_())

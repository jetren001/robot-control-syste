"""
主窗口模块
实现HMI主界面布局和逻辑

作者: Cursor AI
日期: 2026-01-28
"""

import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QStatusBar, QMenuBar, QMenu, QAction,
    QMessageBox, QFrame, QLabel, QDockWidget, QTextEdit,
    QApplication, QGroupBox, QTableWidget, QTableWidgetItem,
    QHeaderView
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QIcon, QKeySequence

from typing import Optional, List
from datetime import datetime

from .control_panel import ControlPanel
from .alarm_manager import AlarmManager, Alarm, AlarmSeverity


class AlarmWidget(QWidget):
    """报警显示组件"""
    
    def __init__(self, alarm_manager: AlarmManager, parent=None):
        super().__init__(parent)
        self.alarm_manager = alarm_manager
        self._setup_ui()
        
        # 设置报警回调
        self.alarm_manager.set_on_alarm(self._on_new_alarm)
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # 报警表格
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["时间", "代码", "来源", "描述"])
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setMaximumHeight(150)
        layout.addWidget(self.table)
        
        # 按钮行
        btn_layout = QHBoxLayout()
        
        from PyQt5.QtWidgets import QPushButton
        self.btn_ack = QPushButton("确认报警")
        self.btn_ack.clicked.connect(self._acknowledge_selected)
        btn_layout.addWidget(self.btn_ack)
        
        self.btn_clear = QPushButton("清除报警")
        self.btn_clear.clicked.connect(self._clear_acknowledged)
        btn_layout.addWidget(self.btn_clear)
        
        btn_layout.addStretch()
        layout.addLayout(btn_layout)
    
    def _on_new_alarm(self, alarm: Alarm):
        """新报警回调"""
        self._refresh_table()
    
    def _refresh_table(self):
        """刷新报警表格"""
        alarms = self.alarm_manager.get_active_alarms()
        self.table.setRowCount(len(alarms))
        
        for i, alarm in enumerate(alarms):
            time_str = alarm.timestamp.strftime("%H:%M:%S")
            self.table.setItem(i, 0, QTableWidgetItem(time_str))
            self.table.setItem(i, 1, QTableWidgetItem(alarm.code))
            self.table.setItem(i, 2, QTableWidgetItem(alarm.source))
            self.table.setItem(i, 3, QTableWidgetItem(alarm.message))
            
            # 根据严重程度设置颜色
            color = {
                AlarmSeverity.INFO: Qt.blue,
                AlarmSeverity.WARNING: Qt.darkYellow,
                AlarmSeverity.ERROR: Qt.red,
                AlarmSeverity.CRITICAL: Qt.darkRed
            }.get(alarm.severity, Qt.black)
            
            for j in range(4):
                item = self.table.item(i, j)
                if item:
                    item.setForeground(color)
                    item.setData(Qt.UserRole, alarm.id)
    
    def _acknowledge_selected(self):
        """确认选中的报警"""
        for item in self.table.selectedItems():
            alarm_id = item.data(Qt.UserRole)
            if alarm_id:
                self.alarm_manager.acknowledge_alarm(alarm_id)
        self._refresh_table()
    
    def _clear_acknowledged(self):
        """清除已确认的报警"""
        self.alarm_manager.clear_all()
        self._refresh_table()


class MainWindow(QMainWindow):
    """
    HMI主窗口
    
    布局:
    - 左侧: 3D机器人视窗 (预留)
    - 右侧: 控制面板
    - 底部: 报警区域
    - 状态栏: 系统状态
    """
    
    # 信号定义
    request_update = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # 初始化管理器
        self.alarm_manager = AlarmManager()
        
        # 3D视窗占位 (将在 robot_viewer_3d 模块中实现)
        self.robot_viewer = None
        
        # 设置UI
        self._setup_ui()
        self._setup_menu()
        self._setup_statusbar()
        self._connect_signals()
        
        # 定时器 - 用于更新显示
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._on_update_timer)
        self.update_timer.start(100)  # 100ms更新
    
    def _setup_ui(self):
        """设置主界面"""
        self.setWindowTitle("珞石 SR5-C 机器人控制系统")
        self.setMinimumSize(1280, 720)
        self.resize(1600, 900)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # 上部分割器 (3D视窗 | 控制面板)
        self.main_splitter = QSplitter(Qt.Horizontal)
        
        # 左侧: 3D视窗区域 (占位)
        self.viewer_container = QFrame()
        self.viewer_container.setFrameStyle(QFrame.StyledPanel)
        self.viewer_container.setStyleSheet("background-color: #2d2d2d;")
        
        viewer_layout = QVBoxLayout(self.viewer_container)
        self.viewer_placeholder = QLabel("3D 机器人视窗\n(PyVista 加载中...)")
        self.viewer_placeholder.setAlignment(Qt.AlignCenter)
        self.viewer_placeholder.setStyleSheet("color: #888; font-size: 24px;")
        viewer_layout.addWidget(self.viewer_placeholder)
        
        self.main_splitter.addWidget(self.viewer_container)
        
        # 右侧: 控制面板
        self.control_panel = ControlPanel()
        self.control_panel.setMinimumWidth(380)
        self.control_panel.setMaximumWidth(500)
        self.main_splitter.addWidget(self.control_panel)
        
        # 设置分割比例 (7:3)
        self.main_splitter.setSizes([1100, 400])
        
        main_layout.addWidget(self.main_splitter, 1)
        
        # 底部: 报警区域
        alarm_group = QGroupBox("报警信息")
        alarm_layout = QVBoxLayout(alarm_group)
        self.alarm_widget = AlarmWidget(self.alarm_manager)
        alarm_layout.addWidget(self.alarm_widget)
        alarm_group.setMaximumHeight(200)
        
        main_layout.addWidget(alarm_group)
    
    def _setup_menu(self):
        """设置菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu("文件(&F)")
        
        action_export = QAction("导出报警历史", self)
        action_export.triggered.connect(self._export_alarm_history)
        file_menu.addAction(action_export)
        
        file_menu.addSeparator()
        
        action_exit = QAction("退出(&X)", self)
        action_exit.setShortcut(QKeySequence.Quit)
        action_exit.triggered.connect(self.close)
        file_menu.addAction(action_exit)
        
        # 视图菜单
        view_menu = menubar.addMenu("视图(&V)")
        
        action_reset_view = QAction("重置3D视图", self)
        action_reset_view.triggered.connect(self._reset_3d_view)
        view_menu.addAction(action_reset_view)
        
        # 帮助菜单
        help_menu = menubar.addMenu("帮助(&H)")
        
        action_about = QAction("关于(&A)", self)
        action_about.triggered.connect(self._show_about)
        help_menu.addAction(action_about)
        
        action_manual = QAction("用户手册", self)
        action_manual.setShortcut(QKeySequence.HelpContents)
        action_manual.triggered.connect(self._show_manual)
        help_menu.addAction(action_manual)
    
    def _setup_statusbar(self):
        """设置状态栏"""
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        
        # 消息区域
        self.status_message = QLabel("就绪")
        self.statusbar.addWidget(self.status_message, 1)
        
        # 模式指示
        self.status_mode = QLabel("模式: 虚拟仿真")
        self.status_mode.setStyleSheet("padding: 0 10px;")
        self.statusbar.addPermanentWidget(self.status_mode)
        
        # 时间显示
        self.status_time = QLabel()
        self.status_time.setStyleSheet("padding: 0 10px;")
        self.statusbar.addPermanentWidget(self.status_time)
    
    def _connect_signals(self):
        """连接信号"""
        # 控制面板信号
        self.control_panel.servo_enable_clicked.connect(self._on_servo_enable)
        self.control_panel.servo_disable_clicked.connect(self._on_servo_disable)
        self.control_panel.home_clicked.connect(self._on_home)
        self.control_panel.estop_clicked.connect(self._on_estop)
        self.control_panel.mode_changed.connect(self._on_mode_changed)
        self.control_panel.virtual_real_changed.connect(self._on_virtual_real_changed)
        self.control_panel.speed_changed.connect(self._on_speed_changed)
        self.control_panel.joint_value_changed.connect(self._on_joint_changed)
        # 程序启动/停止信号在main.py中处理，这里不重复连接
    
    def _on_update_timer(self):
        """定时器更新"""
        # 更新时间显示
        self.status_time.setText(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        
        # 触发更新信号
        self.request_update.emit()
    
    # ==================== 事件处理 ====================
    
    def _on_servo_enable(self):
        """伺服使能"""
        self.show_message("正在使能伺服...")
        # TODO: 实际调用状态机
    
    def _on_servo_disable(self):
        """伺服禁止"""
        self.show_message("正在禁止伺服...")
    
    def _on_home(self):
        """回原点"""
        reply = QMessageBox.question(
            self, "确认", "确定要执行回原点操作吗?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.show_message("正在回原点...")
    
    def _on_estop(self):
        """急停"""
        self.show_message("急停已触发!", "red")
        self.alarm_manager.add_alarm('E009', source='HMI')
    
    def _on_mode_changed(self, mode: str):
        """模式切换"""
        self.show_message(f"切换到{'自动' if mode == 'auto' else '手动'}模式")
        self.control_panel.set_mode_status(mode)
    
    def _on_virtual_real_changed(self, mode: str):
        """虚实切换"""
        mode_text = "虚拟仿真" if mode == "virtual" else "实机连接"
        self.status_mode.setText(f"模式: {mode_text}")
        self.show_message(f"切换到{mode_text}模式")
    
    def _on_speed_changed(self, value: int):
        """速度变化"""
        pass  # 可以在状态栏显示
    
    def _on_joint_changed(self, joint_id: int, value: float):
        """关节值变化"""
        # 更新3D视图
        if self.robot_viewer:
            pass  # TODO: 更新3D模型
    
    def _on_program_start(self):
        """启动程序"""
        self.show_message("程序启动")
    
    def _on_program_stop(self):
        """停止程序"""
        self.show_message("程序停止")
    
    # ==================== 菜单操作 ====================
    
    def _export_alarm_history(self):
        """导出报警历史"""
        from PyQt5.QtWidgets import QFileDialog
        filepath, _ = QFileDialog.getSaveFileName(
            self, "导出报警历史", "alarm_history.json", "JSON Files (*.json)"
        )
        if filepath:
            self.alarm_manager.export_history(filepath)
            self.show_message(f"报警历史已导出到: {filepath}")
    
    def _reset_3d_view(self):
        """重置3D视图"""
        if self.robot_viewer:
            pass  # TODO: 重置视图
        self.show_message("3D视图已重置")
    
    def _show_about(self):
        """显示关于对话框"""
        QMessageBox.about(
            self,
            "关于",
            """<h2>珞石 SR5-C 机器人控制系统</h2>
            <p>版本: 1.0.0</p>
            <p>基于 AI 的六轴机器人全栈控制系统</p>
            <p>作者: Cursor AI</p>
            <p>日期: 2026-01-28</p>
            """
        )
    
    def _show_manual(self):
        """显示用户手册"""
        QMessageBox.information(
            self, "用户手册", 
            "请参阅 docs/用户操作手册.md"
        )
    
    # ==================== 公共接口 ====================
    
    def show_message(self, message: str, color: str = "black"):
        """在状态栏显示消息"""
        self.status_message.setText(message)
        self.status_message.setStyleSheet(f"color: {color};")
    
    def set_robot_viewer(self, viewer):
        """设置3D视图组件"""
        self.robot_viewer = viewer
        
        # 替换占位符
        layout = self.viewer_container.layout()
        layout.removeWidget(self.viewer_placeholder)
        self.viewer_placeholder.deleteLater()
        layout.addWidget(viewer)
    
    def update_joint_display(self, values: List[float]):
        """更新关节显示"""
        self.control_panel.set_joint_values(values)
    
    def update_tcp_display(self, position: List[float], orientation: List[float]):
        """更新TCP显示"""
        self.control_panel.set_tcp_values(position, orientation)
    
    def update_state_display(self, state: str):
        """更新状态机显示"""
        self.control_panel.set_state_machine_status(state)
    
    def closeEvent(self, event):
        """关闭事件"""
        reply = QMessageBox.question(
            self, "确认退出",
            "确定要退出程序吗?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.update_timer.stop()
            event.accept()
        else:
            event.ignore()


def run_hmi():
    """运行HMI程序"""
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    # 创建主窗口
    window = MainWindow()
    window.show()
    
    # 测试数据
    window.control_panel.set_joint_values([0, -30, 60, 0, 30, 0])
    window.control_panel.set_tcp_values([400, 100, 500], [180, 0, 0])
    window.control_panel.set_connection_status(True)
    window.control_panel.set_state_machine_status("STANDBY")
    window.control_panel.set_servo_status(True)
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    run_hmi()

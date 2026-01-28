"""
3D机器人视窗模块
使用PyVista实现机器人的3D可视化和数字孪生

作者: Cursor AI
日期: 2026-01-28
"""

import numpy as np
from typing import List, Optional, Tuple
import logging

# 尝试导入PyVista
try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False
    logging.warning("PyVista未安装, 3D视窗功能受限")

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QFrame, QLabel
from PyQt5.QtCore import Qt, pyqtSignal

# 导入运动学模块
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from kinematics import RokaeSR5Kinematics
except ImportError:
    RokaeSR5Kinematics = None


class RobotViewer3D(QWidget):
    """
    3D机器人可视化组件
    
    功能:
    - 显示机器人3D模型 (简化几何体)
    - 实时更新关节角度
    - 显示TCP轨迹
    - 显示坐标系
    """
    
    joint_angles_changed = pyqtSignal(list)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 运动学计算器
        self.kinematics = RokaeSR5Kinematics(use_simplified=True) if RokaeSR5Kinematics else None
        
        # 关节角度
        self._joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 轨迹记录
        self._trajectory_points: List[np.ndarray] = []
        self._max_trajectory_points = 500
        
        # PyVista对象
        self._plotter = None
        self._robot_actors = []
        self._trajectory_actor = None
        self._coordinate_actors = []
        
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        if PYVISTA_AVAILABLE:
            # 创建PyVista交互器
            self._plotter = QtInteractor(self)
            self._plotter.set_background('#2d2d2d')
            layout.addWidget(self._plotter.interactor)
            
            # 初始化场景
            self._init_scene()
        else:
            # 无PyVista时显示占位
            placeholder = QLabel("PyVista未安装\n请运行: pip install pyvista pyvistaqt")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("""
                QLabel {
                    background-color: #2d2d2d;
                    color: #888;
                    font-size: 18px;
                }
            """)
            layout.addWidget(placeholder)
    
    def _init_scene(self):
        """初始化3D场景"""
        if not self._plotter:
            return
        
        # 添加地面网格
        grid = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), 
                       i_size=1000, j_size=1000, i_resolution=20, j_resolution=20)
        self._plotter.add_mesh(grid, color='#404040', opacity=0.3, 
                              style='wireframe', line_width=1)
        
        # 添加世界坐标系
        self._add_coordinate_frame(np.eye(4), scale=100, name="world")
        
        # 初始绘制机器人
        self._draw_robot()
        
        # 设置相机
        self._plotter.camera_position = [
            (1200, -800, 800),   # 相机位置
            (200, 0, 300),      # 焦点
            (0, 0, 1)           # 上方向
        ]
        
        # 添加光源
        self._plotter.add_light(pv.Light(position=(500, 500, 1000), intensity=0.8))
    
    def _draw_robot(self):
        """绘制机器人"""
        if not self._plotter or not self.kinematics:
            return
        
        # 清除旧的机器人
        for actor in self._robot_actors:
            self._plotter.remove_actor(actor)
        self._robot_actors.clear()
        
        # 获取关节位置
        joint_positions = self.kinematics.get_joint_positions(self._joint_angles)
        
        # 连杆颜色
        colors = ['#FFD700', '#4169E1', '#32CD32', '#FF6347', '#9370DB', '#20B2AA']
        
        # 绘制连杆
        for i in range(len(joint_positions) - 1):
            start = joint_positions[i]
            end = joint_positions[i + 1]
            
            # 创建圆柱体连杆
            link = self._create_link(start, end, radius=20)
            if link:
                actor = self._plotter.add_mesh(link, color=colors[i % len(colors)], 
                                               opacity=0.9, smooth_shading=True)
                self._robot_actors.append(actor)
        
        # 绘制关节球
        for i, pos in enumerate(joint_positions):
            sphere = pv.Sphere(radius=25, center=pos)
            actor = self._plotter.add_mesh(sphere, color='#808080', 
                                          smooth_shading=True)
            self._robot_actors.append(actor)
        
        # 绘制末端执行器坐标系
        if len(joint_positions) > 0:
            pose = self.kinematics.forward_kinematics(self._joint_angles)
            T = np.eye(4)
            T[:3, :3] = pose.rotation_matrix
            T[:3, 3] = pose.position
            self._add_coordinate_frame(T, scale=50, name="tcp")
    
    def _create_link(self, start: np.ndarray, end: np.ndarray, 
                    radius: float = 15):
        """创建连杆几何体"""
        direction = end - start
        length = np.linalg.norm(direction)
        
        if length < 1e-6:
            return None
        
        # 创建圆柱体
        cylinder = pv.Cylinder(center=(start + end) / 2, 
                              direction=direction,
                              radius=radius, 
                              height=length)
        return cylinder
    
    def _add_coordinate_frame(self, transform: np.ndarray, scale: float = 50, 
                             name: str = ""):
        """添加坐标系"""
        if not self._plotter:
            return
        
        origin = transform[:3, 3]
        
        # X轴 - 红色
        x_end = origin + transform[:3, 0] * scale
        x_line = pv.Line(origin, x_end)
        actor = self._plotter.add_mesh(x_line, color='red', line_width=3)
        self._coordinate_actors.append(actor)
        
        # Y轴 - 绿色
        y_end = origin + transform[:3, 1] * scale
        y_line = pv.Line(origin, y_end)
        actor = self._plotter.add_mesh(y_line, color='green', line_width=3)
        self._coordinate_actors.append(actor)
        
        # Z轴 - 蓝色
        z_end = origin + transform[:3, 2] * scale
        z_line = pv.Line(origin, z_end)
        actor = self._plotter.add_mesh(z_line, color='blue', line_width=3)
        self._coordinate_actors.append(actor)
    
    def set_joint_angles(self, angles: List[float]):
        """
        设置关节角度并更新显示
        
        Args:
            angles: 6个关节角度 (度)
        """
        if len(angles) != 6:
            return
        
        self._joint_angles = list(angles)
        
        # 记录轨迹点
        if self.kinematics:
            pose = self.kinematics.forward_kinematics(angles)
            self._add_trajectory_point(pose.position)
        
        # 更新机器人显示
        self._draw_robot()
        
        # 更新显示
        if self._plotter:
            self._plotter.update()
        
        self.joint_angles_changed.emit(self._joint_angles)
    
    def get_joint_angles(self) -> List[float]:
        """获取当前关节角度"""
        return self._joint_angles.copy()
    
    def _add_trajectory_point(self, point: np.ndarray):
        """添加轨迹点"""
        self._trajectory_points.append(point.copy())
        
        # 限制点数
        if len(self._trajectory_points) > self._max_trajectory_points:
            self._trajectory_points = self._trajectory_points[-self._max_trajectory_points:]
        
        # 更新轨迹显示
        self._update_trajectory()
    
    def _update_trajectory(self):
        """更新轨迹显示"""
        if not self._plotter or len(self._trajectory_points) < 2:
            return
        
        # 移除旧轨迹
        if self._trajectory_actor:
            self._plotter.remove_actor(self._trajectory_actor)
        
        # 创建轨迹线
        points = np.array(self._trajectory_points)
        lines = pv.Spline(points, len(points) * 3)
        self._trajectory_actor = self._plotter.add_mesh(
            lines, color='yellow', line_width=2, opacity=0.7
        )
    
    def clear_trajectory(self):
        """清除轨迹"""
        self._trajectory_points.clear()
        if self._trajectory_actor and self._plotter:
            self._plotter.remove_actor(self._trajectory_actor)
            self._trajectory_actor = None
    
    def reset_view(self):
        """重置视图"""
        if self._plotter:
            self._plotter.camera_position = [
                (1200, -800, 800),
                (200, 0, 300),
                (0, 0, 1)
            ]
            self._plotter.reset_camera()
    
    def set_view_angle(self, angle: str):
        """
        设置预设视角
        
        Args:
            angle: "front", "side", "top", "iso"
        """
        if not self._plotter:
            return
        
        views = {
            "front": [(0, -1500, 400), (0, 0, 400), (0, 0, 1)],
            "side": [(1500, 0, 400), (0, 0, 400), (0, 0, 1)],
            "top": [(0, 0, 1500), (0, 0, 0), (0, 1, 0)],
            "iso": [(1200, -800, 800), (200, 0, 300), (0, 0, 1)]
        }
        
        if angle in views:
            self._plotter.camera_position = views[angle]
    
    def get_tcp_position(self) -> Tuple[List[float], List[float]]:
        """获取当前TCP位置和姿态"""
        if not self.kinematics:
            return [0, 0, 0], [0, 0, 0]
        
        pose = self.kinematics.forward_kinematics(self._joint_angles)
        return pose.position.tolist(), pose.euler_angles.tolist()
    
    def closeEvent(self, event):
        """关闭事件"""
        if self._plotter:
            self._plotter.close()
        super().closeEvent(event)


class SimpleRobotViewer(QWidget):
    """
    简化版机器人视图 (不使用PyVista)
    使用matplotlib进行2D/伪3D显示
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.kinematics = RokaeSR5Kinematics(use_simplified=True) if RokaeSR5Kinematics else None
        self._joint_angles = [0.0] * 6
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        try:
            import matplotlib
            matplotlib.use('Qt5Agg')
            from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
            from matplotlib.figure import Figure
            from mpl_toolkits.mplot3d import Axes3D
            
            self.figure = Figure(figsize=(8, 6), dpi=100)
            self.figure.patch.set_facecolor('#2d2d2d')
            self.canvas = FigureCanvasQTAgg(self.figure)
            layout.addWidget(self.canvas)
            
            self.ax = self.figure.add_subplot(111, projection='3d')
            self.ax.set_facecolor('#2d2d2d')
            self._draw_robot()
            
        except Exception as e:
            label = QLabel(f"Matplotlib初始化失败: {e}")
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label)
    
    def _draw_robot(self):
        if not hasattr(self, 'ax') or not self.kinematics:
            return
        
        self.ax.clear()
        self.ax.set_facecolor('#2d2d2d')
        
        # 获取关节位置
        positions = self.kinematics.get_joint_positions(self._joint_angles)
        
        # 绘制连杆
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        
        self.ax.plot(xs, ys, zs, 'b-', linewidth=3, marker='o', markersize=8)
        
        # 设置坐标轴
        self.ax.set_xlabel('X (mm)', color='white')
        self.ax.set_ylabel('Y (mm)', color='white')
        self.ax.set_zlabel('Z (mm)', color='white')
        
        # 设置范围
        max_range = 800
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        self.ax.set_zlim(0, max_range)
        
        self.ax.tick_params(colors='white')
        
        self.canvas.draw()
    
    def set_joint_angles(self, angles: List[float]):
        if len(angles) == 6:
            self._joint_angles = list(angles)
            self._draw_robot()
    
    def get_joint_angles(self) -> List[float]:
        return self._joint_angles.copy()


def create_robot_viewer(parent=None) -> QWidget:
    """
    工厂函数: 创建机器人视图组件
    根据可用的库自动选择实现
    """
    if PYVISTA_AVAILABLE:
        return RobotViewer3D(parent)
    else:
        return SimpleRobotViewer(parent)


if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    
    viewer = create_robot_viewer()
    viewer.setMinimumSize(800, 600)
    viewer.show()
    
    # 测试动画
    from PyQt5.QtCore import QTimer
    
    angle = [0]
    def animate():
        angle[0] += 2
        angles = [
            30 * np.sin(np.radians(angle[0])),
            -30 + 20 * np.sin(np.radians(angle[0] * 0.5)),
            45 * np.sin(np.radians(angle[0] * 0.7)),
            0,
            30 * np.sin(np.radians(angle[0] * 1.2)),
            angle[0] % 360 - 180
        ]
        viewer.set_joint_angles(angles)
    
    timer = QTimer()
    timer.timeout.connect(animate)
    timer.start(50)
    
    sys.exit(app.exec_())

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
    - 显示物料箱子和托盘
    - 支持拖动示教
    """
    
    joint_angles_changed = pyqtSignal(list)
    tcp_dragged = pyqtSignal(list, list)  # 拖动示教信号 (position, orientation)
    
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
        
        # 物料和场景对象
        self._pick_box_actor = None      # 取料位的箱子
        self._pallet_actor = None        # 托盘
        self._placed_boxes = []          # 已放置的箱子actors
        self._pick_position = None       # 取料位置
        self._place_base_position = None # 放料起始位置
        self._gripper_holding = False    # 夹爪是否抓取中
        self._gripper_box_actor = None   # 夹爪上的箱子
        
        # 拖动示教相关
        self._drag_enabled = False
        self._dragging = False
        self._tcp_sphere_actor = None
        
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        if PYVISTA_AVAILABLE:
            # 创建PyVista交互器
            self._plotter = QtInteractor(self)
            self._plotter.set_background('#1a1a2e')
            layout.addWidget(self._plotter.interactor)
            
            # 初始化场景
            self._init_scene()
            
            # 设置鼠标交互回调
            self._setup_mouse_interaction()
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
        
        # 添加地面 (更真实的地面)
        floor = pv.Plane(center=(300, 0, -5), direction=(0, 0, 1), 
                        i_size=1500, j_size=1200, i_resolution=1, j_resolution=1)
        self._plotter.add_mesh(floor, color='#3a3a3a', opacity=1.0)
        
        # 添加地面网格线
        grid = pv.Plane(center=(300, 0, 0), direction=(0, 0, 1), 
                       i_size=1500, j_size=1200, i_resolution=30, j_resolution=24)
        self._plotter.add_mesh(grid, color='#505050', opacity=0.5, 
                              style='wireframe', line_width=1)
        
        # 添加世界坐标系
        self._add_coordinate_frame(np.eye(4), scale=100, name="world")
        
        # 初始绘制机器人
        self._draw_robot()
        
        # 添加默认的取料台和托盘
        self._setup_material_scene()
        
        # 设置相机
        self._plotter.camera_position = [
            (1000, -1000, 800),   # 相机位置
            (300, 0, 200),       # 焦点
            (0, 0, 1)            # 上方向
        ]
        
        # 添加光源
        self._plotter.add_light(pv.Light(position=(500, 500, 1000), intensity=0.7))
        self._plotter.add_light(pv.Light(position=(-500, 500, 800), intensity=0.3))
    
    def _setup_material_scene(self):
        """设置物料场景"""
        if not self._plotter:
            return
        
        # 取料台 (左侧) - 带传送带效果
        pick_table = pv.Box(bounds=(450, 650, -200, 0, 0, 80))
        self._plotter.add_mesh(pick_table, color='#2196F3', opacity=0.9)
        
        # 取料台上的箱子 (无限供料)
        self._pick_position = [550, -100, 110]
        self._update_pick_box()
        
        # 放料托盘 (右侧) - 木托盘
        pallet_base = pv.Box(bounds=(450, 750, 100, 400, 0, 20))
        self._pallet_actor = self._plotter.add_mesh(pallet_base, color='#8B4513', opacity=0.9)
        
        # 托盘横条
        for i in range(5):
            slat = pv.Box(bounds=(450, 750, 100 + i*75, 110 + i*75, 20, 30))
            self._plotter.add_mesh(slat, color='#A0522D', opacity=0.9)
        
        self._place_base_position = [500, 150, 50]
        
        # 添加标签
        self._plotter.add_text("取料台", position=(520, 50, 150), font_size=12, color='white')
        self._plotter.add_text("码垛托盘", position=(550, 350, 100), font_size=12, color='white')
    
    def _update_pick_box(self):
        """更新取料位的箱子"""
        if not self._plotter:
            return
        
        if self._pick_box_actor:
            self._plotter.remove_actor(self._pick_box_actor)
        
        # 创建箱子 (纸箱颜色)
        pos = self._pick_position
        box = pv.Box(bounds=(pos[0]-30, pos[0]+30, pos[1]-30, pos[1]+30, 
                            pos[2]-30, pos[2]+30))
        self._pick_box_actor = self._plotter.add_mesh(box, color='#D2691E', opacity=0.95)
    
    def set_pick_position(self, tcp_pos):
        """设置取料位置"""
        self._pick_position = list(tcp_pos)
        self._update_pick_box()
        if self._plotter:
            self._plotter.render()
    
    def set_place_position(self, tcp_pos):
        """设置放料起始位置"""
        self._place_base_position = list(tcp_pos)
    
    def pick_box(self):
        """抓取箱子"""
        self._gripper_holding = True
        # 取料位的箱子变暗（表示正在抓取）
        if self._pick_box_actor:
            self._plotter.remove_actor(self._pick_box_actor)
        self._update_gripper_box()
    
    def place_box(self, position=None):
        """放置箱子 - 使用当前TCP位置"""
        if not self._gripper_holding or not self._plotter:
            return
        
        self._gripper_holding = False
        
        # 移除夹爪上的箱子
        if self._gripper_box_actor:
            self._plotter.remove_actor(self._gripper_box_actor)
            self._gripper_box_actor = None
        
        # 获取当前真实TCP位置作为放置位置
        if self.kinematics:
            pose = self.kinematics.forward_kinematics(self._joint_angles)
            actual_pos = pose.position
        else:
            actual_pos = position if position else [0, 0, 0]
        
        # 在真实TCP位置放置箱子（箱子底部在TCP下方）
        box = pv.Box(bounds=(actual_pos[0]-30, actual_pos[0]+30, 
                            actual_pos[1]-30, actual_pos[1]+30,
                            actual_pos[2]-60, actual_pos[2]))
        actor = self._plotter.add_mesh(box, color='#D2691E', opacity=0.95)
        self._placed_boxes.append(actor)
        
        print(f"[3D] 箱子放置在真实TCP位置: [{actual_pos[0]:.1f}, {actual_pos[1]:.1f}, {actual_pos[2]:.1f}]")
        
        # 恢复取料位的箱子
        self._update_pick_box()
        
        if self._plotter:
            self._plotter.render()
    
    def _update_gripper_box(self):
        """更新夹爪上的箱子"""
        if not self._gripper_holding or not self._plotter:
            return
        
        if self._gripper_box_actor:
            self._plotter.remove_actor(self._gripper_box_actor)
        
        # 获取当前TCP位置
        if self.kinematics:
            pose = self.kinematics.forward_kinematics(self._joint_angles)
            pos = pose.position
            box = pv.Box(bounds=(pos[0]-30, pos[0]+30, 
                                pos[1]-30, pos[1]+30,
                                pos[2]-60, pos[2]))
            self._gripper_box_actor = self._plotter.add_mesh(box, color='#FF8C00', opacity=0.9)
    
    def clear_placed_boxes(self):
        """清除所有已放置的箱子"""
        if not self._plotter:
            return
        for actor in self._placed_boxes:
            self._plotter.remove_actor(actor)
        self._placed_boxes.clear()
        if self._plotter:
            self._plotter.render()
    
    def _setup_mouse_interaction(self):
        """设置鼠标交互 - 点击移动模式"""
        if not self._plotter:
            return
        
        # 添加提示文字
        self._plotter.add_text(
            "SR5-C 机器人控制系统",
            position='upper_left', font_size=10, color='white', name='help_text'
        )
        
        # 显示当前TCP坐标
        self._update_tcp_display()
    
    def _update_tcp_display(self):
        """更新TCP坐标显示"""
        if not self._plotter or not self.kinematics:
            return
        pose = self.kinematics.forward_kinematics(self._joint_angles)
        tcp = pose.position
        self._plotter.add_text(
            f"TCP: [{tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}]",
            position='upper_right', font_size=10, color='cyan', name='tcp_display'
        )
    
    def enable_drag_teaching(self, enabled: bool):
        """启用/禁用点击移动示教"""
        self._drag_enabled = enabled
        if not self._plotter:
            return
            
        if enabled:
            # 启用点选模式 - 点击场景中任意位置，TCP移动到该XY位置
            self._plotter.enable_surface_point_picking(
                callback=self._on_surface_picked,
                show_message=False,
                show_point=True,
                color='lime',
                point_size=15,
                tolerance=0.01,
            )
            
            pose = self.kinematics.forward_kinematics(self._joint_angles)
            tcp_pos = pose.position
            
            self._plotter.add_text(
                f"点击示教已启用 - 点击场景设置目标XY位置 (Z保持{tcp_pos[2]:.0f}mm)",
                position='upper_left', font_size=11, color='lime', name='help_text'
            )
            
            # 重绘机器人以显示TCP指示球
            self._draw_robot()
        else:
            # 禁用点选
            try:
                self._plotter.disable_picking()
            except:
                pass
            
            self._plotter.add_text(
                "SR5-C 机器人控制系统",
                position='upper_left', font_size=10, color='white', name='help_text'
            )
            self._draw_robot()
        
        if self._plotter:
            self._plotter.render()
    
    def _on_surface_picked(self, point):
        """当用户点击场景表面时"""
        if not self._drag_enabled or point is None:
            return
        
        print(f"[3D] 点击位置: {point}")
        
        # 获取当前TCP位置和姿态
        pose = self.kinematics.forward_kinematics(self._joint_angles)
        current_z = pose.position[2]
        
        # 新目标位置：点击的XY + 当前Z高度
        target_pos = [point[0], point[1], current_z]
        target_ori = pose.euler_angles.tolist()
        
        print(f"[3D] 目标TCP: {target_pos}")
        
        # 发送信号
        self.tcp_dragged.emit(target_pos, target_ori)
        
        # 更新提示
        self._plotter.add_text(
            f"移动到: [{target_pos[0]:.0f}, {target_pos[1]:.0f}, {target_pos[2]:.0f}]",
            position='upper_left', font_size=11, color='yellow', name='help_text'
        )
    
    def set_tcp_z_height(self, z: float):
        """设置TCP目标Z高度"""
        self._target_z = z
    
    def _draw_robot(self):
        """绘制机器人 - 使用关节位置直接绘制"""
        if not self._plotter or not self.kinematics:
            return
        
        # 清除旧的机器人
        for actor in self._robot_actors:
            self._plotter.remove_actor(actor)
        self._robot_actors.clear()
        
        # 清除旧的坐标系
        for actor in self._coordinate_actors:
            self._plotter.remove_actor(actor)
        self._coordinate_actors.clear()
        
        # 获取关节位置 (7个点: 基座 + 6个关节)
        positions = self.kinematics.get_joint_positions(self._joint_angles)
        
        # 颜色方案 - Rokae橙白配色
        ORANGE = '#FF6600'
        WHITE = '#E8E8E8'
        GRAY = '#505050'
        DARK_GRAY = '#303030'
        
        # ========== 1. 基座 ==========
        # 底座圆盘
        base = pv.Cylinder(center=[0, 0, 20], direction=[0, 0, 1], radius=100, height=40)
        actor = self._plotter.add_mesh(base, color=DARK_GRAY, smooth_shading=True)
        self._robot_actors.append(actor)
        
        # 基座到J1的立柱
        p0, p1 = np.array(positions[0]), np.array(positions[1])
        base_link = self._create_cylinder_link(np.array([0, 0, 40]), p1, radius=55)
        if base_link:
            actor = self._plotter.add_mesh(base_link, color=ORANGE, smooth_shading=True)
            self._robot_actors.append(actor)
        
        # ========== 2. 连杆绘制 ==========
        link_colors = [ORANGE, WHITE, ORANGE, WHITE, ORANGE, WHITE]
        link_radii = [50, 45, 40, 35, 30, 25]
        
        for i in range(len(positions) - 1):
            start = np.array(positions[i])
            end = np.array(positions[i + 1])
            
            # 创建连杆
            link = self._create_cylinder_link(start, end, radius=link_radii[i])
            if link:
                actor = self._plotter.add_mesh(link, color=link_colors[i], smooth_shading=True)
                self._robot_actors.append(actor)
            
            # 关节球
            joint_sphere = pv.Sphere(radius=link_radii[i] + 5, center=start)
            actor = self._plotter.add_mesh(joint_sphere, color=GRAY, smooth_shading=True)
            self._robot_actors.append(actor)
        
        # ========== 3. 末端执行器 ==========
        pose = self.kinematics.forward_kinematics(self._joint_angles)
        tcp_pos = pose.position
        
        # TCP坐标系
        T_tcp = np.eye(4)
        T_tcp[:3, :3] = pose.rotation_matrix
        T_tcp[:3, 3] = pose.position
        self._add_coordinate_frame(T_tcp, scale=100, name="tcp")
        
        # 末端关节球
        end_sphere = pv.Sphere(radius=30, center=positions[-1])
        actor = self._plotter.add_mesh(end_sphere, color=GRAY, smooth_shading=True)
        self._robot_actors.append(actor)
        
        # 夹爪 (简化)
        tcp_z = pose.rotation_matrix[:, 2]
        gripper = pv.Cylinder(center=tcp_pos, direction=tcp_z, radius=25, height=80)
        actor = self._plotter.add_mesh(gripper, color=DARK_GRAY, smooth_shading=True)
        self._robot_actors.append(actor)
        
        # ========== 4. TCP指示球 (拖动用) ==========
        if self._drag_enabled:
            tcp_sphere = pv.Sphere(radius=35, center=tcp_pos)
            self._tcp_sphere_actor = self._plotter.add_mesh(
                tcp_sphere, color='#00FF00', opacity=0.7, pickable=True
            )
            self._robot_actors.append(self._tcp_sphere_actor)
        
        # 更新夹爪上的箱子
        if self._gripper_holding:
            self._update_gripper_box()
    
    def _create_cylinder_link(self, start: np.ndarray, end: np.ndarray, radius: float = 30):
        """创建圆柱体连杆"""
        direction = end - start
        length = np.linalg.norm(direction)
        
        if length < 1e-6:
            return None
        
        center = (start + end) / 2
        cylinder = pv.Cylinder(center=center, direction=direction, radius=radius, height=length)
        return cylinder
    
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
        
        # 检查角度是否真的变化了 (优化性能)
        angle_changed = False
        for i in range(6):
            if abs(self._joint_angles[i] - angles[i]) > 0.01:
                angle_changed = True
                break
        
        if not angle_changed:
            return
        
        self._joint_angles = list(angles)
        
        # 记录轨迹点 (降低频率，每5次更新记录一次)
        if not hasattr(self, '_trajectory_counter'):
            self._trajectory_counter = 0
        self._trajectory_counter += 1
        
        if self.kinematics and self._trajectory_counter % 5 == 0:
            pose = self.kinematics.forward_kinematics(angles)
            self._add_trajectory_point(pose.position)
        
        # 更新机器人显示
        self._draw_robot()
        
        # 更新显示 (使用render代替update以提高性能)
        if self._plotter:
            self._plotter.render()
        
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

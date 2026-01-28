"""
珞石 SR5-C 机器人控制系统 - 主程序入口

作者: Cursor AI
日期: 2026-01-28
"""

import sys
import os

# 添加src目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from PyQt5.QtWidgets import QApplication, QSplashScreen, QMessageBox
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QFont

import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def check_dependencies():
    """检查依赖项"""
    missing = []
    
    try:
        import numpy
    except ImportError:
        missing.append('numpy')
    
    try:
        import PyQt5
    except ImportError:
        missing.append('PyQt5')
    
    # 可选依赖
    optional_missing = []
    
    try:
        import pyvista
        import pyvistaqt
    except ImportError:
        optional_missing.append('pyvista pyvistaqt')
    
    return missing, optional_missing


def main():
    """主函数"""
    logger.info("启动珞石 SR5-C 机器人控制系统...")
    
    # 检查依赖
    missing, optional_missing = check_dependencies()
    
    if missing:
        print(f"错误: 缺少必要依赖: {', '.join(missing)}")
        print("请运行: pip install " + " ".join(missing))
        return 1
    
    if optional_missing:
        logger.warning(f"可选依赖未安装: {', '.join(optional_missing)}")
        logger.warning("3D视图功能可能受限")
    
    # 创建应用
    app = QApplication(sys.argv)
    app.setApplicationName("珞石 SR5-C 机器人控制系统")
    app.setOrganizationName("ROKAE")
    app.setStyle('Fusion')
    
    # 设置字体
    font = QFont("Microsoft YaHei", 9)
    app.setFont(font)
    
    # 导入主窗口
    from hmi import MainWindow
    from hmi.robot_viewer_3d import create_robot_viewer
    from control import RobotStateMachine
    from driver import VirtualDriver
    
    # 创建主窗口
    window = MainWindow()
    
    # 创建3D视图
    robot_viewer = create_robot_viewer()
    window.set_robot_viewer(robot_viewer)
    
    # 创建状态机和驱动器
    state_machine = RobotStateMachine()
    driver = VirtualDriver()
    
    # 连接虚拟驱动器
    driver.connect()
    
    # 设置状态机回调
    def on_state_change(old_state, new_state):
        window.update_state_display(new_state.name)
        logger.info(f"状态变更: {old_state.name} -> {new_state.name}")
    
    state_machine.set_on_state_change(on_state_change)
    
    # 连接控制面板信号
    def on_servo_enable():
        if state_machine.current_state.name in ['READY', 'STANDBY']:
            driver.enable_servo()
            state_machine.enable_servo()
            window.control_panel.set_servo_status(True)
            window.show_message("伺服已使能")
    
    def on_servo_disable():
        driver.disable_servo()
        state_machine.disable_servo()
        window.control_panel.set_servo_status(False)
        window.show_message("伺服已禁止")
    
    def on_home():
        if driver.is_servo_enabled():
            driver.home()
            state_machine.start_homing()
            window.show_message("正在回原点...")
    
    def on_estop():
        # 只有在非急停状态下才触发急停
        if state_machine.current_state.name != 'EMERGENCY_STOP':
            driver.emergency_stop()
            state_machine.emergency_stop()
            window.alarm_manager.add_alarm('E009', source='HMI')
            window.show_message("急停已触发!", "red")
        else:
            # 已经在急停状态，执行复位
            driver.reset_error()
            state_machine.release_emergency()
            state_machine.reset_fault()
            state_machine.start_scan()
            state_machine.scan_complete()
            window.alarm_manager.acknowledge_all()
            window.alarm_manager.clear_all()
            window.control_panel.set_servo_status(False)
            window.show_message("急停已复位，系统就绪")
    
    # 关节值变化处理 - 用户输入
    user_editing = [False]  # 标记用户是否正在编辑
    
    def on_joint_changed(joint_id, value):
        """用户修改关节值"""
        user_editing[0] = True
        if driver.is_servo_enabled():
            # 获取当前所有关节位置
            current = driver.get_positions()
            current[joint_id] = value
            driver.set_positions(current, velocity=50)  # 使用50%速度
        # 立即更新3D视图
        angles = window.control_panel.get_joint_values()
        robot_viewer.set_joint_angles(angles)
        # 更新TCP显示
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        window.update_tcp_display(tcp_pos, tcp_ori)
    
    def on_joint_jog_pressed(joint_id, direction):
        """点动按钮按下"""
        user_editing[0] = True
        if driver.is_servo_enabled():
            current = driver.get_positions()
            # 每次点动移动5度
            current[joint_id] += direction * 5
            driver.set_positions(current, velocity=30)
    
    def on_joint_jog_released(joint_id):
        """点动按钮释放"""
        # 停止运动
        if driver.is_servo_enabled():
            driver.stop()
    
    def on_tcp_jog_pressed(axis, direction):
        """TCP点动按钮按下"""
        user_editing[0] = True
        print(f"[DEBUG] TCP点动: axis={axis}, direction={direction}")  # 调试输出
        
        if not driver.is_servo_enabled():
            window.show_message("请先使能伺服", "orange")
            print("[DEBUG] 伺服未使能")
            return
        
        # 获取当前TCP位置
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        print(f"[DEBUG] 当前TCP位置: {tcp_pos}, 姿态: {tcp_ori}")
        
        # TCP点动步长
        pos_step = 10.0  # mm
        ori_step = 5.0   # 度
        
        # 根据轴调整位置
        axis_map = {'X': 0, 'Y': 1, 'Z': 2, 'Rx': 0, 'Ry': 1, 'Rz': 2}
        idx = axis_map.get(axis, 0)
        
        if axis in ['X', 'Y', 'Z']:
            tcp_pos[idx] += direction * pos_step
        else:
            tcp_ori[idx] += direction * ori_step
        
        print(f"[DEBUG] 目标TCP位置: {tcp_pos}, 姿态: {tcp_ori}")
        
        # 使用逆运动学计算关节角度
        from src.kinematics import RokaeSR5Kinematics
        kin = RokaeSR5Kinematics(use_simplified=True)
        current_joints = driver.get_positions()
        # 收敛条件: tolerance=0.5mm位置误差, max_iterations=500
        # 姿态误差在kinematics内部会乘以系数
        result_joints, converged = kin.inverse_kinematics(
            tcp_pos, tcp_ori, initial_guess=current_joints,
            max_iterations=500, tolerance=0.5
        )
        
        print(f"[DEBUG] 逆运动学: converged={converged}, joints={result_joints}")
        
        if converged:
            driver.set_positions(list(result_joints), velocity=30)
            # 立即更新3D视图和UI
            robot_viewer.set_joint_angles(list(result_joints))
            window.update_joint_display(list(result_joints))
            # 更新TCP显示
            new_tcp_pos, new_tcp_ori = robot_viewer.get_tcp_position()
            window.update_tcp_display(new_tcp_pos, new_tcp_ori)
            window.show_message(f"TCP {axis} 点动 {'+' if direction > 0 else '-'}")
        else:
            window.show_message("逆运动学求解失败", "red")
    
    def on_tcp_jog_released(axis):
        """TCP点动按钮释放"""
        if driver.is_servo_enabled():
            driver.stop()
    
    def on_tcp_position_changed(position, orientation):
        """TCP位置输入变化，执行运动"""
        user_editing[0] = True
        print(f"[DEBUG] TCP位置输入: pos={position}, ori={orientation}")
        
        if not driver.is_servo_enabled():
            window.show_message("请先使能伺服", "orange")
            return
        
        # 使用逆运动学计算关节角度
        from src.kinematics import RokaeSR5Kinematics
        kin = RokaeSR5Kinematics(use_simplified=True)
        current_joints = driver.get_positions()
        
        result_joints, converged = kin.inverse_kinematics(
            position, orientation, initial_guess=current_joints,
            max_iterations=500, tolerance=0.5
        )
        
        print(f"[DEBUG] 逆运动学: converged={converged}, joints={result_joints}")
        
        if converged:
            driver.set_positions(list(result_joints), velocity=50)
            robot_viewer.set_joint_angles(list(result_joints))
            window.update_joint_display(list(result_joints))
            # 更新TCP显示（实际到达位置）
            new_tcp_pos, new_tcp_ori = robot_viewer.get_tcp_position()
            window.update_tcp_display(new_tcp_pos, new_tcp_ori)
            window.show_message("TCP运动完成")
        else:
            window.show_message("逆运动学求解失败，目标位置可能超出工作空间", "red")
    
    window.control_panel.servo_enable_clicked.connect(on_servo_enable)
    window.control_panel.servo_disable_clicked.connect(on_servo_disable)
    window.control_panel.home_clicked.connect(on_home)
    window.control_panel.estop_clicked.connect(on_estop)
    window.control_panel.joint_value_changed.connect(on_joint_changed)
    window.control_panel.joint_jog_pressed.connect(on_joint_jog_pressed)
    window.control_panel.tcp_jog_pressed.connect(on_tcp_jog_pressed)
    window.control_panel.tcp_jog_released.connect(on_tcp_jog_released)
    window.control_panel.tcp_position_changed.connect(on_tcp_position_changed)
    window.control_panel.joint_jog_released.connect(on_joint_jog_released)
    
    # 缓存上次位置，避免重复渲染
    last_positions = [None]
    edit_cooldown = [0]  # 编辑冷却计数器
    
    # 更新定时器
    def update_display():
        # 编辑冷却机制：用户编辑后等待几个周期再允许自动更新
        if user_editing[0]:
            edit_cooldown[0] = 5  # 500ms冷却
            user_editing[0] = False
        
        if edit_cooldown[0] > 0:
            edit_cooldown[0] -= 1
            # 冷却期间只更新3D视图，不覆盖UI输入框
            positions = driver.get_positions()
            robot_viewer.set_joint_angles(positions)
            return
        
        # 如果驱动器在运动中，更新关节显示
        if driver.is_moving():
            positions = driver.get_positions()
            window.update_joint_display(positions)
            robot_viewer.set_joint_angles(positions)
            last_positions[0] = positions.copy()
            # 更新TCP显示
            tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
            window.update_tcp_display(tcp_pos, tcp_ori)
        else:
            # 不在运动时，同步显示
            positions = driver.get_positions()
            if last_positions[0] is None or positions != last_positions[0]:
                window.update_joint_display(positions)
                robot_viewer.set_joint_angles(positions)
                last_positions[0] = positions.copy()
                # 更新TCP显示
                tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
                window.update_tcp_display(tcp_pos, tcp_ori)
        
        # 检查回原点完成
        if state_machine.current_state.name == 'HOMING' and driver.is_homed():
            state_machine.homing_complete()
            window.show_message("回原点完成")
    
    update_timer = QTimer()
    update_timer.timeout.connect(update_display)
    update_timer.start(100)  # 100ms
    
    # 初始化状态
    state_machine.start_scan()
    state_machine.scan_complete()
    window.control_panel.set_connection_status(True)
    window.update_state_display(state_machine.current_state.name)
    
    # 显示窗口
    window.show()
    window.show_message("系统就绪")
    
    logger.info("系统启动完成")
    
    # 运行应用
    result = app.exec_()
    
    # 清理
    update_timer.stop()
    driver.disconnect()
    
    logger.info("系统已关闭")
    return result


if __name__ == "__main__":
    sys.exit(main())

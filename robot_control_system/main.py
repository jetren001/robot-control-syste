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
        
        # 先保存当前实际位置，用于恢复
        current_tcp_pos, current_tcp_ori = robot_viewer.get_tcp_position()
        print(f"[DEBUG] 当前实际TCP: pos={current_tcp_pos}, ori={current_tcp_ori}")
        
        if not driver.is_servo_enabled():
            window.show_message("请先使能伺服", "orange")
            # 恢复原值
            window.update_tcp_display(current_tcp_pos, current_tcp_ori)
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
            # 验证结果：用正运动学检查是否真的到达目标位置
            pose = kin.forward_kinematics(result_joints)
            test_pos = pose.position.tolist()
            test_ori = pose.euler_angles.tolist()
            pos_error = sum((a-b)**2 for a,b in zip(test_pos, position)) ** 0.5
            ori_error = sum((a-b)**2 for a,b in zip(test_ori, orientation)) ** 0.5
            print(f"[DEBUG] 验证误差: 位置={pos_error:.3f}mm, 姿态={ori_error:.3f}deg")
            
            if pos_error > 5.0 or ori_error > 5.0:
                # 误差过大，不执行
                window.show_message(f"目标位置误差过大(位置:{pos_error:.1f}mm,姿态:{ori_error:.1f}°)，不执行", "red")
                window.update_tcp_display(current_tcp_pos, current_tcp_ori)
                return
            
            driver.set_positions(list(result_joints), velocity=50)
            robot_viewer.set_joint_angles(list(result_joints))
            window.update_joint_display(list(result_joints))
            # 更新TCP显示（实际到达位置）
            new_tcp_pos, new_tcp_ori = robot_viewer.get_tcp_position()
            window.update_tcp_display(new_tcp_pos, new_tcp_ori)
            window.show_message(f"TCP运动完成 (误差:{pos_error:.2f}mm)")
        else:
            # IK不收敛，恢复原值
            window.show_message("目标位置无法到达，已恢复原值", "red")
            window.update_tcp_display(current_tcp_pos, current_tcp_ori)
    
    # 记录的点位列表
    recorded_points = []
    
    # 码垛相关状态
    palletizing_state = {
        'pick_point': None,  # 取料点 [joints]
        'pick_tcp': None,    # 取料点TCP [pos, ori]
        'place_point': None, # 放料起始点 [joints]
        'place_tcp': None,   # 放料起始点TCP [pos, ori]
        'running': False,    # 是否运行中
        'paused': False,     # 是否暂停
        'current_index': 0,  # 当前执行索引
        'total_points': 0,   # 总点位数
        'path': [],          # 码垛路径 [(action, joints), ...]
        'step': 0,           # 当前步骤 (0=取料, 1=放料)
    }
    
    def on_joints_zero():
        """全部归零 - 移动机器人到零位"""
        if not driver.is_servo_enabled():
            window.show_message("请先使能伺服", "orange")
            return
        # 设置所有关节到零位
        zero_positions = [0.0] * 6
        driver.set_positions(zero_positions, velocity=50)
        robot_viewer.set_joint_angles(zero_positions)
        window.update_joint_display(zero_positions)
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        window.update_tcp_display(tcp_pos, tcp_ori)
        window.show_message("已移动到零位")
    
    def on_record_point(joints):
        """记录当前点位"""
        recorded_points.append(joints.copy())
        window.show_message(f"已记录点位 #{len(recorded_points)}: {[f'{j:.1f}' for j in joints]}")
        print(f"[DEBUG] 记录点位: {joints}, 总计: {len(recorded_points)}个点位")
    
    def on_teach_pick():
        """示教取料点"""
        joints = driver.get_positions()
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        palletizing_state['pick_point'] = joints.copy()
        palletizing_state['pick_tcp'] = [tcp_pos.copy(), tcp_ori.copy()]
        window.control_panel.set_pick_point(tcp_pos)
        # 更新3D场景中的取料位置
        robot_viewer.set_pick_position(tcp_pos)
        window.show_message(f"取料点已设置: X={tcp_pos[0]:.1f} Y={tcp_pos[1]:.1f} Z={tcp_pos[2]:.1f}")
        logger.info(f"取料点设置: TCP={tcp_pos}, 关节={joints}")
    
    def on_teach_place():
        """示教放料起始点"""
        joints = driver.get_positions()
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        palletizing_state['place_point'] = joints.copy()
        palletizing_state['place_tcp'] = [tcp_pos.copy(), tcp_ori.copy()]
        window.control_panel.set_place_point(tcp_pos)
        # 更新3D场景中的放料位置
        robot_viewer.set_place_position(tcp_pos)
        window.show_message(f"放料点已设置: X={tcp_pos[0]:.1f} Y={tcp_pos[1]:.1f} Z={tcp_pos[2]:.1f}")
        logger.info(f"放料点设置: TCP={tcp_pos}, 关节={joints}")
    
    def on_drag_teaching_changed(enabled):
        """拖动示教开关"""
        robot_viewer.enable_drag_teaching(enabled)
        if enabled:
            window.show_message("拖动示教已启用 - 按P键后点击3D空间设置目标位置")
        else:
            window.show_message("拖动示教已禁用")
    
    # 拖动示教的IK缓存和节流
    drag_ik_cache = {'last_time': 0, 'last_pos': None}
    
    def on_tcp_dragged(target_pos, target_ori):
        """处理拖动示教 - 实时移动机器人到目标位置"""
        import time
        
        if not driver.is_servo_enabled():
            return  # 拖动时不弹消息，避免干扰
        
        # 节流：限制IK计算频率 (最多20Hz)
        current_time = time.time()
        if current_time - drag_ik_cache['last_time'] < 0.05:
            return
        drag_ik_cache['last_time'] = current_time
        
        # 使用逆运动学计算关节角度
        from src.kinematics import RokaeSR5Kinematics
        kin = RokaeSR5Kinematics(use_simplified=True)
        current_joints = driver.get_positions()
        
        # 快速IK (减少迭代次数以提高响应速度)
        result_joints, converged = kin.inverse_kinematics(
            target_pos, target_ori, initial_guess=current_joints,
            max_iterations=50, tolerance=5.0  # 放宽精度以加速
        )
        
        if converged:
            driver.set_positions(list(result_joints), velocity=100)
            robot_viewer.set_joint_angles(list(result_joints))
            window.update_joint_display(list(result_joints))
            # 更新TCP显示
            new_tcp_pos, new_tcp_ori = robot_viewer.get_tcp_position()
            window.update_tcp_display(new_tcp_pos, new_tcp_ori)
    
    def generate_palletizing_path():
        """生成码垛路径"""
        config = window.control_panel.get_palletizing_config()
        base_tcp = palletizing_state['place_tcp']
        base_pos, base_ori = base_tcp[0], base_tcp[1]
        
        from src.kinematics import RokaeSR5Kinematics
        kin = RokaeSR5Kinematics(use_simplified=True)
        
        path = []
        for layer in range(config['layers']):
            for row in range(config['rows']):
                for col in range(config['cols']):
                    # 计算放料位置
                    place_pos = [
                        base_pos[0] + col * config['x_spacing'],
                        base_pos[1] + row * config['y_spacing'],
                        base_pos[2] + layer * config['z_spacing']
                    ]
                    # 计算逆运动学
                    current_joints = palletizing_state['place_point']
                    result_joints, converged = kin.inverse_kinematics(
                        place_pos, base_ori, initial_guess=current_joints,
                        max_iterations=200, tolerance=1.0
                    )
                    if converged:
                        path.append({
                            'index': len(path),
                            'pick_joints': palletizing_state['pick_point'],
                            'place_joints': list(result_joints),
                            'place_pos': place_pos
                        })
        return path
    
    def execute_palletizing_step():
        """执行码垛单步"""
        if not palletizing_state['running'] or palletizing_state['paused']:
            return
        
        idx = palletizing_state['current_index']
        step = palletizing_state['step']
        path = palletizing_state['path']
        
        if idx >= len(path):
            # 完成
            palletizing_state['running'] = False
            state_machine.stop_program()
            window.show_message("码垛程序完成!")
            window.control_panel.set_progress(len(path), len(path))
            return
        
        point = path[idx]
        
        if step == 0:
            # 移动到取料点
            joints = point['pick_joints']
            driver.set_positions(joints, velocity=80)
            robot_viewer.set_joint_angles(joints)
            window.update_joint_display(joints)
            palletizing_state['step'] = 1
            window.show_message(f"[{idx+1}/{len(path)}] 移动到取料点...")
        elif step == 1:
            # 抓取箱子
            robot_viewer.pick_box()
            palletizing_state['step'] = 2
            window.show_message(f"[{idx+1}/{len(path)}] 抓取箱子...")
        elif step == 2:
            # 移动到放料点
            joints = point['place_joints']
            driver.set_positions(joints, velocity=80)
            robot_viewer.set_joint_angles(joints)
            window.update_joint_display(joints)
            palletizing_state['step'] = 3
            window.show_message(f"[{idx+1}/{len(path)}] 移动到放料点...")
        else:
            # 放置箱子 - 使用当前TCP真实位置，不使用预设坐标
            tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
            robot_viewer.place_box()  # 不传参数，使用当前真实TCP位置
            palletizing_state['step'] = 0
            palletizing_state['current_index'] += 1
            window.control_panel.set_progress(idx + 1, len(path))
            window.show_message(f"[{idx+1}/{len(path)}] 放置完成! TCP:[{tcp_pos[0]:.0f},{tcp_pos[1]:.0f},{tcp_pos[2]:.0f}]")
        
        # 更新TCP显示
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        window.update_tcp_display(tcp_pos, tcp_ori)
    
    # 码垛执行定时器
    palletizing_timer = QTimer()
    palletizing_timer.timeout.connect(execute_palletizing_step)
    
    def on_program_start():
        """启动码垛程序"""
        logger.info("=== on_program_start 被调用 ===")
        
        # 检查运行模式
        current_mode = window.control_panel.mode_combo.currentText()
        if current_mode == "手动":
            window.show_message("请先切换到自动模式", "orange")
            return
        
        # 检查前置条件
        if not driver.is_servo_enabled():
            window.show_message("请先使能伺服", "orange")
            return
        
        if state_machine.current_state.name not in ['STANDBY', 'ENABLED']:
            window.show_message(f"当前状态 {state_machine.current_state.name} 无法启动程序", "orange")
            return
        
        if not driver.is_homed():
            window.show_message("请先回原点", "orange")
            return
        
        # 检查示教点
        if palletizing_state['pick_point'] is None:
            window.show_message("请先示教取料点", "orange")
            return
        if palletizing_state['place_point'] is None:
            window.show_message("请先示教放料点", "orange")
            return
        
        # 生成路径
        path = generate_palletizing_path()
        if not path:
            window.show_message("无法生成码垛路径", "red")
            return
        
        palletizing_state['path'] = path
        palletizing_state['current_index'] = 0
        palletizing_state['step'] = 0
        palletizing_state['running'] = True
        palletizing_state['paused'] = False
        palletizing_state['total_points'] = len(path)
        
        config = window.control_panel.get_palletizing_config()
        window.show_message(f"码垛启动: {config['rows']}x{config['cols']}x{config['layers']}={len(path)}个点位")
        window.control_panel.set_progress(0, len(path))
        state_machine.start_program()
        window.control_panel.set_mode_status("auto")
        
        # 启动执行定时器 (每500ms执行一步)
        palletizing_timer.start(500)
        logger.info(f"码垛程序启动, 共{len(path)}个点位")
    
    def on_program_stop():
        """停止码垛程序"""
        palletizing_state['running'] = False
        palletizing_state['paused'] = False
        palletizing_timer.stop()
        state_machine.stop_program()
        robot_viewer.clear_placed_boxes()
        window.show_message("码垛程序已停止")
    
    def on_program_pause():
        """暂停/恢复码垛程序"""
        if palletizing_state['paused']:
            palletizing_state['paused'] = False
            window.show_message("码垛程序已恢复")
        else:
            palletizing_state['paused'] = True
            window.show_message("码垛程序已暂停")
    
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
    window.control_panel.joints_zero_clicked.connect(on_joints_zero)
    window.control_panel.record_point_clicked.connect(on_record_point)
    window.control_panel.teach_pick_clicked.connect(on_teach_pick)
    window.control_panel.teach_place_clicked.connect(on_teach_place)
    window.control_panel.drag_teaching_changed.connect(on_drag_teaching_changed)
    robot_viewer.tcp_dragged.connect(on_tcp_dragged)
    window.control_panel.program_start_clicked.connect(on_program_start)
    window.control_panel.program_stop_clicked.connect(on_program_stop)
    window.control_panel.program_pause_clicked.connect(on_program_pause)
    
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
    update_timer.start(200)  # 200ms (5 FPS，减少卡顿)
    
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

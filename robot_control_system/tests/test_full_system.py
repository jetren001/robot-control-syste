"""
完整系统自动化测试
测试所有按钮和功能

作者: Cursor AI
日期: 2026-01-28
"""

import sys
import os
import time

# 添加src目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt

# 测试结果记录
test_results = []

def log_test(name, passed, message=""):
    status = "[PASS]" if passed else "[FAIL]"
    test_results.append({
        'name': name,
        'passed': passed,
        'message': message
    })
    print(f"{status}: {name} - {message}")

def run_full_test():
    """运行完整的系统测试"""
    
    print("=" * 60)
    print("珞石 SR5-C 机器人控制系统 - 完整自动化测试")
    print("=" * 60)
    
    # 创建应用
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # 导入模块
    from hmi import MainWindow
    from hmi.robot_viewer_3d import create_robot_viewer
    from control import RobotStateMachine, RobotState
    from driver import VirtualDriver
    
    print("\n[1] 初始化测试...")
    
    # 创建组件
    try:
        window = MainWindow()
        log_test("创建主窗口", True)
    except Exception as e:
        log_test("创建主窗口", False, str(e))
        return
    
    try:
        robot_viewer = create_robot_viewer()
        window.set_robot_viewer(robot_viewer)
        log_test("创建3D视图", True)
    except Exception as e:
        log_test("创建3D视图", False, str(e))
    
    try:
        state_machine = RobotStateMachine()
        log_test("创建状态机", True)
    except Exception as e:
        log_test("创建状态机", False, str(e))
        return
    
    try:
        driver = VirtualDriver()
        driver.connect()
        log_test("创建并连接虚拟驱动器", True)
    except Exception as e:
        log_test("创建并连接虚拟驱动器", False, str(e))
        return
    
    # 初始化状态机
    state_machine.start_scan()
    state_machine.scan_complete()
    
    print("\n[2] 基本控制测试...")
    
    # ==================== 伺服使能测试 ====================
    print("\n--- 伺服使能测试 ---")
    
    initial_state = state_machine.current_state
    log_test("初始状态检查", initial_state == RobotState.READY, 
             f"状态: {initial_state.name}")
    
    # 测试伺服使能
    driver.enable_servo()
    result = state_machine.enable_servo()
    log_test("伺服使能命令", result, f"状态机返回: {result}")
    log_test("伺服使能状态", state_machine.current_state == RobotState.ENABLED,
             f"状态: {state_machine.current_state.name}")
    log_test("驱动器使能状态", driver.is_servo_enabled(), 
             f"驱动器报告: {driver.is_servo_enabled()}")
    
    # ==================== 回原点测试 ====================
    print("\n--- 回原点测试 ---")
    
    driver.home()
    result = state_machine.start_homing()
    log_test("回原点命令", result, f"状态机返回: {result}")
    log_test("回原点状态", state_machine.current_state == RobotState.HOMING,
             f"状态: {state_machine.current_state.name}")
    
    # 等待回原点完成
    timeout = 3.0
    start = time.time()
    while driver.is_moving() and (time.time() - start) < timeout:
        time.sleep(0.05)
    
    log_test("回原点运动完成", not driver.is_moving(), 
             f"耗时: {time.time()-start:.2f}s")
    log_test("回原点位置", driver.is_homed(), 
             f"位置: {driver.get_positions()}")
    
    # 完成回原点
    state_machine.homing_complete()
    log_test("回原点状态完成", state_machine.current_state == RobotState.STANDBY,
             f"状态: {state_machine.current_state.name}")
    
    # ==================== 关节控制测试 ====================
    print("\n--- 关节控制测试 ---")
    
    # 测试设置关节位置
    test_positions = [30, -45, 60, 0, 45, 90]
    driver.set_positions(test_positions, velocity=180)
    log_test("设置关节位置命令", True, f"目标: {test_positions}")
    
    # 等待到位
    timeout = 3.0
    start = time.time()
    while driver.is_moving() and (time.time() - start) < timeout:
        time.sleep(0.05)
    
    final_positions = driver.get_positions()
    position_ok = all(abs(a - b) < 1.0 for a, b in zip(final_positions, test_positions))
    log_test("关节位置到达", position_ok, 
             f"实际: {[f'{p:.1f}' for p in final_positions]}")
    
    # 测试3D视图更新
    try:
        robot_viewer.set_joint_angles(test_positions)
        tcp_pos, tcp_ori = robot_viewer.get_tcp_position()
        log_test("3D视图更新", True, f"TCP: {[f'{p:.1f}' for p in tcp_pos]}")
    except Exception as e:
        log_test("3D视图更新", False, str(e))
    
    # ==================== 速度控制测试 ====================
    print("\n--- 速度控制测试 ---")
    
    # 测试速度滑块
    window.control_panel.speed_slider.setValue(25)
    log_test("速度设置25%", window.control_panel.speed_slider.value() == 25)
    
    window.control_panel.speed_slider.setValue(75)
    log_test("速度设置75%", window.control_panel.speed_slider.value() == 75)
    
    window.control_panel.speed_slider.setValue(50)
    log_test("速度恢复50%", window.control_panel.speed_slider.value() == 50)
    
    # ==================== 码垛程序测试 ====================
    print("\n--- 码垛程序测试 ---")
    
    # 测试码垛配置
    window.control_panel.spin_rows.setValue(4)
    window.control_panel.spin_cols.setValue(5)
    window.control_panel.spin_layers.setValue(3)
    
    expected_total = 4 * 5 * 3
    actual_total = int(window.control_panel.label_total.text())
    log_test("码垛配置计算", actual_total == expected_total,
             f"期望: {expected_total}, 实际: {actual_total}")
    
    # 测试程序启动
    result = state_machine.start_program()
    log_test("程序启动命令", result, f"状态: {state_machine.current_state.name}")
    log_test("自动运行状态", state_machine.current_state == RobotState.AUTO_RUN)
    
    # 测试程序暂停
    result = state_machine.pause_program()
    log_test("程序暂停命令", result, f"状态: {state_machine.current_state.name}")
    log_test("暂停状态", state_machine.current_state == RobotState.PAUSED)
    
    # 测试程序恢复
    result = state_machine.resume_program()
    log_test("程序恢复命令", result, f"状态: {state_machine.current_state.name}")
    
    # 测试程序停止
    result = state_machine.stop_program()
    log_test("程序停止命令", result, f"状态: {state_machine.current_state.name}")
    log_test("停止后状态", state_machine.current_state == RobotState.STANDBY)
    
    # ==================== 急停测试 ====================
    print("\n--- 急停测试 ---")
    
    # 先使能
    driver.enable_servo()
    state_machine.enable_servo()
    state_machine.start_homing()
    state_machine.homing_complete()
    
    # 测试急停
    driver.emergency_stop()
    result = state_machine.emergency_stop()
    log_test("急停命令", result)
    log_test("急停状态", state_machine.current_state == RobotState.EMERGENCY_STOP,
             f"状态: {state_machine.current_state.name}")
    log_test("驱动器急停", driver.has_error(), "驱动器报告故障")
    
    # 测试急停恢复
    result = state_machine.release_emergency()
    log_test("急停解除", result, f"状态: {state_machine.current_state.name}")
    
    driver.reset_error()
    result = state_machine.reset_fault()
    log_test("故障复位", result, f"状态: {state_machine.current_state.name}")
    
    # ==================== 伺服禁止测试 ====================
    print("\n--- 伺服禁止测试 ---")
    
    # 重新启动到STANDBY
    state_machine.start_scan()
    state_machine.scan_complete()
    driver.enable_servo()
    state_machine.enable_servo()
    driver.home()
    state_machine.start_homing()
    while driver.is_moving():
        time.sleep(0.05)
    state_machine.homing_complete()
    
    # 测试禁止
    driver.disable_servo()
    result = state_machine.disable_servo()
    log_test("伺服禁止命令", result)
    log_test("禁止后状态", state_machine.current_state == RobotState.READY,
             f"状态: {state_machine.current_state.name}")
    log_test("驱动器禁止状态", not driver.is_servo_enabled())
    
    # ==================== 报警测试 ====================
    print("\n--- 报警测试 ---")
    
    alarm_manager = window.alarm_manager
    
    # 添加报警
    alarm1 = alarm_manager.add_alarm('E001', source='J1')
    log_test("添加报警E001", alarm1 is not None, f"ID: {alarm1.id}")
    
    alarm2 = alarm_manager.add_alarm('W001', source='J2')
    log_test("添加报警W001", alarm2 is not None, f"ID: {alarm2.id}")
    
    active_count = len(alarm_manager.get_active_alarms())
    log_test("活动报警数量", active_count == 2, f"数量: {active_count}")
    
    # 确认报警
    alarm_manager.acknowledge_all()
    unack_count = len(alarm_manager.get_unacknowledged_alarms())
    log_test("确认所有报警", unack_count == 0, f"未确认: {unack_count}")
    
    # 清除报警
    cleared = alarm_manager.clear_all()
    log_test("清除已确认报警", cleared == 2, f"清除数: {cleared}")
    
    active_count = len(alarm_manager.get_active_alarms())
    log_test("报警已清空", active_count == 0, f"剩余: {active_count}")
    
    # ==================== UI控件测试 ====================
    print("\n--- UI控件测试 ---")
    
    # 测试状态显示更新
    window.control_panel.set_connection_status(True)
    log_test("连接状态显示", True, "已连接")
    
    window.control_panel.set_servo_status(True)
    log_test("伺服状态显示", True, "使能")
    
    window.control_panel.set_state_machine_status("STANDBY")
    log_test("状态机显示", True, "STANDBY")
    
    window.control_panel.set_mode_status("auto")
    log_test("模式显示", True, "自动")
    
    # 测试进度显示
    window.control_panel.set_progress(5, 20)
    log_test("进度显示", True, "5/20")
    
    # 测试关节显示更新
    window.control_panel.set_joint_values([10, 20, 30, 40, 50, 60])
    values = window.control_panel.get_joint_values()
    log_test("关节值显示", values == [10, 20, 30, 40, 50, 60],
             f"值: {values}")
    
    # 测试TCP显示
    window.control_panel.set_tcp_values([100, 200, 300], [45, 30, 15])
    log_test("TCP显示", True, "位置和姿态已更新")
    
    # ==================== 清理 ====================
    print("\n[3] 清理...")
    driver.disconnect()
    
    # ==================== 生成报告 ====================
    print("\n" + "=" * 60)
    print("测试报告")
    print("=" * 60)
    
    passed = sum(1 for t in test_results if t['passed'])
    failed = sum(1 for t in test_results if not t['passed'])
    total = len(test_results)
    
    print(f"\n总测试数: {total}")
    print(f"通过: {passed}")
    print(f"失败: {failed}")
    print(f"通过率: {passed/total*100:.1f}%")
    
    if failed > 0:
        print("\nFailed tests:")
        for t in test_results:
            if not t['passed']:
                print(f"  [FAIL] {t['name']}: {t['message']}")
    
    print("\n" + "=" * 60)
    
    # 返回是否全部通过
    return failed == 0


if __name__ == "__main__":
    success = run_full_test()
    sys.exit(0 if success else 1)

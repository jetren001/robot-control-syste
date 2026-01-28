"""
仿真测试模块
测试整个系统的集成功能

作者: Cursor AI
日期: 2026-01-28
"""

import sys
import os
import unittest
import time
import logging

# 添加src目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from kinematics import RokaeSR5Kinematics
from control import RobotState, RobotStateMachine
from path_planning import PalletConfig, PalletizingPlanner, TrajectoryGenerator
from driver import VirtualDriver
from hmi import AlarmManager, AlarmSeverity

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestKinematics(unittest.TestCase):
    """运动学测试"""
    
    def setUp(self):
        self.kin = RokaeSR5Kinematics(use_simplified=True)
    
    def test_forward_kinematics_zero(self):
        """测试零位正运动学"""
        pose = self.kin.forward_kinematics([0, 0, 0, 0, 0, 0])
        self.assertIsNotNone(pose)
        self.assertEqual(len(pose.position), 3)
    
    def test_forward_kinematics_various(self):
        """测试不同姿态正运动学"""
        test_cases = [
            [30, -45, 60, 0, 45, 0],
            [0, 0, 90, 0, 0, 0],
            [-30, 30, -30, 30, -30, 30],
        ]
        for angles in test_cases:
            pose = self.kin.forward_kinematics(angles)
            self.assertIsNotNone(pose)
    
    def test_inverse_kinematics(self):
        """测试逆运动学"""
        # 先正运动学得到目标位置
        original_angles = [20, -30, 45, 0, 30, 0]
        pose = self.kin.forward_kinematics(original_angles)
        
        # 逆运动学求解
        result_angles, converged = self.kin.inverse_kinematics(
            pose.position.tolist(),
            pose.euler_angles.tolist(),
            initial_guess=[0, 0, 0, 0, 0, 0]
        )
        
        self.assertTrue(converged)
        
        # 验证结果
        verify_pose = self.kin.forward_kinematics(result_angles)
        import numpy as np
        pos_error = np.linalg.norm(pose.position - verify_pose.position)
        self.assertLess(pos_error, 1.0)  # 误差小于1mm
    
    def test_jacobian(self):
        """测试雅可比矩阵"""
        J = self.kin.jacobian([0, 0, 0, 0, 0, 0])
        self.assertEqual(J.shape, (6, 6))
    
    def test_joint_positions(self):
        """测试关节位置获取"""
        positions = self.kin.get_joint_positions([0, 0, 0, 0, 0, 0])
        self.assertEqual(len(positions), 7)  # 基座 + 6个关节


class TestStateMachine(unittest.TestCase):
    """状态机测试"""
    
    def setUp(self):
        self.sm = RobotStateMachine()
    
    def test_initial_state(self):
        """测试初始状态"""
        self.assertEqual(self.sm.current_state, RobotState.IDLE)
    
    def test_normal_startup_sequence(self):
        """测试正常启动流程"""
        # IDLE -> SCANNING
        self.assertTrue(self.sm.start_scan())
        self.assertEqual(self.sm.current_state, RobotState.SCANNING)
        
        # SCANNING -> READY
        self.assertTrue(self.sm.scan_complete())
        self.assertEqual(self.sm.current_state, RobotState.READY)
        
        # READY -> ENABLED
        self.assertTrue(self.sm.enable_servo())
        self.assertEqual(self.sm.current_state, RobotState.ENABLED)
        
        # ENABLED -> HOMING
        self.assertTrue(self.sm.start_homing())
        self.assertEqual(self.sm.current_state, RobotState.HOMING)
        
        # HOMING -> STANDBY
        self.assertTrue(self.sm.homing_complete())
        self.assertEqual(self.sm.current_state, RobotState.STANDBY)
    
    def test_fault_handling(self):
        """测试故障处理"""
        self.sm.start_scan()
        self.sm.scan_complete()
        
        # 报告故障
        self.assertTrue(self.sm.report_fault("测试故障"))
        self.assertEqual(self.sm.current_state, RobotState.FAULT)
        
        # 复位
        self.assertTrue(self.sm.reset_fault())
        self.assertEqual(self.sm.current_state, RobotState.IDLE)
    
    def test_emergency_stop(self):
        """测试急停"""
        self.sm.start_scan()
        self.sm.scan_complete()
        self.sm.enable_servo()
        
        # 急停
        self.assertTrue(self.sm.emergency_stop())
        self.assertEqual(self.sm.current_state, RobotState.EMERGENCY_STOP)
        
        # 解除急停
        self.assertTrue(self.sm.release_emergency())
        self.assertEqual(self.sm.current_state, RobotState.FAULT)


class TestPalletizing(unittest.TestCase):
    """码垛规划测试"""
    
    def setUp(self):
        self.config = PalletConfig(rows=3, cols=3, layers=2)
        self.planner = PalletizingPlanner(self.config)
    
    def test_pallet_point_count(self):
        """测试码垛点数量"""
        expected = 3 * 3 * 2  # 18个点
        self.assertEqual(self.planner.get_total_count(), expected)
    
    def test_pallet_point_positions(self):
        """测试码垛点位置"""
        points = self.planner.get_pallet_points()
        
        # 第一个点应该在原点附近
        first_point = points[0]
        self.assertEqual(first_point.row, 0)
        self.assertEqual(first_point.col, 0)
        self.assertEqual(first_point.layer, 0)
    
    def test_trajectory_generation(self):
        """测试轨迹生成"""
        import numpy as np
        current_pos = np.array([300, 0, 300])
        cycle = self.planner.generate_full_cycle(0, current_pos)
        
        self.assertGreater(len(cycle), 0)
        
        # 检查轨迹包含必要步骤
        actions = [step['type'] for step in cycle]
        self.assertIn('linear', actions)
        self.assertIn('gripper', actions)


class TestTrajectory(unittest.TestCase):
    """轨迹生成测试"""
    
    def setUp(self):
        self.gen = TrajectoryGenerator(dt=0.01)
    
    def test_linear_interpolation(self):
        """测试线性插补"""
        import numpy as np
        start = np.array([0, 0, 0])
        end = np.array([100, 50, 30])
        
        traj = self.gen.linear_interpolation(start, end, velocity=50, acceleration=100)
        
        self.assertGreater(len(traj), 0)
        self.assertTrue(np.allclose(traj[0].position, start))
        self.assertTrue(np.allclose(traj[-1].position, end, atol=1e-3))
    
    def test_joint_interpolation(self):
        """测试关节插补"""
        import numpy as np
        start = np.zeros(6)
        end = np.array([30, -45, 60, 0, 45, 90])
        
        traj = self.gen.joint_interpolation(start, end, max_velocity=100, max_acceleration=200)
        
        self.assertGreater(len(traj), 0)
        self.assertTrue(np.allclose(traj[0].position, start))
        self.assertTrue(np.allclose(traj[-1].position, end, atol=1e-3))


class TestVirtualDriver(unittest.TestCase):
    """虚拟驱动器测试"""
    
    def setUp(self):
        self.driver = VirtualDriver()
    
    def tearDown(self):
        if self.driver.is_connected():
            self.driver.disconnect()
    
    def test_connect_disconnect(self):
        """测试连接断开"""
        self.assertFalse(self.driver.is_connected())
        
        self.assertTrue(self.driver.connect())
        self.assertTrue(self.driver.is_connected())
        
        self.driver.disconnect()
        self.assertFalse(self.driver.is_connected())
    
    def test_scan_devices(self):
        """测试设备扫描"""
        self.driver.connect()
        devices = self.driver.scan_devices()
        
        self.assertEqual(len(devices), 7)  # 6轴 + 1个IO模块
    
    def test_servo_enable_disable(self):
        """测试伺服使能禁止"""
        self.driver.connect()
        
        self.assertTrue(self.driver.enable_servo())
        self.assertTrue(self.driver.is_servo_enabled())
        
        self.assertTrue(self.driver.disable_servo())
        self.assertFalse(self.driver.is_servo_enabled())
    
    def test_position_control(self):
        """测试位置控制"""
        self.driver.connect()
        self.driver.enable_servo()
        
        target = [10, -20, 30, 0, 15, 45]
        self.driver.set_positions(target, velocity=180)
        
        # 等待到位
        timeout = 5.0
        start_time = time.time()
        while self.driver.is_moving() and (time.time() - start_time) < timeout:
            time.sleep(0.05)
        
        positions = self.driver.get_positions()
        for i, (actual, expected) in enumerate(zip(positions, target)):
            self.assertAlmostEqual(actual, expected, places=0)
    
    def test_emergency_stop(self):
        """测试急停"""
        self.driver.connect()
        self.driver.enable_servo()
        
        self.assertTrue(self.driver.emergency_stop())
        self.assertTrue(self.driver.has_error())
        self.assertFalse(self.driver.is_servo_enabled())


class TestAlarmManager(unittest.TestCase):
    """报警管理器测试"""
    
    def setUp(self):
        self.manager = AlarmManager()
    
    def test_add_alarm(self):
        """测试添加报警"""
        alarm = self.manager.add_alarm('E001', source='J1')
        
        self.assertEqual(alarm.code, 'E001')
        self.assertEqual(alarm.source, 'J1')
        self.assertEqual(len(self.manager.get_active_alarms()), 1)
    
    def test_acknowledge_alarm(self):
        """测试确认报警"""
        alarm = self.manager.add_alarm('E001')
        
        self.assertTrue(self.manager.acknowledge_alarm(alarm.id))
        
        unack = self.manager.get_unacknowledged_alarms()
        self.assertEqual(len(unack), 0)
    
    def test_clear_alarm(self):
        """测试清除报警"""
        alarm = self.manager.add_alarm('E001')
        self.manager.acknowledge_alarm(alarm.id)
        
        cleared = self.manager.clear_all()
        self.assertEqual(cleared, 1)
        self.assertEqual(len(self.manager.get_active_alarms()), 0)


class TestIntegration(unittest.TestCase):
    """集成测试"""
    
    def test_full_palletizing_cycle(self):
        """测试完整码垛流程"""
        import numpy as np
        
        # 初始化组件
        kin = RokaeSR5Kinematics(use_simplified=True)
        sm = RobotStateMachine()
        driver = VirtualDriver()
        planner = PalletizingPlanner(PalletConfig(rows=2, cols=2, layers=1))
        
        # 连接驱动器
        driver.connect()
        
        # 状态机启动流程
        sm.start_scan()
        sm.scan_complete()
        sm.enable_servo()
        sm.start_homing()
        
        # 虚拟驱动器使能和回原点
        driver.enable_servo()
        driver.home()
        
        # 等待回原点
        while driver.is_moving():
            time.sleep(0.05)
        
        sm.homing_complete()
        
        # 验证状态
        self.assertEqual(sm.current_state, RobotState.STANDBY)
        self.assertTrue(driver.is_homed())
        
        # 启动自动程序
        sm.start_program()
        self.assertEqual(sm.current_state, RobotState.AUTO_RUN)
        
        # 执行一个码垛循环
        total = planner.get_total_count()
        logger.info(f"开始码垛测试, 共{total}个点位")
        
        for i in range(min(2, total)):  # 只测试2个点
            point = planner.get_pallet_point(i)
            logger.info(f"执行点位 {i+1}/{total}: {point}")
            
            # 运动学计算
            pose = kin.forward_kinematics(driver.get_positions())
            
            # 这里可以添加更多测试...
        
        # 停止程序
        sm.stop_program()
        self.assertEqual(sm.current_state, RobotState.STANDBY)
        
        # 断开连接
        driver.disconnect()
        
        logger.info("集成测试完成")


def run_simulation_test():
    """运行仿真测试"""
    print("=" * 60)
    print("珞石 SR5-C 机器人控制系统 - 仿真测试")
    print("=" * 60)
    
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试
    suite.addTests(loader.loadTestsFromTestCase(TestKinematics))
    suite.addTests(loader.loadTestsFromTestCase(TestStateMachine))
    suite.addTests(loader.loadTestsFromTestCase(TestPalletizing))
    suite.addTests(loader.loadTestsFromTestCase(TestTrajectory))
    suite.addTests(loader.loadTestsFromTestCase(TestVirtualDriver))
    suite.addTests(loader.loadTestsFromTestCase(TestAlarmManager))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 输出总结
    print("\n" + "=" * 60)
    print(f"测试总数: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    print("=" * 60)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_simulation_test()
    sys.exit(0 if success else 1)

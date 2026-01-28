"""
虚拟驱动模块
模拟真实伺服驱动器的行为，用于仿真测试

作者: Cursor AI
日期: 2026-01-28
"""

import numpy as np
from typing import List, Optional, Dict
import threading
import time
import logging

from .interface import DriverInterface, AxisState, AxisInfo

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class VirtualAxis:
    """虚拟轴"""
    
    def __init__(self, axis_id: int, min_pos: float = -180, max_pos: float = 180):
        self.axis_id = axis_id
        self.min_pos = min_pos
        self.max_pos = max_pos
        
        # 状态
        self.state = AxisState.NOT_READY
        self.enabled = False
        self.homed = False
        
        # 位置和速度
        self.current_position = 0.0
        self.target_position = 0.0
        self.velocity = 0.0
        self.max_velocity = 180.0  # 度/秒
        self.acceleration = 360.0  # 度/秒^2
        
        # 运动状态
        self.is_moving = False
        self.in_position = True
        self.position_tolerance = 0.1  # 度
        
        # 错误
        self.error_code = 0
        self.torque = 0.0
    
    def enable(self) -> bool:
        if self.error_code != 0:
            return False
        self.enabled = True
        self.state = AxisState.ENABLED
        return True
    
    def disable(self) -> bool:
        self.enabled = False
        self.state = AxisState.READY if self.error_code == 0 else AxisState.FAULT
        return True
    
    def set_target(self, position: float, velocity: Optional[float] = None):
        """设置目标位置"""
        # 检查限位
        position = np.clip(position, self.min_pos, self.max_pos)
        self.target_position = position
        
        if velocity is not None:
            self.max_velocity = min(velocity, 180.0)
        
        self.in_position = False
        self.is_moving = True
        self.state = AxisState.MOVING
    
    def update(self, dt: float):
        """更新位置 (仿真循环调用)"""
        if not self.enabled or not self.is_moving:
            return
        
        # 计算位置差
        error = self.target_position - self.current_position
        
        if abs(error) < self.position_tolerance:
            # 到位
            self.current_position = self.target_position
            self.velocity = 0.0
            self.is_moving = False
            self.in_position = True
            self.state = AxisState.ENABLED
            return
        
        # 简化的运动模型 (一阶响应)
        direction = np.sign(error)
        max_step = self.max_velocity * dt
        
        if abs(error) > max_step:
            self.current_position += direction * max_step
            self.velocity = direction * self.max_velocity
        else:
            self.current_position = self.target_position
            self.velocity = 0.0
        
        # 模拟扭矩
        self.torque = abs(self.velocity) / self.max_velocity * 30  # 简化模型
    
    def home(self):
        """回原点"""
        self.set_target(0.0)
        self.homed = True
    
    def reset_error(self):
        """复位错误"""
        self.error_code = 0
        if self.enabled:
            self.state = AxisState.ENABLED
        else:
            self.state = AxisState.READY
    
    def inject_fault(self, error_code: int):
        """注入故障 (用于测试)"""
        self.error_code = error_code
        self.state = AxisState.FAULT
        self.enabled = False
        self.is_moving = False
    
    def get_info(self) -> AxisInfo:
        """获取轴信息"""
        return AxisInfo(
            axis_id=self.axis_id,
            state=self.state,
            position=self.current_position,
            velocity=self.velocity,
            torque=self.torque,
            error_code=self.error_code,
            enabled=self.enabled,
            in_position=self.in_position
        )


class VirtualDriver(DriverInterface):
    """
    虚拟驱动器
    模拟6轴机器人的伺服驱动行为
    """
    
    def __init__(self):
        # 创建6个虚拟轴
        self.axes: List[VirtualAxis] = []
        joint_limits = [
            (-170, 170),   # J1
            (-120, 120),   # J2
            (-170, 170),   # J3
            (-120, 120),   # J4
            (-170, 170),   # J5
            (-360, 360),   # J6
        ]
        
        for i, (min_p, max_p) in enumerate(joint_limits):
            self.axes.append(VirtualAxis(i, min_p, max_p))
        
        # 连接状态
        self._connected = False
        
        # 仿真线程
        self._sim_thread: Optional[threading.Thread] = None
        self._sim_running = False
        self._sim_dt = 0.01  # 10ms仿真周期
        
        # 急停状态
        self._estop_active = False
        
        logger.info("虚拟驱动器初始化完成")
    
    def _simulation_loop(self):
        """仿真主循环"""
        while self._sim_running:
            if not self._estop_active:
                for axis in self.axes:
                    axis.update(self._sim_dt)
            time.sleep(self._sim_dt)
    
    # ==================== 连接管理 ====================
    
    def connect(self) -> bool:
        if self._connected:
            return True
        
        logger.info("虚拟驱动器连接中...")
        
        # 模拟连接延迟
        time.sleep(0.5)
        
        # 初始化轴状态
        for axis in self.axes:
            axis.state = AxisState.READY
        
        # 启动仿真线程
        self._sim_running = True
        self._sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._sim_thread.start()
        
        self._connected = True
        logger.info("虚拟驱动器已连接")
        return True
    
    def disconnect(self) -> None:
        if not self._connected:
            return
        
        logger.info("虚拟驱动器断开连接...")
        
        # 停止仿真
        self._sim_running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=1.0)
        
        # 禁止所有轴
        for axis in self.axes:
            axis.disable()
            axis.state = AxisState.NOT_READY
        
        self._connected = False
        logger.info("虚拟驱动器已断开")
    
    def is_connected(self) -> bool:
        return self._connected
    
    def scan_devices(self) -> List[Dict]:
        """扫描设备"""
        if not self._connected:
            return []
        
        devices = []
        for i, axis in enumerate(self.axes):
            devices.append({
                'slave_id': i,
                'name': f'ROKAE xServo (Virtual)',
                'product_code': 0x00016101,
                'state': axis.state.name
            })
        
        # 添加IO模块
        devices.append({
            'slave_id': 6,
            'name': 'ROKAE-SR_KEY (Virtual)',
            'product_code': 0x00000002,
            'state': 'READY'
        })
        
        return devices
    
    # ==================== 伺服控制 ====================
    
    def enable_servo(self, axis_id: Optional[int] = None) -> bool:
        if not self._connected or self._estop_active:
            return False
        
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                return self.axes[axis_id].enable()
            return False
        
        # 使能所有轴
        success = True
        for axis in self.axes:
            if not axis.enable():
                success = False
        
        logger.info(f"伺服使能: {'成功' if success else '失败'}")
        return success
    
    def disable_servo(self, axis_id: Optional[int] = None) -> bool:
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                return self.axes[axis_id].disable()
            return False
        
        for axis in self.axes:
            axis.disable()
        
        logger.info("伺服已禁止")
        return True
    
    def is_servo_enabled(self, axis_id: Optional[int] = None) -> bool:
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                return self.axes[axis_id].enabled
            return False
        
        return all(axis.enabled for axis in self.axes)
    
    # ==================== 位置控制 ====================
    
    def set_position(self, axis_id: int, position: float, velocity: float = None) -> bool:
        if not self._connected or self._estop_active:
            return False
        
        if not (0 <= axis_id < len(self.axes)):
            return False
        
        axis = self.axes[axis_id]
        if not axis.enabled:
            return False
        
        axis.set_target(position, velocity)
        return True
    
    def set_positions(self, positions: List[float], velocity: float = None) -> bool:
        if not self._connected or self._estop_active:
            return False
        
        if len(positions) != len(self.axes):
            return False
        
        for axis, pos in zip(self.axes, positions):
            if axis.enabled:
                axis.set_target(pos, velocity)
        
        return True
    
    def get_position(self, axis_id: int) -> float:
        if 0 <= axis_id < len(self.axes):
            return self.axes[axis_id].current_position
        return 0.0
    
    def get_positions(self) -> List[float]:
        return [axis.current_position for axis in self.axes]
    
    # ==================== 状态读取 ====================
    
    def get_axis_info(self, axis_id: int) -> AxisInfo:
        if 0 <= axis_id < len(self.axes):
            return self.axes[axis_id].get_info()
        return AxisInfo(axis_id, AxisState.NOT_READY, 0, 0, 0, 0, False, False)
    
    def get_all_axis_info(self) -> List[AxisInfo]:
        return [axis.get_info() for axis in self.axes]
    
    def is_in_position(self, axis_id: Optional[int] = None) -> bool:
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                return self.axes[axis_id].in_position
            return False
        
        return all(axis.in_position for axis in self.axes)
    
    def is_moving(self) -> bool:
        return any(axis.is_moving for axis in self.axes)
    
    # ==================== 错误处理 ====================
    
    def get_error_code(self, axis_id: int) -> int:
        if 0 <= axis_id < len(self.axes):
            return self.axes[axis_id].error_code
        return 0
    
    def has_error(self) -> bool:
        return any(axis.error_code != 0 for axis in self.axes) or self._estop_active
    
    def reset_error(self, axis_id: Optional[int] = None) -> bool:
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                self.axes[axis_id].reset_error()
                return True
            return False
        
        for axis in self.axes:
            axis.reset_error()
        
        self._estop_active = False
        logger.info("错误已复位")
        return True
    
    # ==================== 回原点 ====================
    
    def home(self, axis_id: Optional[int] = None) -> bool:
        if not self._connected or self._estop_active:
            return False
        
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                axis = self.axes[axis_id]
                if axis.enabled:
                    axis.home()
                    return True
            return False
        
        for axis in self.axes:
            if axis.enabled:
                axis.home()
        
        logger.info("开始回原点")
        return True
    
    def is_homed(self, axis_id: Optional[int] = None) -> bool:
        if axis_id is not None:
            if 0 <= axis_id < len(self.axes):
                return self.axes[axis_id].homed
            return False
        
        return all(axis.homed for axis in self.axes)
    
    # ==================== 急停 ====================
    
    def stop(self) -> bool:
        for axis in self.axes:
            axis.target_position = axis.current_position
            axis.is_moving = False
            axis.in_position = True
        
        logger.info("运动已停止")
        return True
    
    def emergency_stop(self) -> bool:
        self._estop_active = True
        
        # 立即停止所有运动
        for axis in self.axes:
            axis.target_position = axis.current_position
            axis.is_moving = False
            axis.velocity = 0
            axis.enabled = False
            axis.state = AxisState.FAULT
            axis.error_code = 0xFF00  # 急停代码
        
        logger.warning("急停已触发!")
        return True
    
    # ==================== 测试功能 ====================
    
    def inject_fault(self, axis_id: int, error_code: int):
        """注入故障 (测试用)"""
        if 0 <= axis_id < len(self.axes):
            self.axes[axis_id].inject_fault(error_code)
            logger.warning(f"轴{axis_id}故障注入: 0x{error_code:04X}")


if __name__ == "__main__":
    # 测试代码
    print("虚拟驱动器测试")
    print("=" * 60)
    
    driver = VirtualDriver()
    
    # 连接
    print("\n连接驱动器...")
    driver.connect()
    
    # 扫描设备
    print("\n扫描设备:")
    devices = driver.scan_devices()
    for dev in devices:
        print(f"  [{dev['slave_id']}] {dev['name']}")
    
    # 使能
    print("\n使能伺服...")
    driver.enable_servo()
    
    # 回原点
    print("回原点...")
    driver.home()
    
    # 等待回原点完成
    while driver.is_moving():
        time.sleep(0.1)
    print(f"回原点完成: {driver.is_homed()}")
    
    # 设置目标位置
    print("\n移动到目标位置...")
    driver.set_positions([30, -45, 60, 0, 45, 90])
    
    # 等待到位
    while driver.is_moving():
        positions = driver.get_positions()
        print(f"  位置: {[f'{p:.1f}' for p in positions]}", end='\r')
        time.sleep(0.1)
    
    print(f"\n最终位置: {driver.get_positions()}")
    
    # 断开
    print("\n断开连接...")
    driver.disconnect()
    print("测试完成")

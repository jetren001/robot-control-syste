"""
驱动接口抽象层
定义统一的驱动接口，支持虚拟和真实驱动

作者: Cursor AI
日期: 2026-01-28
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


class DriverMode(Enum):
    """驱动模式"""
    VIRTUAL = "virtual"
    REAL = "real"


class AxisState(Enum):
    """轴状态"""
    NOT_READY = 0
    READY = 1
    ENABLED = 2
    MOVING = 3
    FAULT = 4


@dataclass
class AxisInfo:
    """轴信息"""
    axis_id: int
    state: AxisState
    position: float
    velocity: float
    torque: float
    error_code: int
    enabled: bool
    in_position: bool


class DriverInterface(ABC):
    """
    驱动接口抽象基类
    定义所有驱动必须实现的接口
    """
    
    @abstractmethod
    def connect(self) -> bool:
        """
        连接驱动
        
        Returns:
            bool: 连接成功返回True
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """断开连接"""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """是否已连接"""
        pass
    
    @abstractmethod
    def scan_devices(self) -> List[Dict]:
        """
        扫描设备
        
        Returns:
            设备信息列表
        """
        pass
    
    # ==================== 伺服控制 ====================
    
    @abstractmethod
    def enable_servo(self, axis_id: Optional[int] = None) -> bool:
        """
        使能伺服
        
        Args:
            axis_id: 轴ID, None表示全部
            
        Returns:
            是否成功
        """
        pass
    
    @abstractmethod
    def disable_servo(self, axis_id: Optional[int] = None) -> bool:
        """
        禁止伺服
        
        Args:
            axis_id: 轴ID, None表示全部
            
        Returns:
            是否成功
        """
        pass
    
    @abstractmethod
    def is_servo_enabled(self, axis_id: Optional[int] = None) -> bool:
        """
        检查伺服是否使能
        
        Args:
            axis_id: 轴ID, None表示检查全部
            
        Returns:
            是否使能
        """
        pass
    
    # ==================== 位置控制 ====================
    
    @abstractmethod
    def set_position(self, axis_id: int, position: float, velocity: float = None) -> bool:
        """
        设置单轴目标位置
        
        Args:
            axis_id: 轴ID
            position: 目标位置 (度)
            velocity: 速度 (度/秒)
            
        Returns:
            是否成功
        """
        pass
    
    @abstractmethod
    def set_positions(self, positions: List[float], velocity: float = None) -> bool:
        """
        设置所有轴目标位置
        
        Args:
            positions: 6个目标位置 (度)
            velocity: 速度 (度/秒)
            
        Returns:
            是否成功
        """
        pass
    
    @abstractmethod
    def get_position(self, axis_id: int) -> float:
        """
        获取单轴位置
        
        Args:
            axis_id: 轴ID
            
        Returns:
            当前位置 (度)
        """
        pass
    
    @abstractmethod
    def get_positions(self) -> List[float]:
        """
        获取所有轴位置
        
        Returns:
            6个当前位置 (度)
        """
        pass
    
    # ==================== 状态读取 ====================
    
    @abstractmethod
    def get_axis_info(self, axis_id: int) -> AxisInfo:
        """
        获取轴信息
        
        Args:
            axis_id: 轴ID
            
        Returns:
            轴信息对象
        """
        pass
    
    @abstractmethod
    def get_all_axis_info(self) -> List[AxisInfo]:
        """
        获取所有轴信息
        
        Returns:
            轴信息列表
        """
        pass
    
    @abstractmethod
    def is_in_position(self, axis_id: Optional[int] = None) -> bool:
        """
        检查是否到位
        
        Args:
            axis_id: 轴ID, None表示检查全部
            
        Returns:
            是否到位
        """
        pass
    
    @abstractmethod
    def is_moving(self) -> bool:
        """检查是否在运动中"""
        pass
    
    # ==================== 错误处理 ====================
    
    @abstractmethod
    def get_error_code(self, axis_id: int) -> int:
        """
        获取轴错误代码
        
        Args:
            axis_id: 轴ID
            
        Returns:
            错误代码, 0表示无错误
        """
        pass
    
    @abstractmethod
    def has_error(self) -> bool:
        """检查是否有错误"""
        pass
    
    @abstractmethod
    def reset_error(self, axis_id: Optional[int] = None) -> bool:
        """
        复位错误
        
        Args:
            axis_id: 轴ID, None表示全部
            
        Returns:
            是否成功
        """
        pass
    
    # ==================== 回原点 ====================
    
    @abstractmethod
    def home(self, axis_id: Optional[int] = None) -> bool:
        """
        回原点
        
        Args:
            axis_id: 轴ID, None表示全部
            
        Returns:
            是否启动成功
        """
        pass
    
    @abstractmethod
    def is_homed(self, axis_id: Optional[int] = None) -> bool:
        """
        检查是否已回原点
        
        Args:
            axis_id: 轴ID, None表示检查全部
            
        Returns:
            是否已回原点
        """
        pass
    
    # ==================== 急停 ====================
    
    @abstractmethod
    def stop(self) -> bool:
        """
        停止运动
        
        Returns:
            是否成功
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> bool:
        """
        紧急停止
        
        Returns:
            是否成功
        """
        pass

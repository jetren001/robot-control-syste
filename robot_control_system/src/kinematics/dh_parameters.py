"""
D-H参数定义模块
定义珞石 SR5-C 机器人的 Denavit-Hartenberg 参数

作者: Cursor AI
日期: 2026-01-28
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class DHParameters:
    """D-H参数数据类
    
    标准D-H参数:
    - a: 连杆长度 (mm) - 沿 x_{i} 轴从 z_{i-1} 到 z_{i} 的距离
    - alpha: 连杆扭转角 (度) - 绕 x_{i} 轴从 z_{i-1} 到 z_{i} 的角度
    - d: 连杆偏移 (mm) - 沿 z_{i-1} 轴从 x_{i-1} 到 x_{i} 的距离
    - theta: 关节角 (度) - 绕 z_{i-1} 轴从 x_{i-1} 到 x_{i} 的角度
    """
    a: float      # 连杆长度 (mm)
    alpha: float  # 连杆扭转角 (度)
    d: float      # 连杆偏移 (mm)
    theta: float  # 关节角偏移 (度), 实际角度 = 输入角度 + theta
    
    def __repr__(self):
        return f"DH(a={self.a:.4f}, α={self.alpha:.4f}°, d={self.d:.4f}, θ_offset={self.theta:.4f}°)"


# 珞石 SR5-C 机器人 D-H 参数
# 从项目资料 (1dacccbfe861d822f31ee2a9a279a21a.png) 中提取
ROKAE_SR5_DH_PARAMS: List[DHParameters] = [
    # Link 1: 基座到第一关节
    DHParameters(
        a=0.4062,       # mm (非常小，近似为0)
        alpha=90.003,   # 度 (近似90度)
        d=324.452,      # mm
        theta=0.0       # 度
    ),
    # Link 2: 第一关节到第二关节
    DHParameters(
        a=403.4444,     # mm (主要连杆长度)
        alpha=-0.0369,  # 度 (近似0度)
        d=1.5371,       # mm (近似0)
        theta=90.0      # 度 (初始偏移90度)
    ),
    # Link 3: 第二关节到第三关节  
    DHParameters(
        a=53.192,       # mm
        alpha=90.0746,  # 度 (近似90度)
        d=0.0,          # mm
        theta=0.0       # 度
    ),
    # Link 4: 第三关节到第四关节
    DHParameters(
        a=-1.6823,      # mm (近似0)
        alpha=89.8548,  # 度 (近似90度)
        d=399.9484,     # mm
        theta=0.0       # 度
    ),
    # Link 5: 第四关节到第五关节
    DHParameters(
        a=-0.8193,      # mm (近似0)
        alpha=-90.0597, # 度 (近似-90度)
        d=136.7432,     # mm
        theta=90.0      # 度 (初始偏移90度)
    ),
    # Link 6: 第五关节到末端执行器
    DHParameters(
        a=0.0,          # mm
        alpha=-0.0106,  # 度 (近似0度)
        d=103.037,      # mm
        theta=0.0       # 度
    ),
]

# 简化的D-H参数 (用于快速计算，将接近整数的值四舍五入)
ROKAE_SR5_DH_PARAMS_SIMPLIFIED: List[DHParameters] = [
    DHParameters(a=0.0, alpha=90.0, d=324.452, theta=0.0),
    DHParameters(a=403.444, alpha=0.0, d=0.0, theta=90.0),
    DHParameters(a=53.192, alpha=90.0, d=0.0, theta=0.0),
    DHParameters(a=0.0, alpha=90.0, d=399.948, theta=0.0),
    DHParameters(a=0.0, alpha=-90.0, d=136.743, theta=90.0),
    DHParameters(a=0.0, alpha=0.0, d=103.037, theta=0.0),
]

# 关节限位 (度)
JOINT_LIMITS: List[Tuple[float, float]] = [
    (-170.0, 170.0),   # J1
    (-120.0, 120.0),   # J2
    (-170.0, 170.0),   # J3
    (-120.0, 120.0),   # J4
    (-170.0, 170.0),   # J5
    (-360.0, 360.0),   # J6
]

# 关节最大速度 (度/秒)
JOINT_MAX_VELOCITIES: List[float] = [
    180.0,  # J1
    180.0,  # J2
    180.0,  # J3
    225.0,  # J4
    225.0,  # J5
    225.0,  # J6
]

# 关节最大加速度 (度/秒^2)
JOINT_MAX_ACCELERATIONS: List[float] = [
    360.0,  # J1
    360.0,  # J2
    360.0,  # J3
    450.0,  # J4
    450.0,  # J5
    450.0,  # J6
]

# 机器人基本参数
ROBOT_CONFIG = {
    "name": "ROKAE xMate SR5-C",
    "dof": 6,
    "payload": 5.0,  # kg
    "reach": 924.0,  # mm (近似工作半径)
    "repeatability": 0.02,  # mm
    "weight": 25.0,  # kg
}


def get_dh_matrix(dh_params: DHParameters, theta: float) -> np.ndarray:
    """
    计算单个连杆的D-H变换矩阵
    
    Args:
        dh_params: D-H参数
        theta: 关节角度 (度)
        
    Returns:
        4x4 齐次变换矩阵
    """
    # 转换为弧度
    theta_rad = np.radians(theta + dh_params.theta)
    alpha_rad = np.radians(dh_params.alpha)
    
    a = dh_params.a
    d = dh_params.d
    
    # 标准D-H变换矩阵
    ct = np.cos(theta_rad)
    st = np.sin(theta_rad)
    ca = np.cos(alpha_rad)
    sa = np.sin(alpha_rad)
    
    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ]
    ])
    
    return T


def validate_joint_angles(angles: List[float]) -> Tuple[bool, str]:
    """
    验证关节角度是否在限位范围内
    
    Args:
        angles: 6个关节角度 (度)
        
    Returns:
        (是否有效, 错误信息)
    """
    if len(angles) != 6:
        return False, f"需要6个关节角度，收到{len(angles)}个"
    
    for i, (angle, (min_val, max_val)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_val or angle > max_val:
            return False, f"J{i+1}角度{angle:.2f}°超出限位范围[{min_val}, {max_val}]°"
    
    return True, ""


if __name__ == "__main__":
    # 测试代码
    print("珞石 SR5-C D-H参数:")
    print("-" * 60)
    for i, dh in enumerate(ROKAE_SR5_DH_PARAMS):
        print(f"Link {i+1}: {dh}")
    
    print("\n关节限位:")
    for i, (min_val, max_val) in enumerate(JOINT_LIMITS):
        print(f"J{i+1}: [{min_val}°, {max_val}°]")
    
    print("\n机器人配置:")
    for key, value in ROBOT_CONFIG.items():
        print(f"  {key}: {value}")

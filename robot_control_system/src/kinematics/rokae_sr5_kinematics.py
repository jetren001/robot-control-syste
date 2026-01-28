"""
珞石 SR5-C 机器人运动学模块
实现正运动学、逆运动学和雅可比矩阵计算

作者: Cursor AI
日期: 2026-01-28
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import warnings

from .dh_parameters import (
    DHParameters, 
    ROKAE_SR5_DH_PARAMS,
    ROKAE_SR5_DH_PARAMS_SIMPLIFIED,
    JOINT_LIMITS,
    get_dh_matrix,
    validate_joint_angles
)


@dataclass
class Pose:
    """末端位姿数据类"""
    position: np.ndarray      # [x, y, z] (mm)
    rotation_matrix: np.ndarray  # 3x3 旋转矩阵
    euler_angles: np.ndarray  # [rx, ry, rz] (度) - ZYX欧拉角
    
    def __repr__(self):
        return (f"Pose(pos=[{self.position[0]:.2f}, {self.position[1]:.2f}, "
                f"{self.position[2]:.2f}], euler=[{self.euler_angles[0]:.2f}°, "
                f"{self.euler_angles[1]:.2f}°, {self.euler_angles[2]:.2f}°])")


class RokaeSR5Kinematics:
    """
    珞石 SR5-C 六轴机器人运动学类
    
    实现功能:
    - 正运动学 (Forward Kinematics)
    - 逆运动学 (Inverse Kinematics)
    - 雅可比矩阵计算 (Jacobian)
    - 工作空间检查
    """
    
    def __init__(self, use_simplified: bool = False):
        """
        初始化运动学计算器
        
        Args:
            use_simplified: 是否使用简化的D-H参数
        """
        self.dh_params = ROKAE_SR5_DH_PARAMS_SIMPLIFIED if use_simplified else ROKAE_SR5_DH_PARAMS
        self.joint_limits = JOINT_LIMITS
        self.num_joints = 6
        
        # 缓存变换矩阵
        self._cached_transforms: Optional[List[np.ndarray]] = None
        self._cached_angles: Optional[List[float]] = None
    
    def forward_kinematics(self, joint_angles: List[float], 
                          return_all_transforms: bool = False) -> Pose:
        """
        正运动学: 根据关节角度计算末端位姿
        
        Args:
            joint_angles: 6个关节角度 (度)
            return_all_transforms: 是否返回所有中间变换矩阵
            
        Returns:
            Pose: 末端位姿
            
        Raises:
            ValueError: 关节角度无效
        """
        # 验证输入
        valid, msg = validate_joint_angles(joint_angles)
        if not valid:
            raise ValueError(msg)
        
        # 计算各连杆变换矩阵
        transforms = []
        T_cumulative = np.eye(4)
        
        for i, (dh, angle) in enumerate(zip(self.dh_params, joint_angles)):
            T_i = get_dh_matrix(dh, angle)
            T_cumulative = T_cumulative @ T_i
            transforms.append(T_cumulative.copy())
        
        # 缓存结果
        self._cached_transforms = transforms
        self._cached_angles = list(joint_angles)
        
        # 提取位置和姿态
        T_end = transforms[-1]
        position = T_end[:3, 3]
        rotation_matrix = T_end[:3, :3]
        euler_angles = self._rotation_matrix_to_euler(rotation_matrix)
        
        pose = Pose(
            position=position,
            rotation_matrix=rotation_matrix,
            euler_angles=euler_angles
        )
        
        if return_all_transforms:
            return pose, transforms
        return pose
    
    def inverse_kinematics(self, target_position: List[float], 
                           target_orientation: List[float],
                           initial_guess: Optional[List[float]] = None,
                           max_iterations: int = 100,
                           tolerance: float = 1e-3) -> Tuple[List[float], bool]:
        """
        逆运动学: 根据目标位姿计算关节角度
        使用数值迭代方法 (阻尼最小二乘法/Levenberg-Marquardt)
        
        Args:
            target_position: 目标位置 [x, y, z] (mm)
            target_orientation: 目标姿态 [rx, ry, rz] (度) - ZYX欧拉角
            initial_guess: 初始猜测关节角度, None则使用当前缓存或零位
            max_iterations: 最大迭代次数
            tolerance: 收敛容差 (mm/度)
            
        Returns:
            (关节角度列表, 是否收敛)
        """
        # 初始化
        if initial_guess is not None:
            q = np.array(initial_guess, dtype=float)
        elif self._cached_angles is not None:
            q = np.array(self._cached_angles, dtype=float)
        else:
            q = np.zeros(6)
        
        target_pos = np.array(target_position)
        target_euler = np.array(target_orientation)
        
        # 阻尼因子 - 使用自适应阻尼
        damping = 0.01
        
        for iteration in range(max_iterations):
            # 计算当前位姿
            current_pose = self.forward_kinematics(q.tolist())
            
            # 计算位置误差 (mm)
            pos_error = target_pos - current_pose.position
            
            # 计算姿态误差 (度)
            euler_error = target_euler - current_pose.euler_angles
            
            # 归一化角度误差到[-180, 180]
            euler_error = np.array([self._normalize_angle(e) for e in euler_error])
            
            # 组合误差向量 - 姿态误差需要缩放使其与位置误差量级相近
            # 1度姿态误差约等于10mm末端位移
            error = np.concatenate([pos_error, euler_error * 10.0])
            
            # 检查收敛
            pos_norm = np.linalg.norm(pos_error)
            rot_norm = np.linalg.norm(euler_error)
            
            if pos_norm < tolerance and rot_norm < tolerance * 2:  # 姿态容差(度)
                # 应用关节限位
                q = self._apply_joint_limits(q)
                return q.tolist(), True
            
            # 计算雅可比矩阵
            J = self.jacobian(q.tolist())
            
            # 姿态雅可比也需要缩放
            J_scaled = J.copy()
            J_scaled[3:, :] = J_scaled[3:, :] * 10.0
            
            # 阻尼最小二乘法更新
            # Δq = (J^T * J + λ²I)^(-1) * J^T * error
            JtJ = J_scaled.T @ J_scaled
            damping_matrix = damping * np.eye(6)
            try:
                delta_q = np.linalg.solve(JtJ + damping_matrix, J_scaled.T @ error)
            except np.linalg.LinAlgError:
                # 奇异矩阵,使用伪逆
                delta_q = np.linalg.pinv(J_scaled) @ error
            
            # 限制步长
            max_step = 10.0  # 度
            step_norm = np.linalg.norm(delta_q)
            if step_norm > max_step:
                delta_q = delta_q * (max_step / step_norm)
            
            # 更新关节角度
            q = q + delta_q
            
            # 应用关节限位
            q = self._apply_joint_limits(q)
        
        # 未收敛
        warnings.warn(f"逆运动学未在{max_iterations}次迭代内收敛")
        return q.tolist(), False
    
    def jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """
        计算雅可比矩阵
        
        雅可比矩阵将关节速度映射到末端速度:
        [v]   [J_v]     [q̇]
        [ω] = [J_ω]  *  
        
        Args:
            joint_angles: 6个关节角度 (度)
            
        Returns:
            6x6 雅可比矩阵
        """
        # 获取所有变换矩阵
        if (self._cached_angles is not None and 
            np.allclose(joint_angles, self._cached_angles)):
            transforms = self._cached_transforms
        else:
            _, transforms = self.forward_kinematics(joint_angles, return_all_transforms=True)
        
        # 末端位置
        p_e = transforms[-1][:3, 3]
        
        # 计算雅可比列
        J = np.zeros((6, 6))
        
        # 基座坐标系
        T_prev = np.eye(4)
        
        for i in range(6):
            # 关节轴方向 (z轴)
            z_i = T_prev[:3, 2] if i == 0 else transforms[i-1][:3, 2]
            
            # 关节位置
            o_i = T_prev[:3, 3] if i == 0 else transforms[i-1][:3, 3]
            
            # 旋转关节
            # 线速度雅可比: z_i × (p_e - o_i)
            # 单位: mm / rad, 需要转换为 mm / deg
            J[:3, i] = np.cross(z_i, p_e - o_i) * np.pi / 180.0
            
            # 角速度雅可比: z_i
            # 单位: rad / rad = 1, 输出为 deg / deg = 1
            J[3:, i] = z_i
        
        return J
    
    def get_joint_positions(self, joint_angles: List[float]) -> List[np.ndarray]:
        """
        获取所有关节的3D位置 (用于可视化)
        
        Args:
            joint_angles: 6个关节角度 (度)
            
        Returns:
            7个点的位置列表 (基座 + 6个关节)
        """
        positions = [np.array([0.0, 0.0, 0.0])]  # 基座位置
        
        _, transforms = self.forward_kinematics(joint_angles, return_all_transforms=True)
        
        for T in transforms:
            positions.append(T[:3, 3].copy())
        
        return positions
    
    def get_joint_transforms(self, joint_angles: List[float]) -> List[np.ndarray]:
        """
        获取所有关节的变换矩阵 (用于精确可视化)
        
        Args:
            joint_angles: 6个关节角度 (度)
            
        Returns:
            6个关节的4x4变换矩阵列表
        """
        _, transforms = self.forward_kinematics(joint_angles, return_all_transforms=True)
        return transforms
    
    def check_workspace(self, position: List[float]) -> bool:
        """
        检查位置是否在工作空间内 (简化检查)
        
        Args:
            position: [x, y, z] (mm)
            
        Returns:
            bool: 是否在工作空间内
        """
        x, y, z = position
        
        # 计算到基座的水平距离
        r_horizontal = np.sqrt(x**2 + y**2)
        
        # 简化的工作空间边界
        # 最大伸展约 924mm, 最小约 200mm
        max_reach = 900.0
        min_reach = 200.0
        max_height = 1000.0
        min_height = -200.0
        
        if r_horizontal > max_reach or r_horizontal < min_reach:
            return False
        if z > max_height or z < min_height:
            return False
        
        return True
    
    def _rotation_matrix_to_euler(self, R: np.ndarray) -> np.ndarray:
        """
        旋转矩阵转ZYX欧拉角
        
        Args:
            R: 3x3旋转矩阵
            
        Returns:
            [rx, ry, rz] 欧拉角 (度)
        """
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        
        singular = sy < 1e-6
        
        if not singular:
            rx = np.arctan2(R[2, 1], R[2, 2])
            ry = np.arctan2(-R[2, 0], sy)
            rz = np.arctan2(R[1, 0], R[0, 0])
        else:
            rx = np.arctan2(-R[1, 2], R[1, 1])
            ry = np.arctan2(-R[2, 0], sy)
            rz = 0
        
        return np.degrees(np.array([rx, ry, rz]))
    
    def _euler_to_rotation_matrix(self, euler: List[float]) -> np.ndarray:
        """
        ZYX欧拉角转旋转矩阵
        
        Args:
            euler: [rx, ry, rz] (度)
            
        Returns:
            3x3旋转矩阵
        """
        rx, ry, rz = np.radians(euler)
        
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        return Rz @ Ry @ Rx
    
    def _normalize_angle(self, angle: float) -> float:
        """将角度归一化到[-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def _apply_joint_limits(self, q: np.ndarray) -> np.ndarray:
        """应用关节限位"""
        q_limited = q.copy()
        for i, (min_val, max_val) in enumerate(self.joint_limits):
            q_limited[i] = np.clip(q_limited[i], min_val, max_val)
        return q_limited


# 便捷函数
def create_kinematics(simplified: bool = False) -> RokaeSR5Kinematics:
    """创建运动学计算器实例"""
    return RokaeSR5Kinematics(use_simplified=simplified)


if __name__ == "__main__":
    # 测试代码
    print("珞石 SR5-C 运动学测试")
    print("=" * 60)
    
    kin = RokaeSR5Kinematics(use_simplified=True)
    
    # 测试正运动学
    test_angles = [0, 0, 0, 0, 0, 0]
    print(f"\n测试关节角度: {test_angles}")
    
    pose = kin.forward_kinematics(test_angles)
    print(f"正运动学结果: {pose}")
    
    # 测试不同姿态
    test_angles_2 = [30, -45, 60, 0, 45, 0]
    print(f"\n测试关节角度: {test_angles_2}")
    
    pose_2 = kin.forward_kinematics(test_angles_2)
    print(f"正运动学结果: {pose_2}")
    
    # 测试逆运动学
    print("\n测试逆运动学...")
    target_pos = pose_2.position.tolist()
    target_euler = pose_2.euler_angles.tolist()
    
    result_angles, converged = kin.inverse_kinematics(
        target_pos, target_euler, initial_guess=[0, 0, 0, 0, 0, 0]
    )
    
    print(f"目标位置: {target_pos}")
    print(f"目标姿态: {target_euler}")
    print(f"求解结果: {result_angles}")
    print(f"是否收敛: {converged}")
    
    # 验证
    if converged:
        verify_pose = kin.forward_kinematics(result_angles)
        print(f"验证结果: {verify_pose}")
        pos_error = np.linalg.norm(np.array(target_pos) - verify_pose.position)
        print(f"位置误差: {pos_error:.4f} mm")
    
    # 测试雅可比矩阵
    print("\n雅可比矩阵 (零位):")
    J = kin.jacobian([0, 0, 0, 0, 0, 0])
    print(J)
    
    # 获取关节位置
    print("\n关节位置:")
    positions = kin.get_joint_positions([0, -45, 90, 0, 45, 0])
    for i, pos in enumerate(positions):
        print(f"  关节{i}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

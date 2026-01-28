"""
轨迹生成模块
实现轨迹插补和速度规划

作者: Cursor AI
日期: 2026-01-28
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class MotionType(Enum):
    """运动类型"""
    JOINT = "joint"  # 关节空间运动
    LINEAR = "linear"  # 直线运动
    CIRCULAR = "circular"  # 圆弧运动


@dataclass
class TrajectoryPoint:
    """轨迹点数据类"""
    position: np.ndarray      # 位置 [x, y, z] 或关节角度
    velocity: np.ndarray      # 速度
    acceleration: np.ndarray  # 加速度
    time: float              # 时间戳
    
    def __repr__(self):
        return f"TrajPoint(t={self.time:.3f}, pos={self.position})"


class TrajectoryGenerator:
    """
    轨迹生成器
    支持关节空间和笛卡尔空间的轨迹插补
    """
    
    def __init__(self, dt: float = 0.001):
        """
        初始化轨迹生成器
        
        Args:
            dt: 插补周期 (秒)
        """
        self.dt = dt
    
    def linear_interpolation(self, start: np.ndarray, end: np.ndarray, 
                            velocity: float, 
                            acceleration: float) -> List[TrajectoryPoint]:
        """
        线性插补 (带梯形速度规划)
        
        Args:
            start: 起点
            end: 终点
            velocity: 最大速度
            acceleration: 加速度
            
        Returns:
            轨迹点列表
        """
        # 计算总距离
        distance = np.linalg.norm(end - start)
        if distance < 1e-6:
            return [TrajectoryPoint(start.copy(), np.zeros_like(start), 
                                   np.zeros_like(start), 0.0)]
        
        # 方向向量
        direction = (end - start) / distance
        
        # 梯形速度规划
        # 计算加速距离和减速距离
        t_acc = velocity / acceleration
        d_acc = 0.5 * acceleration * t_acc ** 2
        
        if 2 * d_acc >= distance:
            # 无匀速段 - 三角形速度曲线
            t_acc = np.sqrt(distance / acceleration)
            d_acc = distance / 2
            v_peak = acceleration * t_acc
            total_time = 2 * t_acc
        else:
            # 有匀速段 - 梯形速度曲线
            v_peak = velocity
            d_const = distance - 2 * d_acc
            t_const = d_const / velocity
            total_time = 2 * t_acc + t_const
        
        # 生成轨迹点
        trajectory = []
        t = 0.0
        
        while t <= total_time:
            if t < t_acc:
                # 加速段
                s = 0.5 * acceleration * t ** 2
                v = acceleration * t
                a = acceleration
            elif 2 * d_acc < distance and t < total_time - t_acc:
                # 匀速段
                s = d_acc + v_peak * (t - t_acc)
                v = v_peak
                a = 0.0
            else:
                # 减速段
                t_dec = total_time - t
                s = distance - 0.5 * acceleration * t_dec ** 2
                v = acceleration * t_dec
                a = -acceleration
            
            pos = start + direction * s
            vel = direction * v
            acc = direction * a
            
            trajectory.append(TrajectoryPoint(pos, vel, acc, t))
            t += self.dt
        
        # 确保终点准确
        trajectory.append(TrajectoryPoint(end.copy(), np.zeros_like(end), 
                                         np.zeros_like(end), total_time))
        
        return trajectory
    
    def joint_interpolation(self, start_joints: np.ndarray, end_joints: np.ndarray,
                           max_velocity: float, max_acceleration: float,
                           joint_velocities: Optional[List[float]] = None,
                           joint_accelerations: Optional[List[float]] = None) -> List[TrajectoryPoint]:
        """
        关节空间插补
        
        Args:
            start_joints: 起始关节角度
            end_joints: 目标关节角度
            max_velocity: 最大速度 (度/秒) - 如果未指定各轴速度
            max_acceleration: 最大加速度 (度/秒^2)
            joint_velocities: 各轴最大速度
            joint_accelerations: 各轴最大加速度
            
        Returns:
            轨迹点列表
        """
        # 计算各轴运动距离
        distances = np.abs(end_joints - start_joints)
        
        # 计算各轴所需时间
        if joint_velocities is None:
            joint_velocities = [max_velocity] * len(start_joints)
        if joint_accelerations is None:
            joint_accelerations = [max_acceleration] * len(start_joints)
        
        # 找到最慢的轴 (决定总时间)
        max_time = 0.0
        for i, (d, v, a) in enumerate(zip(distances, joint_velocities, joint_accelerations)):
            if d < 1e-6:
                continue
            t_acc = v / a
            d_acc = 0.5 * a * t_acc ** 2
            if 2 * d_acc >= d:
                t = 2 * np.sqrt(d / a)
            else:
                t = 2 * t_acc + (d - 2 * d_acc) / v
            max_time = max(max_time, t)
        
        if max_time < 1e-6:
            return [TrajectoryPoint(start_joints.copy(), np.zeros_like(start_joints),
                                   np.zeros_like(start_joints), 0.0)]
        
        # 使用同步时间为所有轴生成轨迹
        # 根据总时间反算各轴的实际速度和加速度
        trajectory = []
        t = 0.0
        
        while t <= max_time:
            # 使用S曲线或梯形曲线计算归一化位置
            s = self._normalized_position(t, max_time)
            s_dot = self._normalized_velocity(t, max_time)
            s_ddot = self._normalized_acceleration(t, max_time)
            
            pos = start_joints + (end_joints - start_joints) * s
            vel = (end_joints - start_joints) * s_dot
            acc = (end_joints - start_joints) * s_ddot
            
            trajectory.append(TrajectoryPoint(pos, vel, acc, t))
            t += self.dt
        
        # 确保终点
        trajectory.append(TrajectoryPoint(end_joints.copy(), np.zeros_like(end_joints),
                                         np.zeros_like(end_joints), max_time))
        
        return trajectory
    
    def _normalized_position(self, t: float, total_time: float) -> float:
        """计算归一化位置 (0-1)"""
        if total_time < 1e-6:
            return 1.0
        
        tau = t / total_time
        
        # 使用五次多项式 (平滑起止)
        # s = 10*tau^3 - 15*tau^4 + 6*tau^5
        if tau <= 0:
            return 0.0
        elif tau >= 1:
            return 1.0
        else:
            return 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    
    def _normalized_velocity(self, t: float, total_time: float) -> float:
        """计算归一化速度"""
        if total_time < 1e-6:
            return 0.0
        
        tau = t / total_time
        
        if tau <= 0 or tau >= 1:
            return 0.0
        else:
            return (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / total_time
    
    def _normalized_acceleration(self, t: float, total_time: float) -> float:
        """计算归一化加速度"""
        if total_time < 1e-6:
            return 0.0
        
        tau = t / total_time
        
        if tau <= 0 or tau >= 1:
            return 0.0
        else:
            return (60 * tau - 180 * tau**2 + 120 * tau**3) / (total_time ** 2)
    
    def blend_trajectories(self, traj1: List[TrajectoryPoint], 
                          traj2: List[TrajectoryPoint],
                          blend_time: float) -> List[TrajectoryPoint]:
        """
        混合两段轨迹 (平滑过渡)
        
        Args:
            traj1: 第一段轨迹
            traj2: 第二段轨迹
            blend_time: 混合时间
            
        Returns:
            混合后的轨迹
        """
        if not traj1 or not traj2:
            return traj1 + traj2
        
        result = []
        
        # 第一段轨迹 (到混合区域前)
        blend_start_time = traj1[-1].time - blend_time / 2
        for pt in traj1:
            if pt.time < blend_start_time:
                result.append(pt)
        
        # 混合区域
        blend_points = []
        t1_blend = [pt for pt in traj1 if pt.time >= blend_start_time]
        t2_blend = [pt for pt in traj2 if pt.time <= blend_time / 2]
        
        # 简单线性混合
        for i, pt1 in enumerate(t1_blend):
            if i < len(t2_blend):
                pt2 = t2_blend[i]
                alpha = (pt1.time - blend_start_time) / blend_time
                pos = (1 - alpha) * pt1.position + alpha * pt2.position
                vel = (1 - alpha) * pt1.velocity + alpha * pt2.velocity
                acc = (1 - alpha) * pt1.acceleration + alpha * pt2.acceleration
                blend_points.append(TrajectoryPoint(pos, vel, acc, pt1.time))
        
        result.extend(blend_points)
        
        # 第二段轨迹 (混合区域后)
        time_offset = traj1[-1].time
        for pt in traj2:
            if pt.time > blend_time / 2:
                new_pt = TrajectoryPoint(
                    pt.position.copy(),
                    pt.velocity.copy(),
                    pt.acceleration.copy(),
                    pt.time + time_offset - blend_time / 2
                )
                result.append(new_pt)
        
        return result


if __name__ == "__main__":
    # 测试代码
    print("轨迹生成器测试")
    print("=" * 60)
    
    gen = TrajectoryGenerator(dt=0.01)
    
    # 测试线性插补
    print("\n线性插补测试:")
    start = np.array([0.0, 0.0, 0.0])
    end = np.array([100.0, 50.0, 30.0])
    traj = gen.linear_interpolation(start, end, velocity=50.0, acceleration=100.0)
    
    print(f"  轨迹点数: {len(traj)}")
    print(f"  总时间: {traj[-1].time:.3f} s")
    print(f"  起点: {traj[0].position}")
    print(f"  终点: {traj[-1].position}")
    
    # 测试关节插补
    print("\n关节插补测试:")
    start_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    end_joints = np.array([30.0, -45.0, 60.0, 0.0, 45.0, 90.0])
    
    traj_joint = gen.joint_interpolation(start_joints, end_joints, 
                                         max_velocity=100.0, max_acceleration=200.0)
    
    print(f"  轨迹点数: {len(traj_joint)}")
    print(f"  总时间: {traj_joint[-1].time:.3f} s")
    print(f"  起点: {traj_joint[0].position}")
    print(f"  终点: {traj_joint[-1].position}")

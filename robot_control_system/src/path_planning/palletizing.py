"""
码垛路径规划模块
实现码垛点位计算和路径生成

作者: Cursor AI
日期: 2026-01-28
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum

from .trajectory import TrajectoryGenerator, TrajectoryPoint


class PalletPattern(Enum):
    """码垛模式"""
    ROW_FIRST = "row_first"      # 行优先
    COLUMN_FIRST = "column_first"  # 列优先
    ZIGZAG = "zigzag"            # 之字形


@dataclass
class PalletConfig:
    """码垛配置"""
    # 托盘参数
    pallet_origin: np.ndarray = field(default_factory=lambda: np.array([400.0, -300.0, 50.0]))  # 托盘原点 (mm)
    
    # 码垛尺寸
    rows: int = 3           # 行数
    cols: int = 3           # 列数
    layers: int = 2         # 层数
    
    # 工件尺寸
    box_length: float = 100.0  # 长度 (mm)
    box_width: float = 80.0    # 宽度 (mm)
    box_height: float = 50.0   # 高度 (mm)
    
    # 间距
    row_gap: float = 10.0      # 行间距 (mm)
    col_gap: float = 10.0      # 列间距 (mm)
    
    # 抓取高度
    pick_height: float = 100.0      # 抓取点高度 (mm)
    approach_height: float = 150.0  # 接近高度 (mm)
    
    # 抓取位置
    pick_position: np.ndarray = field(default_factory=lambda: np.array([500.0, 200.0, 100.0]))
    
    # 姿态 (度)
    pick_orientation: np.ndarray = field(default_factory=lambda: np.array([180.0, 0.0, 0.0]))
    place_orientation: np.ndarray = field(default_factory=lambda: np.array([180.0, 0.0, 0.0]))
    
    # 速度参数
    approach_velocity: float = 50.0    # 接近速度 (%)
    pick_velocity: float = 20.0        # 抓取速度 (%)
    transfer_velocity: float = 80.0    # 移载速度 (%)
    place_velocity: float = 20.0       # 放置速度 (%)
    
    # 码垛模式
    pattern: PalletPattern = PalletPattern.ROW_FIRST


@dataclass
class PalletPoint:
    """码垛点位"""
    position: np.ndarray      # 位置 [x, y, z]
    orientation: np.ndarray   # 姿态 [rx, ry, rz]
    row: int                  # 行号
    col: int                  # 列号
    layer: int                # 层号
    index: int                # 总序号


class PalletizingPlanner:
    """
    码垛路径规划器
    
    功能:
    - 计算码垛点位
    - 生成抓取/放置轨迹
    - 支持多种码垛模式
    """
    
    def __init__(self, config: Optional[PalletConfig] = None):
        """
        初始化码垛规划器
        
        Args:
            config: 码垛配置, None则使用默认配置
        """
        self.config = config or PalletConfig()
        self.trajectory_gen = TrajectoryGenerator(dt=0.01)
        
        # 计算所有码垛点位
        self._pallet_points: List[PalletPoint] = []
        self._calculate_pallet_points()
    
    def _calculate_pallet_points(self):
        """计算所有码垛点位"""
        self._pallet_points.clear()
        
        cfg = self.config
        index = 0
        
        # 计算单个工件的占用尺寸 (包含间距)
        step_x = cfg.box_length + cfg.row_gap
        step_y = cfg.box_width + cfg.col_gap
        step_z = cfg.box_height
        
        for layer in range(cfg.layers):
            # 根据码垛模式生成点位顺序
            if cfg.pattern == PalletPattern.ROW_FIRST:
                positions = self._row_first_order(cfg.rows, cfg.cols)
            elif cfg.pattern == PalletPattern.COLUMN_FIRST:
                positions = self._column_first_order(cfg.rows, cfg.cols)
            else:  # ZIGZAG
                positions = self._zigzag_order(cfg.rows, cfg.cols)
            
            for row, col in positions:
                # 计算位置
                x = cfg.pallet_origin[0] + row * step_x
                y = cfg.pallet_origin[1] + col * step_y
                z = cfg.pallet_origin[2] + layer * step_z + cfg.box_height / 2
                
                pos = np.array([x, y, z])
                
                # 姿态 (可以根据层数调整)
                ori = cfg.place_orientation.copy()
                
                self._pallet_points.append(PalletPoint(
                    position=pos,
                    orientation=ori,
                    row=row,
                    col=col,
                    layer=layer,
                    index=index
                ))
                index += 1
    
    def _row_first_order(self, rows: int, cols: int) -> List[Tuple[int, int]]:
        """行优先顺序"""
        return [(r, c) for r in range(rows) for c in range(cols)]
    
    def _column_first_order(self, rows: int, cols: int) -> List[Tuple[int, int]]:
        """列优先顺序"""
        return [(r, c) for c in range(cols) for r in range(rows)]
    
    def _zigzag_order(self, rows: int, cols: int) -> List[Tuple[int, int]]:
        """之字形顺序"""
        result = []
        for r in range(rows):
            if r % 2 == 0:
                result.extend([(r, c) for c in range(cols)])
            else:
                result.extend([(r, c) for c in range(cols - 1, -1, -1)])
        return result
    
    def get_pallet_points(self) -> List[PalletPoint]:
        """获取所有码垛点位"""
        return self._pallet_points.copy()
    
    def get_pallet_point(self, index: int) -> Optional[PalletPoint]:
        """获取指定索引的码垛点位"""
        if 0 <= index < len(self._pallet_points):
            return self._pallet_points[index]
        return None
    
    def get_total_count(self) -> int:
        """获取总码垛数量"""
        return len(self._pallet_points)
    
    def generate_pick_trajectory(self, current_position: np.ndarray) -> List[dict]:
        """
        生成抓取轨迹
        
        Args:
            current_position: 当前位置 [x, y, z]
            
        Returns:
            轨迹点列表 (字典格式)
        """
        cfg = self.config
        trajectory = []
        
        # 抓取点
        pick_pos = cfg.pick_position.copy()
        
        # 接近点 (抓取点上方)
        approach_pos = pick_pos.copy()
        approach_pos[2] = pick_pos[2] + cfg.approach_height
        
        # 1. 移动到接近点
        trajectory.append({
            'type': 'linear',
            'target': approach_pos.tolist(),
            'orientation': cfg.pick_orientation.tolist(),
            'velocity': cfg.transfer_velocity,
            'description': '移动到抓取接近点'
        })
        
        # 2. 下降到抓取点
        trajectory.append({
            'type': 'linear',
            'target': pick_pos.tolist(),
            'orientation': cfg.pick_orientation.tolist(),
            'velocity': cfg.approach_velocity,
            'description': '下降到抓取点'
        })
        
        # 3. 抓取动作
        trajectory.append({
            'type': 'gripper',
            'action': 'close',
            'description': '夹爪闭合'
        })
        
        # 4. 提升
        trajectory.append({
            'type': 'linear',
            'target': approach_pos.tolist(),
            'orientation': cfg.pick_orientation.tolist(),
            'velocity': cfg.pick_velocity,
            'description': '提升工件'
        })
        
        return trajectory
    
    def generate_place_trajectory(self, pallet_index: int) -> List[dict]:
        """
        生成放置轨迹
        
        Args:
            pallet_index: 码垛点索引
            
        Returns:
            轨迹点列表 (字典格式)
        """
        if pallet_index >= len(self._pallet_points):
            return []
        
        cfg = self.config
        point = self._pallet_points[pallet_index]
        trajectory = []
        
        # 放置点
        place_pos = point.position.copy()
        
        # 接近点
        approach_pos = place_pos.copy()
        approach_pos[2] = place_pos[2] + cfg.approach_height
        
        # 1. 移动到接近点
        trajectory.append({
            'type': 'linear',
            'target': approach_pos.tolist(),
            'orientation': point.orientation.tolist(),
            'velocity': cfg.transfer_velocity,
            'description': f'移动到放置接近点 (第{point.layer+1}层, 第{point.row+1}行, 第{point.col+1}列)'
        })
        
        # 2. 下降到放置点
        trajectory.append({
            'type': 'linear',
            'target': place_pos.tolist(),
            'orientation': point.orientation.tolist(),
            'velocity': cfg.approach_velocity,
            'description': '下降到放置点'
        })
        
        # 3. 放置动作
        trajectory.append({
            'type': 'gripper',
            'action': 'open',
            'description': '夹爪打开'
        })
        
        # 4. 提升
        trajectory.append({
            'type': 'linear',
            'target': approach_pos.tolist(),
            'orientation': point.orientation.tolist(),
            'velocity': cfg.place_velocity,
            'description': '提升退出'
        })
        
        return trajectory
    
    def generate_full_cycle(self, pallet_index: int, 
                           current_position: np.ndarray) -> List[dict]:
        """
        生成完整的抓取-放置循环
        
        Args:
            pallet_index: 码垛点索引
            current_position: 当前位置
            
        Returns:
            完整轨迹列表
        """
        trajectory = []
        
        # 抓取轨迹
        pick_traj = self.generate_pick_trajectory(current_position)
        trajectory.extend(pick_traj)
        
        # 放置轨迹
        place_traj = self.generate_place_trajectory(pallet_index)
        trajectory.extend(place_traj)
        
        return trajectory
    
    def generate_full_program(self, start_position: np.ndarray) -> List[dict]:
        """
        生成完整码垛程序
        
        Args:
            start_position: 起始位置
            
        Returns:
            完整程序轨迹
        """
        program = []
        
        for i in range(len(self._pallet_points)):
            cycle = self.generate_full_cycle(i, start_position)
            program.append({
                'cycle': i + 1,
                'point': self._pallet_points[i],
                'trajectory': cycle
            })
        
        return program
    
    def visualize_layout(self) -> str:
        """生成码垛布局可视化文本"""
        cfg = self.config
        
        lines = [
            "码垛布局:",
            f"  托盘原点: {cfg.pallet_origin}",
            f"  尺寸: {cfg.rows}行 x {cfg.cols}列 x {cfg.layers}层",
            f"  工件尺寸: {cfg.box_length} x {cfg.box_width} x {cfg.box_height} mm",
            f"  总数量: {self.get_total_count()}",
            "",
            "点位列表:"
        ]
        
        for pt in self._pallet_points:
            lines.append(
                f"  [{pt.index:3d}] 层{pt.layer+1} 行{pt.row+1} 列{pt.col+1}: "
                f"({pt.position[0]:.1f}, {pt.position[1]:.1f}, {pt.position[2]:.1f})"
            )
        
        return "\n".join(lines)
    
    def update_config(self, **kwargs):
        """更新配置参数"""
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
        
        # 重新计算点位
        self._calculate_pallet_points()


if __name__ == "__main__":
    # 测试代码
    print("码垛规划器测试")
    print("=" * 60)
    
    # 创建规划器
    config = PalletConfig(
        rows=3,
        cols=4,
        layers=2,
        box_length=100.0,
        box_width=80.0,
        box_height=50.0
    )
    
    planner = PalletizingPlanner(config)
    
    # 显示布局
    print(planner.visualize_layout())
    
    # 生成单个循环
    print("\n" + "=" * 60)
    print("单循环轨迹 (第1个点位):")
    
    current_pos = np.array([300.0, 0.0, 300.0])
    cycle = planner.generate_full_cycle(0, current_pos)
    
    for i, step in enumerate(cycle):
        print(f"  步骤 {i+1}: {step['description']}")
        if step['type'] == 'linear':
            print(f"         目标: {step['target']}")
            print(f"         速度: {step['velocity']}%")
        elif step['type'] == 'gripper':
            print(f"         动作: {step['action']}")
    
    # 测试不同码垛模式
    print("\n" + "=" * 60)
    print("之字形模式测试:")
    
    config.pattern = PalletPattern.ZIGZAG
    planner.update_config(pattern=PalletPattern.ZIGZAG)
    
    points = planner.get_pallet_points()
    for pt in points[:6]:  # 只显示前6个
        print(f"  [{pt.index:3d}] 层{pt.layer+1} 行{pt.row+1} 列{pt.col+1}")

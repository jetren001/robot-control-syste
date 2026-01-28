"""
报警管理模块
实现报警的记录、显示和管理

作者: Cursor AI
日期: 2026-01-28
"""

from enum import Enum
from typing import List, Optional, Callable
from dataclasses import dataclass, field
from datetime import datetime
import threading
import json


class AlarmSeverity(Enum):
    """报警严重程度"""
    INFO = "info"        # 信息
    WARNING = "warning"  # 警告
    ERROR = "error"      # 错误
    CRITICAL = "critical"  # 严重


@dataclass
class Alarm:
    """报警数据类"""
    id: int
    code: str
    message: str
    severity: AlarmSeverity
    source: str  # 报警来源 (如: J1, System, etc.)
    timestamp: datetime = field(default_factory=datetime.now)
    acknowledged: bool = False
    cleared: bool = False
    
    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'code': self.code,
            'message': self.message,
            'severity': self.severity.value,
            'source': self.source,
            'timestamp': self.timestamp.isoformat(),
            'acknowledged': self.acknowledged,
            'cleared': self.cleared
        }
    
    def __repr__(self):
        return f"Alarm({self.code}: {self.message})"


class AlarmManager:
    """
    报警管理器
    
    功能:
    - 报警记录和存储
    - 报警确认和清除
    - 报警历史查询
    - 报警回调通知
    """
    
    def __init__(self, max_history: int = 1000):
        """
        初始化报警管理器
        
        Args:
            max_history: 最大历史记录数
        """
        self._alarms: List[Alarm] = []
        self._history: List[Alarm] = []
        self._max_history = max_history
        self._next_id = 1
        self._lock = threading.Lock()
        
        # 回调函数
        self._on_alarm: Optional[Callable[[Alarm], None]] = None
        self._on_clear: Optional[Callable[[Alarm], None]] = None
        
        # 预定义报警代码
        self._alarm_codes = {
            'E001': ('过流保护', AlarmSeverity.ERROR),
            'E002': ('过压保护', AlarmSeverity.ERROR),
            'E003': ('欠压保护', AlarmSeverity.ERROR),
            'E004': ('过温保护', AlarmSeverity.ERROR),
            'E005': ('编码器故障', AlarmSeverity.ERROR),
            'E006': ('位置超限', AlarmSeverity.ERROR),
            'E007': ('通信故障', AlarmSeverity.ERROR),
            'E008': ('跟随误差', AlarmSeverity.ERROR),
            'E009': ('急停触发', AlarmSeverity.CRITICAL),
            'E010': ('使能失败', AlarmSeverity.ERROR),
            'W001': ('温度预警', AlarmSeverity.WARNING),
            'W002': ('电压偏低', AlarmSeverity.WARNING),
            'W003': ('负载偏高', AlarmSeverity.WARNING),
            'I001': ('程序完成', AlarmSeverity.INFO),
            'I002': ('回原点完成', AlarmSeverity.INFO),
        }
    
    def add_alarm(self, code: str, message: Optional[str] = None, 
                  source: str = "System", 
                  severity: Optional[AlarmSeverity] = None) -> Alarm:
        """
        添加报警
        
        Args:
            code: 报警代码
            message: 报警消息 (可选,使用预定义消息)
            source: 报警来源
            severity: 严重程度 (可选,使用预定义级别)
            
        Returns:
            创建的报警对象
        """
        with self._lock:
            # 获取预定义信息
            if code in self._alarm_codes:
                default_msg, default_severity = self._alarm_codes[code]
                if message is None:
                    message = default_msg
                if severity is None:
                    severity = default_severity
            else:
                if message is None:
                    message = f"未知报警 ({code})"
                if severity is None:
                    severity = AlarmSeverity.WARNING
            
            alarm = Alarm(
                id=self._next_id,
                code=code,
                message=message,
                severity=severity,
                source=source
            )
            
            self._next_id += 1
            self._alarms.append(alarm)
            self._history.append(alarm)
            
            # 限制历史记录大小
            if len(self._history) > self._max_history:
                self._history = self._history[-self._max_history:]
        
        # 触发回调
        if self._on_alarm:
            try:
                self._on_alarm(alarm)
            except Exception:
                pass
        
        return alarm
    
    def acknowledge_alarm(self, alarm_id: int) -> bool:
        """
        确认报警
        
        Args:
            alarm_id: 报警ID
            
        Returns:
            是否成功
        """
        with self._lock:
            for alarm in self._alarms:
                if alarm.id == alarm_id:
                    alarm.acknowledged = True
                    return True
        return False
    
    def acknowledge_all(self) -> int:
        """
        确认所有报警
        
        Returns:
            确认的报警数量
        """
        count = 0
        with self._lock:
            for alarm in self._alarms:
                if not alarm.acknowledged:
                    alarm.acknowledged = True
                    count += 1
        return count
    
    def clear_alarm(self, alarm_id: int) -> bool:
        """
        清除报警
        
        Args:
            alarm_id: 报警ID
            
        Returns:
            是否成功
        """
        alarm_to_clear = None
        with self._lock:
            for i, alarm in enumerate(self._alarms):
                if alarm.id == alarm_id:
                    alarm.cleared = True
                    alarm_to_clear = self._alarms.pop(i)
                    break
        
        if alarm_to_clear and self._on_clear:
            try:
                self._on_clear(alarm_to_clear)
            except Exception:
                pass
        
        return alarm_to_clear is not None
    
    def clear_all(self) -> int:
        """
        清除所有已确认的报警
        
        Returns:
            清除的报警数量
        """
        cleared = []
        with self._lock:
            remaining = []
            for alarm in self._alarms:
                if alarm.acknowledged:
                    alarm.cleared = True
                    cleared.append(alarm)
                else:
                    remaining.append(alarm)
            self._alarms = remaining
        
        for alarm in cleared:
            if self._on_clear:
                try:
                    self._on_clear(alarm)
                except Exception:
                    pass
        
        return len(cleared)
    
    def get_active_alarms(self) -> List[Alarm]:
        """获取活动报警列表"""
        with self._lock:
            return [a for a in self._alarms if not a.cleared]
    
    def get_unacknowledged_alarms(self) -> List[Alarm]:
        """获取未确认的报警列表"""
        with self._lock:
            return [a for a in self._alarms if not a.acknowledged]
    
    def get_alarm_history(self, count: Optional[int] = None) -> List[Alarm]:
        """
        获取报警历史
        
        Args:
            count: 返回数量, None返回全部
        """
        with self._lock:
            if count is None:
                return self._history.copy()
            return self._history[-count:]
    
    def has_active_alarms(self) -> bool:
        """是否有活动报警"""
        with self._lock:
            return len(self._alarms) > 0
    
    def has_error(self) -> bool:
        """是否有错误级别以上的报警"""
        with self._lock:
            for alarm in self._alarms:
                if alarm.severity in [AlarmSeverity.ERROR, AlarmSeverity.CRITICAL]:
                    return True
        return False
    
    def get_alarm_count(self) -> dict:
        """获取各级别报警数量"""
        counts = {s.value: 0 for s in AlarmSeverity}
        with self._lock:
            for alarm in self._alarms:
                counts[alarm.severity.value] += 1
        return counts
    
    def set_on_alarm(self, callback: Callable[[Alarm], None]):
        """设置报警回调"""
        self._on_alarm = callback
    
    def set_on_clear(self, callback: Callable[[Alarm], None]):
        """设置清除回调"""
        self._on_clear = callback
    
    def export_history(self, filepath: str):
        """导出报警历史到JSON文件"""
        with self._lock:
            data = [a.to_dict() for a in self._history]
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    
    def get_alarm_code_list(self) -> dict:
        """获取预定义报警代码列表"""
        return {code: (msg, sev.value) for code, (msg, sev) in self._alarm_codes.items()}


if __name__ == "__main__":
    # 测试代码
    print("报警管理器测试")
    print("=" * 60)
    
    manager = AlarmManager()
    
    # 设置回调
    def on_alarm(alarm):
        print(f"  [新报警] {alarm.code}: {alarm.message} ({alarm.severity.value})")
    
    manager.set_on_alarm(on_alarm)
    
    # 添加报警
    print("\n添加报警:")
    manager.add_alarm('E001', source='J1')
    manager.add_alarm('W001', source='J2')
    manager.add_alarm('E009', source='System')
    manager.add_alarm('CUSTOM', message='自定义报警', severity=AlarmSeverity.WARNING)
    
    # 显示活动报警
    print("\n活动报警:")
    for alarm in manager.get_active_alarms():
        print(f"  [{alarm.id}] {alarm.code}: {alarm.message}")
    
    # 确认报警
    print("\n确认所有报警...")
    manager.acknowledge_all()
    
    # 清除已确认的报警
    print("清除已确认的报警...")
    cleared = manager.clear_all()
    print(f"  清除了 {cleared} 个报警")
    
    # 报警统计
    print("\n报警统计:")
    counts = manager.get_alarm_count()
    for level, count in counts.items():
        print(f"  {level}: {count}")

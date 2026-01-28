"""
机器人状态机模块
实现设备主状态机，管理机器人的各种工作状态

作者: Cursor AI
日期: 2026-01-28
"""

from enum import Enum, auto
from typing import Optional, Callable, Dict, List, Set
from dataclasses import dataclass, field
import time
import threading
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotState(Enum):
    """机器人状态枚举"""
    IDLE = auto()           # 空闲/上电初始状态
    SCANNING = auto()       # 扫描总线
    READY = auto()          # 就绪 (扫描完成,未使能)
    ENABLED = auto()        # 伺服已使能
    HOMING = auto()         # 回原点中
    STANDBY = auto()        # 待机 (已回原点,可操作)
    JOG = auto()            # 手动点动模式
    AUTO_RUN = auto()       # 自动运行模式
    PAUSED = auto()         # 暂停
    FAULT = auto()          # 故障状态
    EMERGENCY_STOP = auto() # 急停状态


class RobotEvent(Enum):
    """状态机事件枚举"""
    # 系统事件
    POWER_ON = auto()       # 上电
    SCAN_START = auto()     # 开始扫描
    SCAN_COMPLETE = auto()  # 扫描完成
    SCAN_FAILED = auto()    # 扫描失败
    
    # 伺服事件
    SERVO_ENABLE = auto()   # 伺服使能
    SERVO_DISABLE = auto()  # 伺服禁止
    ENABLE_SUCCESS = auto() # 使能成功
    ENABLE_FAILED = auto()  # 使能失败
    
    # 运动事件
    HOME_START = auto()     # 开始回原点
    HOME_COMPLETE = auto()  # 回原点完成
    HOME_FAILED = auto()    # 回原点失败
    
    # 模式切换
    JOG_MODE = auto()       # 切换到点动模式
    AUTO_MODE = auto()      # 切换到自动模式
    MANUAL_MODE = auto()    # 切换到手动模式
    
    # 程序控制
    PROGRAM_START = auto()  # 启动程序
    PROGRAM_STOP = auto()   # 停止程序
    PROGRAM_PAUSE = auto()  # 暂停程序
    PROGRAM_RESUME = auto() # 恢复程序
    PROGRAM_COMPLETE = auto() # 程序完成
    
    # 故障事件
    FAULT_DETECTED = auto() # 检测到故障
    FAULT_RESET = auto()    # 故障复位
    EMERGENCY_STOP = auto() # 急停触发
    EMERGENCY_RELEASE = auto() # 急停解除


@dataclass
class StateTransition:
    """状态转换定义"""
    from_state: RobotState
    event: RobotEvent
    to_state: RobotState
    condition: Optional[Callable[[], bool]] = None  # 转换条件
    action: Optional[Callable[[], None]] = None     # 转换动作


@dataclass
class StateInfo:
    """状态信息"""
    state: RobotState
    entry_time: float
    message: str = ""
    data: Dict = field(default_factory=dict)


class RobotStateMachine:
    """
    机器人状态机
    
    状态转换图:
    
    IDLE ─────► SCANNING ─────► READY ─────► ENABLED
      ▲            │              │             │
      │            │              │             ▼
      │            ▼              │          HOMING
      │          FAULT◄───────────┼─────────────┤
      │            ▲              │             ▼
      │            │              │          STANDBY
      │            │              │          /     \\
      │            │              │         ▼       ▼
      │            └──────────────┼────── JOG   AUTO_RUN
      │                           │         │       │
      │                           │         ▼       ▼
      └───────────────────────────┴─── EMERGENCY_STOP
    """
    
    def __init__(self):
        """初始化状态机"""
        self._current_state = RobotState.IDLE
        self._previous_state: Optional[RobotState] = None
        self._state_entry_time = time.time()
        self._state_history: List[StateInfo] = []
        
        # 回调函数
        self._on_state_change: Optional[Callable[[RobotState, RobotState], None]] = None
        self._on_fault: Optional[Callable[[str], None]] = None
        
        # 状态数据
        self._fault_message: str = ""
        self._is_homed: bool = False
        self._is_virtual_mode: bool = True
        
        # 线程锁
        self._lock = threading.Lock()
        
        # 定义状态转换表
        self._transitions = self._build_transitions()
        
        # 有效转换映射 (优化查找)
        self._transition_map: Dict[RobotState, Dict[RobotEvent, StateTransition]] = {}
        for t in self._transitions:
            if t.from_state not in self._transition_map:
                self._transition_map[t.from_state] = {}
            self._transition_map[t.from_state][t.event] = t
        
        logger.info(f"状态机初始化完成, 初始状态: {self._current_state.name}")
    
    def _build_transitions(self) -> List[StateTransition]:
        """构建状态转换表"""
        return [
            # IDLE 状态转换
            StateTransition(RobotState.IDLE, RobotEvent.SCAN_START, RobotState.SCANNING),
            StateTransition(RobotState.IDLE, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.IDLE, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # SCANNING 状态转换
            StateTransition(RobotState.SCANNING, RobotEvent.SCAN_COMPLETE, RobotState.READY),
            StateTransition(RobotState.SCANNING, RobotEvent.SCAN_FAILED, RobotState.FAULT),
            StateTransition(RobotState.SCANNING, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # READY 状态转换
            StateTransition(RobotState.READY, RobotEvent.SERVO_ENABLE, RobotState.ENABLED,
                           action=self._action_enable_servo),
            StateTransition(RobotState.READY, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.READY, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # ENABLED 状态转换
            StateTransition(RobotState.ENABLED, RobotEvent.HOME_START, RobotState.HOMING),
            StateTransition(RobotState.ENABLED, RobotEvent.SERVO_DISABLE, RobotState.READY,
                           action=self._action_disable_servo),
            StateTransition(RobotState.ENABLED, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.ENABLED, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            # 如果已经回过原点,可以直接进入STANDBY
            StateTransition(RobotState.ENABLED, RobotEvent.HOME_COMPLETE, RobotState.STANDBY,
                           condition=lambda: self._is_homed),
            
            # HOMING 状态转换
            StateTransition(RobotState.HOMING, RobotEvent.HOME_COMPLETE, RobotState.STANDBY,
                           action=self._action_home_complete),
            StateTransition(RobotState.HOMING, RobotEvent.HOME_FAILED, RobotState.FAULT),
            StateTransition(RobotState.HOMING, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.HOMING, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # STANDBY 状态转换
            StateTransition(RobotState.STANDBY, RobotEvent.JOG_MODE, RobotState.JOG),
            StateTransition(RobotState.STANDBY, RobotEvent.AUTO_MODE, RobotState.AUTO_RUN),
            StateTransition(RobotState.STANDBY, RobotEvent.PROGRAM_START, RobotState.AUTO_RUN),
            StateTransition(RobotState.STANDBY, RobotEvent.SERVO_DISABLE, RobotState.READY,
                           action=self._action_disable_servo),
            StateTransition(RobotState.STANDBY, RobotEvent.HOME_START, RobotState.HOMING),
            StateTransition(RobotState.STANDBY, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.STANDBY, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # JOG 状态转换
            StateTransition(RobotState.JOG, RobotEvent.MANUAL_MODE, RobotState.STANDBY),
            StateTransition(RobotState.JOG, RobotEvent.PROGRAM_STOP, RobotState.STANDBY),
            StateTransition(RobotState.JOG, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.JOG, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # AUTO_RUN 状态转换
            StateTransition(RobotState.AUTO_RUN, RobotEvent.PROGRAM_STOP, RobotState.STANDBY),
            StateTransition(RobotState.AUTO_RUN, RobotEvent.PROGRAM_COMPLETE, RobotState.STANDBY),
            StateTransition(RobotState.AUTO_RUN, RobotEvent.PROGRAM_PAUSE, RobotState.PAUSED),
            StateTransition(RobotState.AUTO_RUN, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.AUTO_RUN, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # PAUSED 状态转换
            StateTransition(RobotState.PAUSED, RobotEvent.PROGRAM_RESUME, RobotState.AUTO_RUN),
            StateTransition(RobotState.PAUSED, RobotEvent.PROGRAM_STOP, RobotState.STANDBY),
            StateTransition(RobotState.PAUSED, RobotEvent.FAULT_DETECTED, RobotState.FAULT),
            StateTransition(RobotState.PAUSED, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # FAULT 状态转换
            StateTransition(RobotState.FAULT, RobotEvent.FAULT_RESET, RobotState.IDLE,
                           action=self._action_fault_reset),
            StateTransition(RobotState.FAULT, RobotEvent.EMERGENCY_STOP, RobotState.EMERGENCY_STOP),
            
            # EMERGENCY_STOP 状态转换
            StateTransition(RobotState.EMERGENCY_STOP, RobotEvent.EMERGENCY_RELEASE, RobotState.FAULT),
        ]
    
    # ==================== 属性 ====================
    
    @property
    def current_state(self) -> RobotState:
        """获取当前状态"""
        return self._current_state
    
    @property
    def previous_state(self) -> Optional[RobotState]:
        """获取上一个状态"""
        return self._previous_state
    
    @property
    def state_duration(self) -> float:
        """获取当前状态持续时间(秒)"""
        return time.time() - self._state_entry_time
    
    @property
    def is_homed(self) -> bool:
        """是否已回原点"""
        return self._is_homed
    
    @property
    def fault_message(self) -> str:
        """获取故障信息"""
        return self._fault_message
    
    @property
    def is_running(self) -> bool:
        """是否在运行中"""
        return self._current_state in [RobotState.AUTO_RUN, RobotState.HOMING, RobotState.JOG]
    
    @property
    def is_ready_for_motion(self) -> bool:
        """是否可以运动"""
        return self._current_state in [RobotState.STANDBY, RobotState.JOG, RobotState.AUTO_RUN]
    
    # ==================== 事件处理 ====================
    
    def trigger(self, event: RobotEvent, data: Dict = None) -> bool:
        """
        触发事件
        
        Args:
            event: 事件类型
            data: 附加数据
            
        Returns:
            bool: 转换是否成功
        """
        with self._lock:
            return self._process_event(event, data or {})
    
    def _process_event(self, event: RobotEvent, data: Dict) -> bool:
        """处理事件"""
        # 查找有效转换
        if self._current_state not in self._transition_map:
            logger.warning(f"状态 {self._current_state.name} 没有定义转换")
            return False
        
        if event not in self._transition_map[self._current_state]:
            logger.debug(f"状态 {self._current_state.name} 不接受事件 {event.name}")
            return False
        
        transition = self._transition_map[self._current_state][event]
        
        # 检查条件
        if transition.condition and not transition.condition():
            logger.debug(f"转换条件不满足: {self._current_state.name} -> {transition.to_state.name}")
            return False
        
        # 执行转换
        old_state = self._current_state
        new_state = transition.to_state
        
        # 执行转换动作
        if transition.action:
            try:
                transition.action()
            except Exception as e:
                logger.error(f"转换动作执行失败: {e}")
                return False
        
        # 更新状态
        self._previous_state = old_state
        self._current_state = new_state
        self._state_entry_time = time.time()
        
        # 记录历史
        self._state_history.append(StateInfo(
            state=new_state,
            entry_time=self._state_entry_time,
            message=f"从 {old_state.name} 转换",
            data=data
        ))
        
        # 保留最近100条历史
        if len(self._state_history) > 100:
            self._state_history = self._state_history[-100:]
        
        logger.info(f"状态转换: {old_state.name} -> {new_state.name} (事件: {event.name})")
        
        # 触发回调
        if self._on_state_change:
            try:
                self._on_state_change(old_state, new_state)
            except Exception as e:
                logger.error(f"状态变更回调执行失败: {e}")
        
        # 故障状态特殊处理
        if new_state == RobotState.FAULT and self._on_fault:
            try:
                self._on_fault(data.get('message', '未知故障'))
            except Exception as e:
                logger.error(f"故障回调执行失败: {e}")
        
        return True
    
    # ==================== 转换动作 ====================
    
    def _action_enable_servo(self):
        """伺服使能动作"""
        logger.info("执行伺服使能")
    
    def _action_disable_servo(self):
        """伺服禁止动作"""
        logger.info("执行伺服禁止")
    
    def _action_home_complete(self):
        """回原点完成动作"""
        self._is_homed = True
        logger.info("回原点完成")
    
    def _action_fault_reset(self):
        """故障复位动作"""
        self._fault_message = ""
        logger.info("故障已复位")
    
    # ==================== 便捷方法 ====================
    
    def start_scan(self) -> bool:
        """开始扫描总线"""
        return self.trigger(RobotEvent.SCAN_START)
    
    def scan_complete(self) -> bool:
        """扫描完成"""
        return self.trigger(RobotEvent.SCAN_COMPLETE)
    
    def enable_servo(self) -> bool:
        """使能伺服"""
        return self.trigger(RobotEvent.SERVO_ENABLE)
    
    def disable_servo(self) -> bool:
        """禁止伺服"""
        return self.trigger(RobotEvent.SERVO_DISABLE)
    
    def start_homing(self) -> bool:
        """开始回原点"""
        return self.trigger(RobotEvent.HOME_START)
    
    def homing_complete(self) -> bool:
        """回原点完成"""
        return self.trigger(RobotEvent.HOME_COMPLETE)
    
    def start_program(self) -> bool:
        """启动程序"""
        return self.trigger(RobotEvent.PROGRAM_START)
    
    def stop_program(self) -> bool:
        """停止程序"""
        return self.trigger(RobotEvent.PROGRAM_STOP)
    
    def pause_program(self) -> bool:
        """暂停程序"""
        return self.trigger(RobotEvent.PROGRAM_PAUSE)
    
    def resume_program(self) -> bool:
        """恢复程序"""
        return self.trigger(RobotEvent.PROGRAM_RESUME)
    
    def enter_jog_mode(self) -> bool:
        """进入点动模式"""
        return self.trigger(RobotEvent.JOG_MODE)
    
    def exit_jog_mode(self) -> bool:
        """退出点动模式"""
        return self.trigger(RobotEvent.MANUAL_MODE)
    
    def report_fault(self, message: str) -> bool:
        """报告故障"""
        self._fault_message = message
        return self.trigger(RobotEvent.FAULT_DETECTED, {'message': message})
    
    def reset_fault(self) -> bool:
        """复位故障"""
        return self.trigger(RobotEvent.FAULT_RESET)
    
    def emergency_stop(self) -> bool:
        """触发急停"""
        return self.trigger(RobotEvent.EMERGENCY_STOP)
    
    def release_emergency(self) -> bool:
        """解除急停"""
        return self.trigger(RobotEvent.EMERGENCY_RELEASE)
    
    # ==================== 回调设置 ====================
    
    def set_on_state_change(self, callback: Callable[[RobotState, RobotState], None]):
        """设置状态变更回调"""
        self._on_state_change = callback
    
    def set_on_fault(self, callback: Callable[[str], None]):
        """设置故障回调"""
        self._on_fault = callback
    
    # ==================== 状态查询 ====================
    
    def get_valid_events(self) -> List[RobotEvent]:
        """获取当前状态下的有效事件"""
        if self._current_state in self._transition_map:
            return list(self._transition_map[self._current_state].keys())
        return []
    
    def can_transition_to(self, target_state: RobotState) -> bool:
        """检查是否可以转换到目标状态"""
        if self._current_state in self._transition_map:
            for event, trans in self._transition_map[self._current_state].items():
                if trans.to_state == target_state:
                    if trans.condition is None or trans.condition():
                        return True
        return False
    
    def get_state_history(self, count: int = 10) -> List[StateInfo]:
        """获取状态历史"""
        return self._state_history[-count:]
    
    def get_state_info(self) -> Dict:
        """获取状态信息字典"""
        return {
            'current_state': self._current_state.name,
            'previous_state': self._previous_state.name if self._previous_state else None,
            'state_duration': self.state_duration,
            'is_homed': self._is_homed,
            'is_running': self.is_running,
            'fault_message': self._fault_message,
            'valid_events': [e.name for e in self.get_valid_events()]
        }


if __name__ == "__main__":
    # 测试代码
    print("机器人状态机测试")
    print("=" * 60)
    
    sm = RobotStateMachine()
    
    # 设置回调
    def on_state_change(old_state, new_state):
        print(f"  [回调] 状态变更: {old_state.name} -> {new_state.name}")
    
    def on_fault(message):
        print(f"  [回调] 故障: {message}")
    
    sm.set_on_state_change(on_state_change)
    sm.set_on_fault(on_fault)
    
    # 模拟正常启动流程
    print("\n正常启动流程:")
    print(f"当前状态: {sm.current_state.name}")
    
    print("\n1. 开始扫描...")
    sm.start_scan()
    print(f"   当前状态: {sm.current_state.name}")
    
    print("\n2. 扫描完成...")
    sm.scan_complete()
    print(f"   当前状态: {sm.current_state.name}")
    
    print("\n3. 使能伺服...")
    sm.enable_servo()
    print(f"   当前状态: {sm.current_state.name}")
    
    print("\n4. 开始回原点...")
    sm.start_homing()
    print(f"   当前状态: {sm.current_state.name}")
    
    print("\n5. 回原点完成...")
    sm.homing_complete()
    print(f"   当前状态: {sm.current_state.name}")
    
    print("\n6. 启动程序...")
    sm.start_program()
    print(f"   当前状态: {sm.current_state.name}")
    
    print("\n7. 程序停止...")
    sm.stop_program()
    print(f"   当前状态: {sm.current_state.name}")
    
    # 测试故障处理
    print("\n故障测试:")
    sm.report_fault("测试故障")
    print(f"   当前状态: {sm.current_state.name}")
    print(f"   故障信息: {sm.fault_message}")
    
    print("\n复位故障...")
    sm.reset_fault()
    print(f"   当前状态: {sm.current_state.name}")
    
    # 打印状态信息
    print("\n状态信息:")
    info = sm.get_state_info()
    for key, value in info.items():
        print(f"  {key}: {value}")

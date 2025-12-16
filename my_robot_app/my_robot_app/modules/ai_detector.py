# my_robot_app/modules/ai_detector.py
"""
AI检测器类 - 语音和手势识别的核心实现
作为语音和手势节点的上层协调器
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
from enum import Enum

class ControlMode(Enum):
    """控制模式枚举"""
    VOICE_ONLY = "voice_only"
    GESTURE_ONLY = "gesture_only" 
    BOTH = "both"
    FUSED = "fused"

class AIDetector(Node):
    """
    AI检测器类，负责协调语音和手势控制
    
    功能：
    1. 接收语音和手势命令
    2. 命令融合和优先级处理
    3. 发布统一控制命令
    4. 状态管理和冲突解决
    """
    
    def __init__(self):
        super().__init__('ai_detector')
        
        # 参数设置
        self.declare_parameter('control_mode', 'both')  # 'voice_only', 'gesture_only', 'both', 'fused'
        self.declare_parameter('voice_priority', 0.7)   # 语音命令优先级权重
        self.declare_parameter('gesture_priority', 0.3) # 手势命令优先级权重
        self.declare_parameter('timeout', 2.0)         # 命令超时时间（秒）
        self.declare_parameter('enable_fusion', True)  # 是否启用命令融合
        
        self.control_mode = self.get_parameter('control_mode').value
        self.voice_priority = self.get_parameter('voice_priority').value
        self.gesture_priority = self.get_parameter('gesture_priority').value
        self.timeout = self.get_parameter('timeout').value
        self.enable_fusion = self.get_parameter('enable_fusion').value
        
        # 发布者 - 发布融合后的控制命令
        self.fused_cmd_pub = self.create_publisher(Twist, '/cmd_vel_ai', 10)
        self.fused_action_pub = self.create_publisher(String, '/ai_action', 10)
        self.status_pub = self.create_publisher(String, '/ai_status', 10)
        
        # 订阅者 - 接收语音和手势命令
        self.voice_sub = self.create_subscription(
            String,
            '/voice_control',
            self.voice_callback,
            10
        )
        
        self.gesture_sub = self.create_subscription(
            String,
            '/gesture_control',
            self.gesture_callback,
            10
        )
        
        # 状态变量
        self.last_voice_command = None
        self.last_voice_time = 0
        self.last_gesture_command = None
        self.last_gesture_time = 0
        self.current_action = None
        
        # 命令映射
        self.command_velocity_map = {
            'move_forward': (0.2, 0.0),
            'move_backward': (-0.15, 0.0),
            'turn_left': (0.0, 0.5),
            'turn_right': (0.0, -0.5),
            'stop': (0.0, 0.0),
        }
        
        # 定时器
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)  # 10Hz融合频率
        self.status_timer = self.create_timer(2.0, self.status_callback)   # 状态发布
        
        self.get_logger().info(f"AI检测器初始化完成 [模式: {self.control_mode}]")
        self.get_logger().info(f"命令融合: {'启用' if self.enable_fusion else '禁用'}")
        self.get_logger().info(f"语音优先级: {self.voice_priority}, 手势优先级: {self.gesture_priority}")
    
    def voice_callback(self, msg):
        """语音命令回调"""
        command = msg.data
        current_time = time.time()
        
        self.get_logger().info(f"收到语音命令: {command}")
        
        # 更新语音命令状态
        self.last_voice_command = command
        self.last_voice_time = current_time
        
        # 如果模式是纯语音，立即执行
        if self.control_mode in ['voice_only', 'both']:
            self._execute_command(command, 'voice')
    
    def gesture_callback(self, msg):
        """手势命令回调"""
        command = msg.data
        current_time = time.time()
        
        self.get_logger().info(f"收到手势命令: {command}")
        
        # 更新手势命令状态
        self.last_gesture_command = command
        self.last_gesture_time = current_time
        
        # 如果模式是纯手势，立即执行
        if self.control_mode in ['gesture_only', 'both']:
            self._execute_command(command, 'gesture')
    
    def _execute_command(self, command, source):
        """执行单个命令"""
        current_time = time.time()
        
        # 检查命令是否过期
        source_time = self.last_voice_time if source == 'voice' else self.last_gesture_time
        if current_time - source_time > self.timeout:
            self.get_logger().warning(f"{source}命令已过期，忽略")
            return
        
        # 执行移动相关命令
        if command in self.command_velocity_map:
            linear, angular = self.command_velocity_map[command]
            
            vel_msg = Twist()
            vel_msg.linear.x = linear
            vel_msg.angular.z = angular
            
            self.fused_cmd_pub.publish(vel_msg)
            self.get_logger().info(f"执行{source}命令: {command} -> v={linear:.2f}, ω={angular:.2f}")
        
        # 执行动作相关命令（抓取、放置等）
        elif command in ['pick_object', 'place_object', 'go_home']:
            action_msg = String()
            action_msg.data = command
            self.fused_action_pub.publish(action_msg)
            self.current_action = command
            self.get_logger().info(f"执行{source}动作: {command}")
    
    def fusion_callback(self):
        """命令融合回调"""
        if not self.enable_fusion or self.control_mode != 'fused':
            return
        
        current_time = time.time()
        
        # 检查命令是否有效
        voice_valid = (self.last_voice_command is not None and 
                      current_time - self.last_voice_time <= self.timeout)
        gesture_valid = (self.last_gesture_command is not None and 
                        current_time - self.last_gesture_time <= self.timeout)
        
        # 如果没有有效命令，跳过融合
        if not voice_valid and not gesture_valid:
            return
        
        # 命令融合逻辑
        linear = 0.0
        angular = 0.0
        final_command = None
        
        if voice_valid and gesture_valid:
            # 双命令融合
            voice_linear, voice_angular = self.command_velocity_map.get(
                self.last_voice_command, (0.0, 0.0)
            )
            gesture_linear, gesture_angular = self.command_velocity_map.get(
                self.last_gesture_command, (0.0, 0.0)
            )
            
            # 加权融合
            linear = (voice_linear * self.voice_priority + 
                     gesture_linear * self.gesture_priority)
            angular = (voice_angular * self.voice_priority + 
                      gesture_angular * self.gesture_priority)
            
            final_command = f"融合[{self.last_voice_command}+{self.last_gesture_command}]"
            
        elif voice_valid:
            # 只有语音命令
            linear, angular = self.command_velocity_map.get(
                self.last_voice_command, (0.0, 0.0)
            )
            final_command = self.last_voice_command
            
        elif gesture_valid:
            # 只有手势命令
            linear, angular = self.command_velocity_map.get(
                self.last_gesture_command, (0.0, 0.0)
            )
            final_command = self.last_gesture_command
        
        # 发布融合后的速度命令
        if final_command:
            vel_msg = Twist()
            vel_msg.linear.x = linear
            vel_msg.angular.z = angular
            
            self.fused_cmd_pub.publish(vel_msg)
            
            # 记录融合状态
            self.get_logger().debug(f"命令融合: {final_command} -> v={linear:.2f}, ω={angular:.2f}")
    
    def status_callback(self):
        """状态发布回调"""
        current_time = time.time()
        
        voice_active = (self.last_voice_command is not None and 
                       current_time - self.last_voice_time <= self.timeout)
        gesture_active = (self.last_gesture_command is not None and 
                         current_time - self.last_gesture_time <= self.timeout)
        
        status_msg = String()
        
        if voice_active and gesture_active:
            status = f"双模控制[语音:{self.last_voice_command}, 手势:{self.last_gesture_command}]"
        elif voice_active:
            status = f"语音控制[{self.last_voice_command}]"
        elif gesture_active:
            status = f"手势控制[{self.last_gesture_command}]"
        else:
            status = "等待输入"
        
        if self.current_action:
            status += f" | 当前动作: {self.current_action}"
        
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def emergency_stop(self):
        """紧急停止"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.fused_cmd_pub.publish(stop_msg)
        
        self.last_voice_command = None
        self.last_gesture_command = None
        self.current_action = None
        
        self.get_logger().warn("⚠️ 紧急停止已触发")
    
    def destroy_node(self):
        """清理资源"""
        self.emergency_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AIDetector()
    
    try:
        node.get_logger().info("\n🤖 AI检测器启动")
        node.get_logger().info("💡 控制模式: " + node.control_mode)
        node.get_logger().info("💡 正在监听语音和手势命令...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"AI检测器错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

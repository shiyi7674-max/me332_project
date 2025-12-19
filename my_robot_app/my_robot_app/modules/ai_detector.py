#!/usr/bin/env python3
"""
AI检测与控制集成模块
支持分开启动手势控制、语音控制或两者同时运行
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time
import os
import sys

# 添加脚本目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(current_dir))
scripts_dir = os.path.join(project_root, 'scripts')

# 打印调试信息
print(f"当前目录: {current_dir}")
print(f"项目根目录: {project_root}")
print(f"脚本目录: {scripts_dir}")

# 添加可能的路径
possible_paths = [
    scripts_dir,
    current_dir,
    project_root,
    os.path.join(project_root, 'my_robot_app', 'scripts'),
    os.path.join(os.getcwd(), 'scripts')
]

for path in possible_paths:
    if os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)
        print(f"添加路径: {path}")

try:
    from gesture_ctrl_node import GestureControlNode
    from voice_ctrl_node import VoiceControlNode
    IMPORT_SUCCESS = True
    print("✅ 成功导入手势和语音控制模块")
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    print(f"当前sys.path: {sys.path}")
    
    # 创建虚拟类
    class GestureControlNode:
        def __init__(self):
            self.is_active = False
            self.linear_speed = 0.2
            self.angular_speed = 0.5
            self.command_cooldown = 1.0
            
        def start_capture(self): 
            print("虚拟手势控制 - 启动捕获")
            self.is_active = True
            
        def stop_capture(self): 
            print("虚拟手势控制 - 停止捕获")
            self.is_active = False
            
        def destroy_node(self):
            pass
    
    class VoiceControlNode:
        def __init__(self):
            self.is_active = False
            self.linear_speed = 0.2
            self.angular_speed = 0.5
            self.command_cooldown = 1.5
            
        def start_listening(self): 
            print("虚拟语音控制 - 开始监听")
            self.is_active = True
            
        def stop_listening(self): 
            print("虚拟语音控制 - 停止监听")
            self.is_active = False
            
        def destroy_node(self):
            pass
            
    IMPORT_SUCCESS = False

class AIDetector(Node):
    """
    AI检测与控制集成类
    支持三种模式: gesture_only, voice_only, combined
    新增功能: 可以分别启动手势和语音控制
    """
    
    def __init__(self):
        super().__init__('ai_detector')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 AI控制集成系统启动")
        self.get_logger().info("=" * 60)
        
        if not IMPORT_SUCCESS:
            self.get_logger().warn("⚠️ 使用虚拟手势和语音控制模块")
            self.get_logger().info("💡 真实功能需要将 gesture_ctrl_node.py 和 voice_ctrl_node.py 放在 scripts/ 目录")
        
        # 参数声明
        self.declare_parameter('mode', 'combined')  # gesture_only, voice_only, combined
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('gesture_cooldown', 1.0)
        self.declare_parameter('voice_cooldown', 1.5)
        self.declare_parameter('auto_start', False)  # 默认不自动启动
        
        # 获取参数
        self.mode = self.get_parameter('mode').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.gesture_cooldown = self.get_parameter('gesture_cooldown').value
        self.voice_cooldown = self.get_parameter('voice_cooldown').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # 状态变量
        self.current_command = None
        self.current_velocity = Twist()
        self.last_command_time = 0
        self.command_source = None
        
        # 控制实例
        self.gesture_controller = None
        self.voice_controller = None
        
        # 控制状态标志
        self.gesture_active = False
        self.voice_active = False
        self.is_running = False
        
        # 控制线程
        self.control_thread = None
        
        # 初始化发布者和订阅者
        self._init_ros_components()
        
        # 显示启动信息
        self._show_startup_info()
        
        # 自动启动
        if self.auto_start:
            if self.mode == 'gesture_only':
                self.start_gesture_control()
            elif self.mode == 'voice_only':
                self.start_voice_control()
            elif self.mode == 'combined':
                self.start_combined_control()
    
    def _init_ros_components(self):
        """初始化ROS组件"""
        # 发布者
        self.ai_command_pub = self.create_publisher(String, '/ai_control', 10)
        self.ai_velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅者
        self.gesture_sub = self.create_subscription(
            String, '/gesture_control', self._gesture_callback, 10
        )
        self.voice_sub = self.create_subscription(
            String, '/voice_control', self._voice_callback, 10
        )
        
        # 定时器
        self.velocity_timer = self.create_timer(0.2, self._publish_velocity)
        
        self.get_logger().info("ROS组件初始化完成")
    
    def _show_startup_info(self):
        """显示启动信息"""
        self.get_logger().info("📊 控制参数:")
        self.get_logger().info(f"  • 控制模式: {self.mode}")
        self.get_logger().info(f"  • 线速度: {self.linear_speed} m/s")
        self.get_logger().info(f"  • 角速度: {self.angular_speed} rad/s")
        self.get_logger().info(f"  • 手势冷却: {self.gesture_cooldown} s")
        self.get_logger().info(f"  • 语音冷却: {self.voice_cooldown} s")
        self.get_logger().info("")
        
        self.get_logger().info("🎮 键盘控制:")
        self.get_logger().info("  • g: 启动手势控制")
        self.get_logger().info("  • v: 启动语音控制")
        self.get_logger().info("  • b: 启动双模式控制")
        self.get_logger().info("  • s: 停止所有控制")
        self.get_logger().info("  • sg: 停止手势控制")
        self.get_logger().info("  • sv: 停止语音控制")
        self.get_logger().info("  • q: 退出程序")
        self.get_logger().info("  • +: 增加速度")
        self.get_logger().info("  • -: 减少速度")
        self.get_logger().info("=" * 60)
    
    def _gesture_callback(self, msg):
        """手势命令回调"""
        if not self.gesture_active:
            return
        
        current_time = time.time()
        
        if current_time - self.last_command_time < self.gesture_cooldown:
            return
        
        command = msg.data
        self.current_command = command
        self.command_source = 'gesture'
        self.last_command_time = current_time
        
        self._process_command(command)
        self.get_logger().info(f"👋 手势: {command}")
    
    def _voice_callback(self, msg):
        """语音命令回调"""
        if not self.voice_active:
            return
        
        current_time = time.time()
        
        if current_time - self.last_command_time < self.voice_cooldown:
            return
        
        command = msg.data
        self.current_command = command
        self.command_source = 'voice'
        self.last_command_time = current_time
        
        self._process_command(command)
        self.get_logger().info(f"🎤 语音: {command}")
    
    def _process_command(self, command):
        """处理控制命令"""
        # 发布AI控制命令
        ai_msg = String()
        ai_msg.data = f"{self.command_source}:{command}"
        self.ai_command_pub.publish(ai_msg)
        
        # 设置速度命令
        vel = Twist()
        
        if command == 'forward':
            vel.linear.x = self.linear_speed
            vel.angular.z = 0.0
        elif command == 'backward':
            vel.linear.x = -self.linear_speed * 0.75
            vel.angular.z = 0.0
        elif command == 'left':
            vel.linear.x = 0.0
            vel.angular.z = self.angular_speed
        elif command == 'right':
            vel.linear.x = 0.0
            vel.angular.z = -self.angular_speed
        elif command == 'stop':
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        elif command in ['grab', 'release']:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        
        self.current_velocity = vel
        self.ai_velocity_pub.publish(vel)
    
    def _publish_velocity(self):
        """定时发布速度命令"""
        if (self.gesture_active or self.voice_active) and rclpy.ok():
            try:
                self.ai_velocity_pub.publish(self.current_velocity)
            except:
                pass
    
    # 新增功能：分别启动手势、语音或双模式控制
    
    def start_gesture_control(self):
        """仅启动手势控制"""
        if self.gesture_active:
            self.get_logger().warn("手势控制已经在运行中")
            return
        
        try:
            self.gesture_controller = GestureControlNode()
            self.gesture_controller.linear_speed = self.linear_speed
            self.gesture_controller.angular_speed = self.angular_speed
            self.gesture_controller.command_cooldown = self.gesture_cooldown
            
            # 在新线程中启动手势控制
            gesture_thread = threading.Thread(target=self.gesture_controller.start_capture)
            gesture_thread.daemon = True
            gesture_thread.start()
            
            self.gesture_active = True
            self.is_running = True
            self.mode = 'gesture_only'
            self.get_logger().info("✅ 手势控制已启动")
        except Exception as e:
            self.get_logger().error(f"❌ 手势控制启动失败: {e}")
    
    def start_voice_control(self):
        """仅启动语音控制"""
        if self.voice_active:
            self.get_logger().warn("语音控制已经在运行中")
            return
        
        try:
            self.voice_controller = VoiceControlNode()
            self.voice_controller.linear_speed = self.linear_speed
            self.voice_controller.angular_speed = self.angular_speed
            self.voice_controller.command_cooldown = self.voice_cooldown
            
            # 在新线程中启动语音控制
            voice_thread = threading.Thread(target=self.voice_controller.start_listening)
            voice_thread.daemon = True
            voice_thread.start()
            
            self.voice_active = True
            self.is_running = True
            self.mode = 'voice_only'
            self.get_logger().info("✅ 语音控制已启动")
        except Exception as e:
            self.get_logger().error(f"❌ 语音控制启动失败: {e}")
    
    def start_combined_control(self):
        """启动手势+语音双控制"""
        self.start_gesture_control()
        time.sleep(0.5)  # 短暂延迟
        self.start_voice_control()
        self.mode = 'combined'
        self.get_logger().info("✅ 手势+语音双控制已启动")
    
    # 修改原来的start_control方法以兼容旧代码
    def start_control(self):
        """启动AI控制（兼容旧版）"""
        if self.mode == 'gesture_only':
            self.start_gesture_control()
        elif self.mode == 'voice_only':
            self.start_voice_control()
        else:
            self.start_combined_control()
    
    # 新增功能：分别停止手势、语音控制
    
    def stop_gesture_control(self):
        """停止手势控制"""
        if not self.gesture_active:
            self.get_logger().warn("手势控制未运行")
            return
        
        if self.gesture_controller:
            try:
                self.gesture_controller.stop_capture()
            except:
                pass
        
        self.gesture_active = False
        self.get_logger().info("⏹️ 手势控制已停止")
        
        # 如果没有其他控制激活，更新运行状态
        if not self.voice_active:
            self.is_running = False
    
    def stop_voice_control(self):
        """停止语音控制"""
        if not self.voice_active:
            self.get_logger().warn("语音控制未运行")
            return
        
        if self.voice_controller:
            try:
                self.voice_controller.stop_listening()
            except:
                pass
        
        self.voice_active = False
        self.get_logger().info("⏹️ 语音控制已停止")
        
        # 如果没有其他控制激活，更新运行状态
        if not self.gesture_active:
            self.is_running = False
    
    def stop_control(self):
        """停止所有AI控制"""
        self.stop_gesture_control()
        self.stop_voice_control()
        
        # 发布停止命令
        stop_vel = Twist()
        self.current_velocity = stop_vel
        self.ai_velocity_pub.publish(stop_vel)
        
        self.get_logger().info("⏹️ 所有AI控制已停止")
    
    def get_status(self):
        """获取状态信息"""
        return {
            'running': self.is_running,
            'gesture_active': self.gesture_active,
            'voice_active': self.voice_active,
            'mode': self.mode,
            'current_command': self.current_command,
            'command_source': self.command_source,
            'linear_speed': self.linear_speed,
            'angular_speed': self.angular_speed
        }
    
    def destroy_node(self):
        """清理资源"""
        self.stop_control()
        
        if hasattr(self, 'velocity_timer'):
            self.velocity_timer.cancel()
        
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 解析命令行参数
    import argparse
    parser = argparse.ArgumentParser(description='AI控制集成系统')
    parser.add_argument('--mode', type=str, default='none',
                       choices=['none', 'gesture', 'voice', 'combined'],
                       help='启动时自动开启的控制模式')
    parser.add_argument('--speed', type=float, default=0.2,
                       help='线速度')
    parser.add_argument('--turn-speed', type=float, default=0.5,
                       help='角速度')
    parser.add_argument('--gesture-cooldown', type=float, default=1.0,
                       help='手势命令冷却时间')
    parser.add_argument('--voice-cooldown', type=float, default=1.5,
                       help='语音命令冷却时间')
    
    # 解析ROS参数
    rclpy_args = rclpy.utilities.remove_ros_args(args)
    parsed_args = parser.parse_args(rclpy_args[1:])
    
    # 创建节点
    node = AIDetector()
    
    # 覆盖参数
    node.linear_speed = parsed_args.speed
    node.angular_speed = parsed_args.turn_speed
    node.gesture_cooldown = parsed_args.gesture_cooldown
    node.voice_cooldown = parsed_args.voice_cooldown
    
    # 根据参数自动启动
    if parsed_args.mode == 'gesture':
        node.start_gesture_control()
    elif parsed_args.mode == 'voice':
        node.start_voice_control()
    elif parsed_args.mode == 'combined':
        node.start_combined_control()
    
    # 键盘控制线程
    def keyboard_control():
        import select
        import termios
        import tty
        
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            last_key = ''
            last_key_time = time.time()
            
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    current_time = time.time()
                    
                    # 处理组合键（如'sg'）
                    if current_time - last_key_time < 0.5:
                        if last_key == 's' and key == 'g':
                            node.stop_gesture_control()
                            last_key = ''
                            continue
                        elif last_key == 's' and key == 'v':
                            node.stop_voice_control()
                            last_key = ''
                            continue
                    
                    last_key = key
                    last_key_time = current_time
                    
                    # 单个键命令
                    if key == 'q':
                        node.get_logger().info("退出程序...")
                        rclpy.shutdown()
                        break
                    elif key == 'g':
                        node.start_gesture_control()
                    elif key == 'v':
                        node.start_voice_control()
                    elif key == 'b':
                        node.start_combined_control()
                    elif key == 's':
                        node.stop_control()
                    elif key == '+':
                        node.linear_speed = min(1.0, node.linear_speed + 0.05)
                        # 更新已启动的控制器速度
                        if node.gesture_controller:
                            node.gesture_controller.linear_speed = node.linear_speed
                        if node.voice_controller:
                            node.voice_controller.linear_speed = node.linear_speed
                        node.get_logger().info(f"速度增加到: {node.linear_speed}")
                    elif key == '-':
                        node.linear_speed = max(0.05, node.linear_speed - 0.05)
                        # 更新已启动的控制器速度
                        if node.gesture_controller:
                            node.gesture_controller.linear_speed = node.linear_speed
                        if node.voice_controller:
                            node.voice_controller.linear_speed = node.linear_speed
                        node.get_logger().info(f"速度减少到: {node.linear_speed}")
                    
                    # 状态显示
                    status = node.get_status()
                    if status['gesture_active'] or status['voice_active']:
                        node.get_logger().info(f"状态: 手势[{'✅' if status['gesture_active'] else '❌'}] 语音[{'✅' if status['voice_active'] else '❌'}]")
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    # 启动键盘控制线程
    kb_thread = threading.Thread(target=keyboard_control)
    kb_thread.daemon = True
    kb_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
手势控制节点
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import threading
import numpy as np
import time
import sys

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        
        # 参数 - 统一速度参数
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('linear_speed', 0.2)  # 统一线速度
        self.declare_parameter('angular_speed', 0.5)  # 统一角速度
        self.declare_parameter('show_video', True)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('flip_horizontal', True)
        self.declare_parameter('process_every_n_frames', 3)  # 增加跳帧，减少处理频率
        self.declare_parameter('image_scale', 0.5)
        self.declare_parameter('command_cooldown', 1.0)  # 新增：命令冷却时间（秒）
        
        self.camera_id = self.get_parameter('camera_id').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.show_video = self.get_parameter('show_video').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').value
        self.image_scale = self.get_parameter('image_scale').value
        self.command_cooldown = self.get_parameter('command_cooldown').value
        
        # 发布者
        self.gesture_pub = self.create_publisher(String, '/gesture_control', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 当前速度命令（持续发布）
        self.current_cmd = Twist()
        # 定时器：持续发布速度命令，降低频率
        self.cmd_publish_period = 0.2  # 降低到5Hz
        self.cmd_timer = self.create_timer(self.cmd_publish_period, self._publish_velocity)
        
        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
            model_complexity=0
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # 手势映射
        self.gesture_map = {
            'finger_left': 'left',         # 食指向左：左转
            'finger_right': 'right',       # 食指向右：右转
            'victory': 'forward',          # 比耶：前进
            'ok_sign': 'backward',         # OK手势：后退
            'four_fingers': 'stop',        # 比4手势：停止
            'five_fingers': 'release',     # 比5手势：放下
            'fist': 'grab',                # 握拳：抓取
        }
        
        # 控制标志
        self.is_active = False
        self.capture_thread = None
        
        # 性能统计
        self.frame_count = 0
        self.start_time = time.time()
        
        # 视频捕获
        self.cap = None
        
        # 手势检测相关
        self.last_gesture = None
        self.gesture_cooldown = 0
        self.cooldown_frames = 10  # 增加冷却帧数
        
        # 命令冷却
        self.last_command_time = 0
        
        # 调试信息
        self.debug_info = {}
        
        # 性能优化缓存
        self.last_gesture_result = None
        self.last_gesture_type = None
        
        self.get_logger().info("手势控制节点初始化完成")
        self.get_logger().info(f"速度参数: 线速度={self.linear_speed}, 角速度={self.angular_speed}")
        self.get_logger().info(f"性能设置: 跳帧={self.process_every_n_frames}, 命令冷却={self.command_cooldown}s")
        self.get_logger().info("手势映射: 比4=停止, 比5=放下, 比耶=前进, OK=后退, 食指向左/右=转向")
    
    def _try_open_camera(self, camera_id):
        """尝试多种方法打开摄像头"""
        methods = [
            cv2.CAP_ANY,
            cv2.CAP_V4L2,
            cv2.CAP_V4L,
            cv2.CAP_DSHOW,
            cv2.CAP_MSMF
        ]
        
        for api in methods:
            try:
                self.get_logger().info(f"尝试使用API {api}打开摄像头 {camera_id}...")
                cap = cv2.VideoCapture(camera_id, api)
                
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    cap.set(cv2.CAP_PROP_FPS, self.fps)
                    
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.get_logger().info(f"摄像头 {camera_id} 打开成功，使用API {api}")
                        return cap
                    else:
                        cap.release()
                else:
                    cap.release()
            except Exception as e:
                self.get_logger().warn(f"API {api} 失败: {str(e)}")
        
        return None
    
    def start_capture(self):
        """开始视频捕获和手势识别"""
        if self.is_active:
            self.get_logger().warn("已经在运行中")
            return
        
        # 查找可用摄像头
        available_cams = self._find_available_cameras()
        if not available_cams:
            self.get_logger().error("没有找到可用的摄像头！")
            return
        
        # 尝试打开摄像头
        for cam_id in available_cams:
            self.get_logger().info(f"尝试打开摄像头 {cam_id}...")
            self.cap = self._try_open_camera(cam_id)
            if self.cap is not None:
                self.camera_id = cam_id
                break
        
        if self.cap is None:
            self.get_logger().error("无法打开任何摄像头！")
            return
        
        # 获取实际参数
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f"摄像头实际参数: {actual_width}x{actual_height}@{actual_fps:.1f}fps")
        
        self.is_active = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        self.get_logger().info("开始手势识别...")
    
    def _find_available_cameras(self):
        """查找所有可用的摄像头"""
        available_cams = []
        
        for i in range(10):
            cap_test = cv2.VideoCapture(i, cv2.CAP_ANY)
            if cap_test.isOpened():
                ret, frame = cap_test.read()
                if ret and frame is not None:
                    available_cams.append(i)
                    self.get_logger().info(f"找到可用的摄像头 {i}")
                cap_test.release()
        
        if not available_cams:
            self.get_logger().warn("没有找到摄像头，将尝试虚拟摄像头...")
            available_cams.append(0)
        
        return available_cams
    
    def stop_capture(self):
        """停止视频捕获"""
        self.is_active = False
        # 停止时发布停止速度，确保底盘停下
        try:
            stop_cmd = Twist()
            self.current_cmd = stop_cmd
            self.velocity_pub.publish(stop_cmd)
        except Exception:
            pass
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("手势识别已停止")
    
    def _capture_loop(self):
        """视频捕获和手势识别循环 - 优化版本"""
        frame_time = 1.0 / self.fps if self.fps > 0 else 0.033
        use_virtual_cam = False
        
        # 性能优化：跳过帧处理
        frame_counter = 0
        skip_frames = 0
        
        while self.is_active and rclpy.ok():
            loop_start = time.time()
            
            # 读取帧
            if self.cap and self.cap.isOpened():
                success, frame = self.cap.read()
                if not success:
                    if skip_frames < 5:  # 尝试重试5次
                        skip_frames += 1
                        time.sleep(0.1)
                        continue
                    else:
                        self.get_logger().warn("无法读取视频帧，使用虚拟摄像头...")
                        use_virtual_cam = True
                else:
                    skip_frames = 0
            else:
                use_virtual_cam = True
            
            if use_virtual_cam:
                frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
                cv2.putText(frame, "VIRTUAL CAMERA", (50, self.frame_height//2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, "Waiting for real camera...", (30, self.frame_height//2 + 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 水平翻转
            if self.flip_horizontal:
                frame = cv2.flip(frame, 1)
            
            frame_counter += 1
            should_process_gesture = (frame_counter % self.process_every_n_frames == 0)
            
            current_gesture = None
            
            if should_process_gesture:
                # 手势检测 - 只在需要处理的帧进行
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # 优化：缩小图像以提高处理速度
                scale_factor = self.image_scale
                small_height = int(self.frame_height * scale_factor)
                small_width = int(self.frame_width * scale_factor)
                frame_small = cv2.resize(frame_rgb, (small_width, small_height))
                
                # 设置超时，防止MediaPipe卡死
                try:
                    results = self.hands.process(frame_small)
                except Exception as e:
                    self.get_logger().warn(f"手势处理异常: {str(e)}")
                    results = None
                
                if results and results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # 将关键点坐标缩放回原始尺寸
                        for landmark in hand_landmarks.landmark:
                            landmark.x = landmark.x / scale_factor
                            landmark.y = landmark.y / scale_factor
                            landmark.z = landmark.z / scale_factor
                        
                        self.mp_draw.draw_landmarks(
                            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                        
                        gesture, gesture_info = self._recognize_gesture(hand_landmarks)
                        
                        # 显示手势信息
                        if gesture_info:
                            y_pos = 60
                            for key, value in gesture_info.items():
                                if key != 'desc':
                                    cv2.putText(frame, f"{key}: {value}", (10, y_pos), 
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                    y_pos += 20
                        
                        if gesture:
                            current_gesture = gesture
                            # 手势冷却机制
                            if self.gesture_cooldown <= 0:
                                self._process_gesture(gesture, hand_landmarks)
                                self.gesture_cooldown = self.cooldown_frames
                            
                            self.last_gesture_result = (gesture, gesture_info)
                            self.last_gesture_type = gesture
                else:
                    # 如果没有检测到手，清空缓存
                    self.last_gesture_result = None
                    self.last_gesture_type = None
            else:
                # 使用缓存的上一帧结果
                if self.last_gesture_result:
                    gesture, gesture_info = self.last_gesture_result
                    current_gesture = self.last_gesture_type
                    
                    # 显示缓存的姿势信息
                    if gesture_info:
                        y_pos = 60
                        for key, value in gesture_info.items():
                            if key != 'desc':
                                cv2.putText(frame, f"{key}: {value}", (10, y_pos), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 0), 1)  # 使用不同颜色表示缓存
                                y_pos += 20
            
            # 减少冷却计数
            if self.gesture_cooldown > 0:
                self.gesture_cooldown -= 1
            
            # 显示FPS
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示处理状态
            process_status = "PROCESSING" if should_process_gesture else "CACHED"
            cv2.putText(frame, f"Status: {process_status}", (self.frame_width - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 显示指令帮助 - 修改：更新手势说明
            cv2.putText(frame, "Index Left: Turn Left", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "Index Right: Turn Right", (10, 140), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "Victory (2): Forward", (10, 160), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "OK Sign (3): Backward", (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "Four Fingers (4): Stop", (10, 200), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "Five Fingers (5): Release", (10, 220), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "Fist: Grab", (10, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Frame Skip: {self.process_every_n_frames-1}", (10, 260), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Cooldown: {self.command_cooldown}s", (10, 280), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Speed: L={self.linear_speed}, A={self.angular_speed}", (10, 300), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "Press 'q' to quit, '+/-' adjust speed", (10, self.frame_height - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(frame, "Press 'r' to reset model", (10, self.frame_height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 显示当前手势
            if current_gesture:
                gesture_text = f"Gesture: {current_gesture}"
                cv2.putText(frame, gesture_text, (self.frame_width - 200, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)
            
            # 显示视频
            if self.show_video:
                cv2.imshow('Gesture Control - Optimized', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.is_active = False
                    break
                elif key == ord('+') or key == ord('='):
                    # 增加处理频率
                    self.process_every_n_frames = max(1, self.process_every_n_frames - 1)
                    self.get_logger().info(f"增加处理频率: 每{self.process_every_n_frames}帧处理一次")
                elif key == ord('-') or key == ord('_'):
                    # 减少处理频率
                    self.process_every_n_frames = min(5, self.process_every_n_frames + 1)
                    self.get_logger().info(f"减少处理频率: 每{self.process_every_n_frames}帧处理一次")
                elif key == ord('r'):
                    # 重置手势识别
                    self.hands = self.mp_hands.Hands(
                        static_image_mode=False,
                        max_num_hands=1,
                        min_detection_confidence=0.6,
                        min_tracking_confidence=0.5,
                        model_complexity=0
                    )
                    self.get_logger().info("重置手势识别模型")
            
            # 控制帧率
            loop_time = time.time() - loop_start
            sleep_time = max(0, frame_time - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if self.cap:
            self.cap.release()
    
    def _recognize_gesture(self, landmarks):
        """识别手势：重点识别比耶、比4、比5和OK手势"""
        landmarks_list = []
        for lm in landmarks.landmark:
            landmarks_list.append([lm.x, lm.y, lm.z])
        
        # 关键点
        wrist = landmarks_list[0]
        thumb_tip = landmarks_list[4]
        index_tip = landmarks_list[8]
        middle_tip = landmarks_list[12]
        ring_tip = landmarks_list[16]
        pinky_tip = landmarks_list[20]
        
        # 手指关节
        index_mcp = landmarks_list[5]
        index_pip = landmarks_list[6]
        index_dip = landmarks_list[7]
        
        middle_mcp = landmarks_list[9]
        middle_pip = landmarks_list[10]
        middle_dip = landmarks_list[11]
        
        ring_mcp = landmarks_list[13]
        ring_pip = landmarks_list[14]
        ring_dip = landmarks_list[15]
        
        pinky_mcp = landmarks_list[17]
        pinky_pip = landmarks_list[18]
        pinky_dip = landmarks_list[19]
        
        # 计算距离函数
        def distance(p1, p2):
            return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        
        # 手指伸直判断
        index_extended = (index_tip[1] < index_dip[1] < index_pip[1] < index_mcp[1])
        middle_extended = (middle_tip[1] < middle_dip[1] < middle_pip[1] < middle_mcp[1])
        ring_extended = (ring_tip[1] < ring_dip[1] < ring_pip[1] < ring_mcp[1])
        pinky_extended = (pinky_tip[1] < pinky_dip[1] < pinky_pip[1] < pinky_mcp[1])
        
        # 拇指伸直判断
        thumb_ip = landmarks_list[3]
        thumb_mcp = landmarks_list[2]
        thumb_extended = (thumb_tip[0] < thumb_ip[0] < thumb_mcp[0])
        
        # 手掌中心
        palm_center = np.mean([index_mcp, middle_mcp, ring_mcp, pinky_mcp], axis=0)
        
        # 1. 比5手势（所有手指伸直）
        if all([index_extended, middle_extended, ring_extended, pinky_extended]):
            # 检查拇指是否也伸直（真正的5个手指都伸直）
            if thumb_extended:
                return 'five_fingers', {'type': 'five_fingers', 'desc': 'Release/Drop', 'confidence': 'high'}
        
        # 2. 比4手势（除了拇指外的四个手指伸直）
        if all([index_extended, middle_extended, ring_extended, pinky_extended]):
            # 拇指弯曲（非伸直）
            if not thumb_extended:
                return 'four_fingers', {'type': 'four_fingers', 'desc': 'Stop', 'confidence': 'high'}
        
        # 3. 比耶手势（食指和中指伸直，其他手指弯曲）
        if index_extended and middle_extended and not any([ring_extended, pinky_extended]):
            return 'victory', {'type': 'victory', 'desc': 'Forward', 'confidence': 'high'}
        
        # 4. OK手势（食指和拇指形成圆圈，其他手指伸直或微曲）
        thumb_index_distance = distance(thumb_tip, index_tip)
        
        if thumb_index_distance < 0.05:
            other_fingers_ok = (middle_extended or distance(middle_tip, palm_center) > 0.2) and \
                               (ring_extended or distance(ring_tip, palm_center) > 0.2) and \
                               (pinky_extended or distance(pinky_tip, palm_center) > 0.2)
            
            if other_fingers_ok:
                return 'ok_sign', {'type': 'ok_sign', 'desc': 'Backward', 'confidence': 'medium'}
        
        # 5. 单指指向（只有食指伸直，用于左右转向）
        if index_extended and not any([middle_extended, ring_extended, pinky_extended]):
            # 计算食指方向
            vector_x = index_tip[0] - index_mcp[0]
            vector_y = index_tip[1] - index_mcp[1]
            
            angle_rad = np.arctan2(vector_y, vector_x)
            angle_deg = np.degrees(angle_rad)
            
            # 判断左右方向
            if self.flip_horizontal:
                if -45 <= angle_deg <= 45:
                    direction = 'finger_right'
                elif angle_deg > 135 or angle_deg <= -135:
                    direction = 'finger_left'
                else:
                    direction = None
            else:
                if -45 <= angle_deg <= 45:
                    direction = 'finger_right'
                elif angle_deg > 135 or angle_deg <= -135:
                    direction = 'finger_left'
                else:
                    direction = None
            
            if direction:
                return direction, {
                    'type': direction, 
                    'angle': f"{angle_deg:.1f}",
                    'desc': self.gesture_map.get(direction, "Unknown"),
                    'confidence': 'medium'
                }
        
        # 6. 握拳（所有手指弯曲）- 放在最后，确保不会覆盖其他手势
        tips = [index_tip, middle_tip, ring_tip, pinky_tip]
        avg_distance = np.mean([distance(tip, palm_center) for tip in tips])
        
        # 检查食指是否明显伸直（用于区分握拳和指向）
        index_extension_ratio = distance(index_tip, palm_center) / distance(index_mcp, palm_center)
        
        # 握拳条件：平均距离小 + 所有手指都不伸直 + 食指没有明显伸长
        if (avg_distance < 0.15 and 
            not any([index_extended, middle_extended, ring_extended, pinky_extended]) and
            index_extension_ratio < 1.3):  # 食指伸长比例小于1.3倍
            return 'fist', {'type': 'fist', 'desc': 'Grab', 'confidence': 'high'}
        
        return None, None
    
    def _process_gesture(self, gesture, landmarks):
        """处理识别到的手势 - 增加冷却时间"""
        current_time = time.time()
        
        # 检查冷却时间
        if current_time - self.last_command_time < self.command_cooldown:
            return
        
        if gesture in self.gesture_map:
            command = self.gesture_map[gesture]
            
            # 更新最后命令时间
            self.last_command_time = current_time
            
            # 发布手势命令
            msg = String()
            msg.data = command
            self.gesture_pub.publish(msg)

            # 根据手势生成并设置当前速度命令
            vel_msg = Twist()

            if command == 'forward':      # 比耶手势
                vel_msg.linear.x = self.linear_speed
                vel_msg.angular.z = 0.0
            elif command == 'backward':   # OK手势
                vel_msg.linear.x = -self.linear_speed * 0.75  # 后退速度略小
                vel_msg.angular.z = 0.0
            elif command == 'left':       # 食指向左
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = self.angular_speed
            elif command == 'right':      # 食指向右
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -self.angular_speed
            elif command == 'stop':       # 比4手势
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0

            # 对于移动命令，设置当前命令
            if command in ['forward', 'backward', 'left', 'right', 'stop']:
                self.current_cmd = vel_msg
                # 立即发布一次
                try:
                    self.velocity_pub.publish(self.current_cmd)
                except Exception:
                    pass

            self.get_logger().info(f"手势: {gesture} -> 命令: {command}")
            self.last_gesture = gesture
    
    def destroy_node(self):
        """清理资源"""
        self.stop_capture()
        # 停用并清理定时器
        try:
            if hasattr(self, 'cmd_timer') and self.cmd_timer is not None:
                self.cmd_timer.cancel()
        except Exception:
            pass
        super().destroy_node()

    def _publish_velocity(self):
        """定时发布当前速度命令"""
        try:
            if rclpy.ok():
                self.velocity_pub.publish(self.current_cmd)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    
    try:
        node.start_capture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"发生错误: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

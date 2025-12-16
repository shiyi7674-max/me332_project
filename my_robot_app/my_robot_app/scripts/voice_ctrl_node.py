#!/usr/bin/env python3
"""
离线语音控制节点
使用Vosk进行本地语音识别
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import json
import os
import pyaudio
import numpy as np
from vosk import Model, KaldiRecognizer
import re

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # 参数声明
        self.declare_parameter('model_path', '~/vosk-model-small-cn-0.22')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('energy_threshold', 0.01)
        self.declare_parameter('min_silence_duration', 1.0)
        
        # 获取参数
        self.sample_rate = self.get_parameter('sample_rate').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.min_silence_duration = self.get_parameter('min_silence_duration').value
        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        
        # 发布者
        self.command_pub = self.create_publisher(String, '/voice_control', 10)
        self.speech_pub = self.create_publisher(String, '/speech_text', 10)
        
        # 初始化Vosk模型
        self.get_logger().info("离线语音控制节点初始化...")
        
        try:
            self.get_logger().info(f"加载Vosk模型: {model_path}")
            if not os.path.exists(model_path):
                self.get_logger().error(f"模型文件不存在: {model_path}")
                self.get_logger().info("请下载中文模型: wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip")
                raise FileNotFoundError(f"Vosk模型未找到: {model_path}")
            
            self.model = Model(model_path)
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            self.get_logger().info("✅ Vosk模型加载成功")
        except Exception as e:
            self.get_logger().error(f"无法加载Vosk模型: {e}")
            raise
        
        # 音频系统初始化
        self.audio = pyaudio.PyAudio()
        self.stream = None
        
        # 控制标志
        self.is_active = False
        self.recognition_thread = None
        self.silence_threshold_frames = int(self.min_silence_duration * self.sample_rate / 4000)
        
        # 统计信息
        self.recognition_count = 0
        self.last_command_time = 0
        self.command_cooldown = 1.0
        
        # 命令映射表（与项目其他部分保持一致）
        self.command_map = {
            # 移动命令
            '前进': 'move_forward',
            '往前走': 'move_forward',
            '向前': 'move_forward',
            '后退': 'move_backward',
            '向后': 'move_backward',
            '左转': 'turn_left',
            '向左转': 'turn_left',
            '向左': 'turn_left',
            '右转': 'turn_right',
            '向右转': 'turn_right',
            '向右': 'turn_right',
            
            # 停止命令
            '停止': 'stop',
            '停下': 'stop',
            '停': 'stop',
            '暂停': 'stop',
            
            # 抓取命令
            '抓取': 'pick_object',
            '抓': 'pick_object',
            '拿': 'pick_object',
            '取物': 'pick_object',
            
            # 放置命令
            '放下': 'place_object',
            '放置': 'place_object',
            '放': 'place_object',
            '释放': 'place_object',
            
            # 导航命令
            '回家': 'go_home',
            '返回': 'go_home',
            '回原点': 'go_home',
            '去起点': 'go_home',
        }
        
        # 查找音频设备
        self._find_audio_device()
        
        self.get_logger().info("✅ 离线语音识别准备就绪")
    
    def _find_audio_device(self):
        """查找合适的音频设备"""
        device_count = self.audio.get_device_count()
        
        for i in range(device_count):
            try:
                info = self.audio.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    self.device_index = i
                    self.get_logger().info(f"✅ 使用音频设备: {info['name']}")
                    return
            except:
                continue
        
        self.get_logger().error("❌ 未找到可用的音频输入设备")
        raise Exception("No audio input device found")
    
    def _open_audio_stream(self):
        """打开音频流"""
        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=int(self.sample_rate * 0.1),
            )
            
            self.get_logger().info(f"✅ 音频流打开成功")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ 无法打开音频流: {e}")
            return False
    
    def _calculate_energy(self, audio_data):
        """计算音频能量"""
        if len(audio_data) == 0:
            return 0.0
        
        try:
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            if len(audio_array) == 0:
                return 0.0
            
            squared = audio_array.astype(np.float32) ** 2
            mean_squared = np.mean(squared)
            
            if mean_squared <= 0:
                return 0.0
            
            energy = np.sqrt(mean_squared) / 32768.0
            
            return float(energy)
                
        except Exception as e:
            self.get_logger().debug(f"能量计算错误: {e}")
            return 0.0
    
    def start_listening(self):
        """开始监听"""
        if self.is_active:
            self.get_logger().warning("已经在监听中")
            return
        
        # 打开音频流
        if not self._open_audio_stream():
            self.get_logger().error("无法打开音频流")
            return
        
        self.is_active = True
        self.recognition_thread = threading.Thread(target=self._listening_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()
        
        self.get_logger().info("🎤 开始离线语音监听...")
    
    def _listening_loop(self):
        """监听循环"""
        try:
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            
            audio_buffer = bytes()
            is_speaking = False
            silence_counter = 0
            chunk_size = 4000
            
            self.get_logger().info("音频处理循环启动...")
            
            while self.is_active and self.stream and rclpy.ok():
                try:
                    # 读取音频数据
                    data = self.stream.read(chunk_size, exception_on_overflow=False)
                    
                    # VAD检测
                    energy = self._calculate_energy(data)
                    has_speech = energy > self.energy_threshold
                    
                    if has_speech:
                        if not is_speaking:
                            self.get_logger().info("🎤 检测到语音，开始录音...")
                            is_speaking = True
                            silence_counter = 0
                        
                        audio_buffer += data
                        silence_counter = 0
                            
                    elif is_speaking:
                        silence_counter += 1
                        
                        if silence_counter >= self.silence_threshold_frames:
                            # 处理录音
                            self.get_logger().info("检测到静音，开始识别...")
                            self._process_audio_buffer(audio_buffer)
                            
                            # 重置状态
                            audio_buffer = bytes()
                            is_speaking = False
                            silence_counter = 0
                            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
                            
                    else:
                        # 没有语音时，短暂休眠以降低CPU使用率
                        time.sleep(0.01)
                    
                except Exception as e:
                    self.get_logger().error(f"音频处理错误: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f"监听循环错误: {e}")
            self.is_active = False
        finally:
            self._close_audio_stream()
    
    def _process_audio_buffer(self, audio_buffer):
        """处理音频缓冲区"""
        if not audio_buffer or len(audio_buffer) < 8000:
            self.get_logger().warning("音频数据太短，忽略")
            return
        
        try:
            # 识别音频
            if self.recognizer.AcceptWaveform(audio_buffer):
                result = json.loads(self.recognizer.Result())
                text = result.get('text', '').strip()
            else:
                result = json.loads(self.recognizer.PartialResult())
                text = result.get('partial', '').strip()
            
            if text:
                self.recognition_count += 1
                self.get_logger().info(f"✅ 识别结果: {text}")
                
                # 发布原始语音文本
                speech_msg = String()
                speech_msg.data = text
                self.speech_pub.publish(speech_msg)
                
                # 处理命令
                self._process_command(text)
                
        except Exception as e:
            self.get_logger().error(f"音频处理错误: {e}")
    
    def _process_command(self, text):
        """处理识别到的命令"""
        # 清理文本
        clean_text = re.sub(r'[。，！？、,\.!\?]', '', text)
        
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            self.get_logger().warning("命令冷却中...")
            return False
        
        self.get_logger().info(f"处理文本: {clean_text}")
        
        # 精确匹配
        for keyword, command in self.command_map.items():
            clean_keyword = re.sub(r'[。，！？、,\.!\?]', '', keyword)
            
            if clean_keyword == clean_text or clean_keyword in clean_text:
                self._publish_command(command, clean_keyword)
                self.last_command_time = current_time
                return True
        
        self.get_logger().warning(f"未识别的命令: {clean_text}")
        return False
    
    def _publish_command(self, command, keyword):
        """发布命令"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"🚀 执行命令: {command}")
    
    def _close_audio_stream(self):
        """关闭音频流"""
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
            finally:
                self.stream = None
    
    def stop_listening(self):
        """停止监听"""
        if self.is_active:
            self.get_logger().info("停止语音监听...")
            self.is_active = False
            
            self._close_audio_stream()
            
            if self.recognition_thread:
                self.recognition_thread.join(timeout=2.0)
    
    def destroy_node(self):
        """清理资源"""
        self.stop_listening()
        if self.audio:
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    
    try:
        node.get_logger().info("\n🎤 离线语音控制节点启动")
        node.get_logger().info("💡 常用命令: '前进', '后退', '左转', '右转', '停止'")
        node.get_logger().info("💡 检测到静音后自动识别")
        
        # 启动监听
        node.start_listening()
        
        # 主循环
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("\n👋 用户中断")
    except Exception as e:
        node.get_logger().error(f"❌ 程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

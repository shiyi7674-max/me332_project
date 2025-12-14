#!/usr/bin/env python3
"""
离线语音控制节点 - 增强版修复
使用Vosk进行本地语音识别，支持PulseAudio设备
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

class EnhancedVoiceControlNode(Node):
    def __init__(self):
        super().__init__('enhanced_voice_control')
        
        # 参数声明
        self.declare_parameter('device_type', 'auto')  # auto, pulse, alsa, default
        self.declare_parameter('device_index', -1)  # -1表示自动选择
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('energy_threshold', 0.01)
        self.declare_parameter('model_path', '~/vosk-model-small-cn-0.22')
        self.declare_parameter('vad_threshold', 0.5)  # 语音活动检测阈值
        self.declare_parameter('min_silence_duration', 1.0)  # 最小静音时间（秒）
        
        # 获取参数
        self.device_type = self.get_parameter('device_type').value
        self.device_index = self.get_parameter('device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.min_silence_duration = self.get_parameter('min_silence_duration').value
        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        
        # 发布者
        self.command_pub = self.create_publisher(String, '/voice_control', 10)
        self.speech_pub = self.create_publisher(String, '/speech_text', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)
        
        # 初始化Vosk模型
        self.get_logger().info("=" * 60)
        self.get_logger().info("增强版离线语音控制节点")
        self.get_logger().info("=" * 60)
        
        try:
            self.get_logger().info(f"加载Vosk模型: {model_path}")
            self.model = Model(model_path)
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            self.get_logger().info("✅ Vosk模型加载成功")
        except Exception as e:
            self.get_logger().error(f"❌ 无法加载Vosk模型: {e}")
            self.get_logger().info("请下载中文模型:")
            self.get_logger().info("wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip")
            self.get_logger().info("unzip vosk-model-small-cn-0.22.zip")
            self.get_logger().info("或使用: pip3 install vosk")
            raise
        
        # 音频系统初始化
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.selected_device_info = None
        
        # 语音检测参数
        self.is_active = False
        self.recognition_thread = None
        self.silence_frames = 0
        self.silence_threshold_frames = int(self.min_silence_duration * self.sample_rate / 4000)
        
        # 统计信息
        self.recognition_count = 0
        self.success_count = 0
        self.last_command_time = 0
        self.command_cooldown = 1.0  # 命令冷却时间（秒）
        
        # 增强的命令映射表
        self.command_map = {
            # 移动命令
            '前进': 'move_forward',
            '往前走': 'move_forward',
            '向前': 'move_forward',
            '前进。': 'move_forward',
            
            '后退': 'move_backward',
            '向后': 'move_backward',
            '后退。': 'move_backward',
            '倒退': 'move_backward',
            
            '左转': 'turn_left',
            '向左转': 'turn_left',
            '左转。': 'turn_left',
            '向左': 'turn_left',
            '左边': 'turn_left',
            
            '右转': 'turn_right',
            '向右转': 'turn_right',
            '右转。': 'turn_right',
            '向右': 'turn_right',
            '右边': 'turn_right',
            
            # 停止命令
            '停止': 'stop',
            '停下': 'stop',
            '停': 'stop',
            '停止。': 'stop',
            '暂停': 'stop',
            
            # 抓取命令
            '抓取': 'pick_object',
            '抓': 'pick_object',
            '抓取。': 'pick_object',
            '拿': 'pick_object',
            '取物': 'pick_object',
            
            # 放置命令
            '放下': 'place_object',
            '放置': 'place_object',
            '放下。': 'place_object',
            '放': 'place_object',
            '释放': 'place_object',
            
            # 导航命令
            '回家': 'go_home',
            '返回': 'go_home',
            '回家。': 'go_home',
            '回原点': 'go_home',
            '去起点': 'go_home',
            
            # 速度控制
            '加速': 'speed_up',
            '快点': 'speed_up',
            '减速': 'speed_down',
            '慢点': 'speed_down',
            
            # 状态查询
            '状态': 'get_status',
            '怎么样了': 'get_status',
            '情况': 'get_status',
        }
        
        # 查找音频设备
        self._find_audio_device()
        
        # 服务或定时器
        self.calibration_timer = self.create_timer(30.0, self._auto_calibrate)  # 每30秒自动校准
        self.status_timer = self.create_timer(5.0, self._publish_status)  # 每5秒发布状态
        
        # 状态变量
        self.current_status = "等待语音输入"
        self.last_energy = 0.0
        
        self.get_logger().info("✅ 增强版离线语音识别准备就绪")
        self.get_logger().info("💡 支持PulseAudio和ALSA设备")
    
    def _find_audio_device(self):
        """查找合适的音频设备"""
        self.get_logger().info("扫描音频设备...")
        
        device_info = None
        device_count = self.audio.get_device_count()
        
        # 先尝试查找指定类型的设备
        if self.device_type in ['pulse', 'alsa']:
            for i in range(device_count):
                try:
                    info = self.audio.get_device_info_by_index(i)
                    if info['maxInputChannels'] > 0:
                        device_name = info['name'].lower()
                        if (self.device_type == 'pulse' and ('pulse' in device_name or 'default' in device_name)) or \
                           (self.device_type == 'alsa' and 'alsa' in device_name):
                            device_info = (i, info)
                            self.get_logger().info(f"✅ 找到{self.device_type.upper()}设备: {info['name']}")
                            break
                except:
                    continue
        
        # 如果没找到指定类型，使用默认输入设备
        if not device_info:
            try:
                default_info = self.audio.get_default_input_device_info()
                device_info = (default_info['index'], default_info)
                self.get_logger().info(f"✅ 使用默认设备: {default_info['name']}")
            except:
                # 手动查找第一个可用的输入设备
                for i in range(device_count):
                    try:
                        info = self.audio.get_device_info_by_index(i)
                        if info['maxInputChannels'] > 0:
                            device_info = (i, info)
                            self.get_logger().info(f"✅ 使用设备: {info['name']}")
                            break
                    except:
                        continue
        
        if not device_info:
            self.get_logger().error("❌ 未找到可用的音频输入设备")
            raise Exception("No audio input device found")
        
        self.selected_device_info = device_info
        self.device_index = device_info[0]
        
        # 显示设备信息
        info = device_info[1]
        self.get_logger().info(f"设备名称: {info['name']}")
        self.get_logger().info(f"采样率范围: {info['defaultSampleRate']}")
        self.get_logger().info(f"最大输入通道: {info['maxInputChannels']}")
        
        return True
    
    def _open_audio_stream(self):
        """打开音频流"""
        try:
            # 使用设备支持的最佳采样率
            device_rate = int(self.selected_device_info[1]['defaultSampleRate'])
            if self.sample_rate > device_rate:
                self.get_logger().warning(f"设备不支持 {self.sample_rate}Hz，使用 {device_rate}Hz")
                self.sample_rate = device_rate
            
            # 确保采样率是支持的
            supported_rates = [8000, 16000, 22050, 44100, 48000]
            closest_rate = min(supported_rates, key=lambda x: abs(x - self.sample_rate))
            if abs(closest_rate - self.sample_rate) > 1000:
                self.get_logger().warning(f"采样率 {self.sample_rate}Hz 可能不支持，使用 {closest_rate}Hz")
                self.sample_rate = closest_rate
            
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=int(self.sample_rate * 0.1),  # 100ms缓冲区
                stream_callback=None  # 不使用回调，避免复杂
            )
            
            self.get_logger().info(f"✅ 音频流打开成功 (采样率: {self.sample_rate}Hz)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ 无法打开音频流: {e}")
            return False
    
    def _calculate_energy(self, audio_data):
        """计算音频能量 - 修复版本"""
        if len(audio_data) == 0:
            return 0.0
        
        try:
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            if len(audio_array) == 0:
                return 0.0
            
            # 避免除零和无效值
            with np.errstate(invalid='ignore', divide='ignore'):
                squared = audio_array.astype(np.float32) ** 2
                mean_squared = np.mean(squared)
                
                # 如果mean_squared是负数或接近零，返回0
                if mean_squared <= 0:
                    return 0.0
                
                energy = np.sqrt(mean_squared) / 32768.0
                
                # 处理可能的NaN值
                if np.isnan(energy) or np.isinf(energy):
                    return 0.0
                
                return float(energy)
                
        except Exception as e:
            self.get_logger().debug(f"能量计算错误: {e}")
            return 0.0
    
    def _vad_decision(self, audio_data):
        """语音活动检测"""
        if len(audio_data) == 0:
            return False
        
        energy = self._calculate_energy(audio_data)
        self.last_energy = energy
        
        # 简单的VAD决策
        is_speech = energy > self.energy_threshold
        
        return is_speech
    
    def _auto_calibrate(self):
        """自动校准环境噪音"""
        if not self.is_active or not self.stream:
            return
        
        self.get_logger().info("正在进行环境噪音校准...")
        
        # 收集短暂的噪音样本
        noise_levels = []
        calibration_frames = 10  # 减少校准样本数
        
        for _ in range(calibration_frames):
            try:
                data = self.stream.read(1024, exception_on_overflow=False)
                energy = self._calculate_energy(data)
                if energy > 0:
                    noise_levels.append(energy)
            except Exception as e:
                self.get_logger().debug(f"校准读取错误: {e}")
                continue
        
        if noise_levels:
            avg_noise = np.mean(noise_levels)
            std_noise = np.std(noise_levels)
            
            # 自适应阈值 - 更保守
            new_threshold = max(0.005, min(0.05, avg_noise + std_noise * 3))
            
            # 只有当变化较大时才更新
            if abs(new_threshold - self.energy_threshold) > 0.001:
                self.energy_threshold = new_threshold
                self.get_logger().info(f"环境噪音: {avg_noise:.4f} ± {std_noise:.4f}")
                self.get_logger().info(f"更新阈值: {self.energy_threshold:.4f}")
        
    def _publish_status(self):
        """发布状态信息"""
        if not self.is_active:
            return
        
        status_msg = String()
        status_data = {
            'status': self.current_status,
            'energy': self.last_energy,
            'threshold': self.energy_threshold,
            'recognitions': self.recognition_count,
            'successes': self.success_count
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
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
        self.current_status = "正在监听"
        self.recognition_thread = threading.Thread(target=self._listening_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()
        
        self.get_logger().info("🎤 开始离线语音监听...")
        self.get_logger().info("💡 请说话，检测到静音后自动识别")
        self.get_logger().info(f"静音检测: {self.min_silence_duration}秒")
        self.get_logger().info(f"能量阈值: {self.energy_threshold:.4f}")
    
    def _listening_loop(self):
        """监听循环"""
        try:
            # 重置识别器
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
            
            audio_buffer = bytes()
            is_speaking = False
            silence_counter = 0
            chunk_size = 4000  # 250ms的音频数据
            
            self.get_logger().info("音频处理循环启动...")
            
            while self.is_active and self.stream and rclpy.ok():
                try:
                    # 读取音频数据
                    data = self.stream.read(chunk_size, exception_on_overflow=False)
                    
                    # VAD检测
                    has_speech = self._vad_decision(data)
                    
                    if has_speech:
                        if not is_speaking:
                            self.get_logger().info("🎤 检测到语音，开始录音...")
                            self.current_status = "正在录音"
                            is_speaking = True
                            silence_counter = 0
                        
                        audio_buffer += data
                        silence_counter = 0
                        
                        # 限制缓冲区大小（避免内存问题）
                        if len(audio_buffer) > self.sample_rate * 5:  # 最多5秒
                            self.get_logger().warning("音频缓冲区过大，清空")
                            audio_buffer = bytes()
                            
                    elif is_speaking:
                        silence_counter += 1
                        
                        if silence_counter >= self.silence_threshold_frames:
                            # 处理录音
                            self.current_status = "正在识别"
                            self.get_logger().info("检测到静音，开始识别...")
                            self._process_audio_buffer(audio_buffer)
                            
                            # 重置状态
                            audio_buffer = bytes()
                            is_speaking = False
                            silence_counter = 0
                            self.current_status = "正在监听"
                            
                            # 重置识别器
                            self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
                            
                            self.get_logger().info("准备下一句话...")
                        else:
                            # 短暂静音，继续累积
                            audio_buffer += data
                    
                except Exception as e:
                    self.get_logger().error(f"音频处理错误: {e}")
                    self.current_status = f"错误: {str(e)}"
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f"监听循环错误: {e}")
            self.current_status = f"循环错误: {str(e)}"
            self.is_active = False
        finally:
            self._close_audio_stream()
            self.current_status = "已停止"
    
    def _process_audio_buffer(self, audio_buffer):
        """处理音频缓冲区"""
        if not audio_buffer or len(audio_buffer) < 8000:  # 至少0.5秒
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
                if self._process_command(text):
                    self.success_count += 1
                    self.current_status = "命令执行成功"
                else:
                    self.current_status = "命令未识别"
                
                # 显示统计
                if self.recognition_count > 0:
                    rate = (self.success_count / self.recognition_count) * 100
                    self.get_logger().info(f"📊 成功率: {rate:.1f}% ({self.success_count}/{self.recognition_count})")
            else:
                self.get_logger().warning("未识别到有效文本")
                self.current_status = "识别失败"
                
        except Exception as e:
            self.get_logger().error(f"音频处理错误: {e}")
            self.current_status = "处理错误"
    
    def _process_command(self, text):
        """处理识别到的命令"""
        # 清理文本
        import re
        clean_text = re.sub(r'[。，！？、,\.!\?]', '', text)
        
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            self.get_logger().warning("命令冷却中...")
            return False
        
        self.get_logger().info(f"处理文本: {clean_text}")
        
        # 精确匹配优先
        matched = False
        for keyword, command in self.command_map.items():
            clean_keyword = re.sub(r'[。，！？、,\.!\?]', '', keyword)
            
            # 精确匹配或包含匹配
            if clean_keyword == clean_text or clean_keyword in clean_text:
                self._publish_command(command, clean_keyword)
                matched = True
                self.last_command_time = current_time
                break
        
        # 如果精确匹配失败，尝试模糊匹配
        if not matched and len(clean_text) >= 2:
            self._fuzzy_match(clean_text)
        
        return matched
    
    def _publish_command(self, command, keyword):
        """发布命令"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"🚀 执行命令: {command} (关键词: {keyword})")
        
        # 简单的音效反馈
        print("\a", end='', flush=True)  # 终端响铃
    
    def _fuzzy_match(self, text):
        """模糊匹配命令"""
        # 简单的编辑距离匹配
        import difflib
        
        clean_commands = []
        for cmd in self.command_map.keys():
            clean_cmd = re.sub(r'[。，！？、,\.!\?]', '', cmd)
            if clean_cmd:
                clean_commands.append(clean_cmd)
        
        # 使用difflib查找最相似的命令
        if clean_commands:
            matches = difflib.get_close_matches(text, clean_commands, n=1, cutoff=0.5)
            
            if matches:
                matched_cmd = matches[0]
                # 找到原始命令
                for orig_cmd in self.command_map.keys():
                    clean_orig = re.sub(r'[。，！？、,\.!\?]', '', orig_cmd)
                    if clean_orig == matched_cmd:
                        command = self.command_map[orig_cmd]
                        self.get_logger().warning(f"近似匹配: '{text}' → '{orig_cmd}'")
                        self._publish_command(command, orig_cmd)
                        self.last_command_time = time.time()
                        return True
        
        self.get_logger().warning(f"未识别的命令: {text}")
        
        # 显示可用命令（前5个）
        self.get_logger().info("可用命令示例:")
        count = 0
        for cmd in self.command_map.keys():
            clean_cmd = re.sub(r'[。，！？、,\.!\?]', '', cmd)
            if count < 5:
                self.get_logger().info(f"  - {clean_cmd}")
                count += 1
        
        return False
    
    def _close_audio_stream(self):
        """关闭音频流"""
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
                self.get_logger().info("音频流已关闭")
            except Exception as e:
                self.get_logger().debug(f"关闭音频流错误: {e}")
            finally:
                self.stream = None
    
    def stop_listening(self):
        """停止监听"""
        if self.is_active:
            self.get_logger().info("停止语音监听...")
            self.is_active = False
            self.current_status = "正在停止"
            
            self._close_audio_stream()
            
            if self.recognition_thread:
                self.recognition_thread.join(timeout=2.0)
            
            # 显示最终统计
            self._show_statistics()
    
    def _show_statistics(self):
        """显示统计信息"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("语音控制统计:")
        self.get_logger().info(f"识别次数: {self.recognition_count}")
        self.get_logger().info(f"成功次数: {self.success_count}")
        if self.recognition_count > 0:
            rate = (self.success_count / self.recognition_count) * 100
            self.get_logger().info(f"成功率: {rate:.1f}%")
        self.get_logger().info("=" * 60)
    
    def destroy_node(self):
        """清理资源"""
        self.stop_listening()
        if self.audio:
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedVoiceControlNode()
    
    try:
        # 显示帮助信息
        node.get_logger().info("\n使用说明:")
        node.get_logger().info("1. 直接对麦克风说话即可控制")
        node.get_logger().info("2. 常用命令: '前进', '后退', '左转', '右转', '停止'")
        node.get_logger().info("3. 检测到静音后自动识别")
        
        # 等待初始化
        time.sleep(2)
        
        # 启动监听
        node.start_listening()
        
        # 主循环
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("\n👋 用户中断")
    except Exception as e:
        node.get_logger().error(f"❌ 程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

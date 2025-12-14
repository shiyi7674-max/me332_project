"""Scripts package."""
def __init__(self):
    super().__init__('gesture_control_node')
    
    # 添加性能优化参数
    self.declare_parameter('camera_id', 0)
    self.declare_parameter('control_sensitivity', 0.5)
    self.declare_parameter('show_video', True)
    self.declare_parameter('frame_width', 640)
    self.declare_parameter('frame_height', 480)
    self.declare_parameter('fps', 30)
    self.declare_parameter('flip_horizontal', True)
    self.declare_parameter('process_every_n_frames', 2)  # 新增：跳帧参数
    self.declare_parameter('image_scale', 0.5)  # 新增：图像缩放参数
    
    # 获取新增参数
    self.process_every_n_frames = self.get_parameter('process_every_n_frames').value
    self.image_scale = self.get_parameter('image_scale').value
    
    # ... 其他初始化代码 ...

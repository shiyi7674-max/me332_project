#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterDescriptor
from rcl_interfaces.srv import SetParameters, ListParameters, GetParameters
import yaml
import os

class ParamTuner(Node):
    def __init__(self):
        super().__init__('param_tuner')
        
    def load_params_from_yaml(self, yaml_file):
        """从YAML文件加载参数"""
        with open(yaml_file, 'r') as f:
            params = yaml.safe_load(f)
        return params
    
    def save_params_to_yaml(self, params, yaml_file):
        """保存参数到YAML文件"""
        with open(yaml_file, 'w') as f:
            yaml.dump(params, f, default_flow_style=False)
    
    def get_node_params(self, node_name):
        """获取节点的所有参数"""
        client = self.create_client(ListParameters, f'{node_name}/list_parameters')
        client.wait_for_service()
        request = ListParameters.Request()
        response = client.call(request)
        return response.result.names
    
    def tune_controller_params(self):
        """调优控制器参数"""
        params_file = 'config/nav2_params/controller_params.yaml'
        params = self.load_params_from_yaml(params_file)
        
        # 调整速度参数
        params['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] = 0.3
        params['controller_server']['ros__parameters']['FollowPath']['max_vel_theta'] = 0.5
        
        # 保存调整后的参数
        self.save_params_to_yaml(params, params_file)
        self.get_logger().info('控制器参数已更新')

# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

import usb

import numpy as np
import cv2
import time

ID_VENDOR_REALSENSE = 0x8086 # Intel
MANUFACTURER_REALSENSE = "Intel(R) RealSense(TM)"
PRODUCT_REALSENSE = "Intel(R) RealSense(TM)"

def reset_realsense_devices():
    usb_devices = usb.core.find(find_all=True)

    def is_realsense_device(dev):
        is_same_idVendor = dev.idVendor == ID_VENDOR_REALSENSE
        if not is_same_idVendor:
            return False

        is_same_manufacturer = MANUFACTURER_REALSENSE in dev.manufacturer
        is_same_product = PRODUCT_REALSENSE in dev.product

        return is_same_manufacturer and is_same_product

    realsense_devices = filter(is_realsense_device, usb_devices)

    for dev in realsense_devices:
        dev.reset()

# reset usb connection of realsense devices
def reset_realsense_devices_with_retry(n_retry=5):
    i_retry = 0
    while True:
        try:
            reset_realsense_devices()
            break
        except:
            print("Exception in reset_realsense_devices")
            import traceback
            traceback.print_exc()

            i_retry += 1
            if i_retry >= n_retry:
                return None

            print("try again in 1 second")
            time.sleep(1)
            continue

    return True

configurable_parameters = [{'name': 'camera_name_front',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'camera_namespace_front',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'serial_no_front',                    'default': "'311322304903'", 'description': 'choose device by serial number'},
                           {'name': 'config_file_front',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'frame_id_front',                  'default': "'camera'", 'description': 'yaml config file'},
                           {'name': 'log_level_front',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_with_gpu',          'default': "1", 'description': '[0-No_GPU, 1-GL_GPU]'},
                           {'name': 'output_front',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'output_left',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'output_back',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'output_right',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.profile',           'default': '1280,720,5', 'description': 'color image width'},
                           {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'depth_module.profile',         'default': '424,240,15', 'description': 'depth module profile'},
                           {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.emitter_enabled', 'default': 'false', 'description': 'Depth module emitter enable'},
                           {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "'enable sync mode'"},
                           {'name': 'enable_rgbd',                  'default': 'false', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel',                 'default': 'false', 'description': "'enable accel stream'"},
                           {'name': 'gyro_fps',                     'default': '400', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '100', 'description': "''"},
                           {'name': 'unite_imu_method',             'default': "2", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
                           {'name': 'angular_velocity_cov',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '5.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''},
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc',        'default': 'false', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'spatial_filter.enable',        'default': 'false', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable',       'default': 'false', 'description': 'enable_temporal_filter'},
                           {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

called_count = 0

def launch_setup(context, params, param_name_suffix=''):
    global called_count
    called_count = called_count + 1
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    #params['serial_no'] = params['serial_no' + param_name_suffix]
    params['frame_id'] = params['frame_id' + param_name_suffix]
    params['camera_name'] = params['camera_name' + param_name_suffix]
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)
    return [
        TimerAction(period=called_count*10.0,
        actions=[
        launch_ros.actions.Node(
            package='realsense2_camera',
            #namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output=LaunchConfiguration('output' + param_name_suffix),
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            ),
        ])
    ]

def generate_launch_description():
    print("reset_realsense_devices_with_retry")
    ret = reset_realsense_devices_with_retry()
    if ret is None:
        print("Error: reset_realsense_devices_with_retry")
        return
    print("reset_realsense_devices_with_retry")
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) +
        [
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters), 'param_name_suffix' : "_front"}),
    ])

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

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=<serial number of 1st camera> serial_no2:=<serial number of 2nd camera>

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [       {'name': 'camera_name1',                  'default': 'camera1', 'description': 'camera unique name'},
                           {'name': 'camera_namespace1',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'serial_no1',                    'default': "'146222253102'", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id1',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type1',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file1',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path1',               'default': [os.path.join(get_package_share_directory('robotname_perception'),'config'),'/realsense.json'], 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset1',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl1',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename1',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'log_level1',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output1',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color1',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile1',     'default': '640,480,15', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format1',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure1', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth1',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'depth_module.depth_profile1',   'default': '640,480,15', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format1',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_profile1',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'depth_module.infra_format1',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format1',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format1',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.exposure1',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain1',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.hdr_enabled1',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure1', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'enable_sync1',                  'default': 'true', 'description': "'enable sync mode'"},
                           {'name': 'enable_rgbd1',                  'default': 'true', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro1',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel1',                 'default': 'false', 'description': "'enable accel stream'"},
                           {'name': 'gyro_fps1',                     'default': '0', 'description': "''"},
                           {'name': 'accel_fps1',                    'default': '0', 'description': "''"},
                           {'name': 'unite_imu_method1',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance1',                'default': '-2.', 'description': "''"},
                           {'name': 'angular_velocity_cov1',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov1',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period1',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf1',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate1',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable1',            'default': 'false', 'description': ''},
                           {'name': 'pointcloud.stream_filter1',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter1','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc1',        'default': 'false', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points1', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable1',           'default': 'true', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable1',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable1',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'spatial_filter.enable1',        'default': 'false', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable1',       'default': 'false', 'description': 'enable_temporal_filter'},
                           {'name': 'disparity_filter.enable1',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable1',   'default': 'true', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable1',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout1',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout1',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                           
                           {'name': 'camera_name2',                  'default': 'camera2', 'description': 'camera unique name'},
                           {'name': 'camera_namespace2',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'serial_no2',                    'default': "'234222302035'", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id2',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type2',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file2',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path2',               'default': [os.path.join(get_package_share_directory('robotname_perception'),'config'),'/realsense.json'], 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset2',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl2',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename2',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'log_level2',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output2',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color2',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile2',     'default': '640,480,15', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format2',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth2',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'depth_module.depth_profile2',   'default': '640,480,15', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format2',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_profile2',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'depth_module.infra_format2',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format2',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format2',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.exposure2',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain2',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.hdr_enabled2',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'enable_sync2',                  'default': 'true', 'description': "'enable sync mode'"},
                           {'name': 'enable_rgbd2',                  'default': 'true', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro2',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel2',                 'default': 'false', 'description': "'enable accel stream'"},
                           {'name': 'gyro_fps2',                     'default': '0', 'description': "''"},
                           {'name': 'accel_fps2',                    'default': '0', 'description': "''"},
                           {'name': 'unite_imu_method2',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance2',                'default': '-2.', 'description': "''"},
                           {'name': 'angular_velocity_cov2',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov2',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period2',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf2',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate2',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable2',            'default': 'false', 'description': ''},
                           {'name': 'pointcloud.stream_filter2',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter2','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc2',        'default': 'false', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points2', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable2',           'default': 'true', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable2',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable2',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'spatial_filter.enable2',        'default': 'false', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable2',       'default': 'false', 'description': 'enable_temporal_filter'},
                           {'name': 'disparity_filter.enable2',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable2',   'default': 'true', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable2',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout2',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout2',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                        ]
def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
    ])

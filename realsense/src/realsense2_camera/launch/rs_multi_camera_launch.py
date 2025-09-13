import copy
import os
import sys
import yaml
import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [
    {'name': 'camera_name1', 'default': 'camera_h', 'description': 'Unique name for Camera Head'},
    {'name': 'serial_no1',   'default': '_',        'description': 'Serial number for Camera Head'},

    {'name': 'camera_name2', 'default': 'camera_l', 'description': 'Unique name for Camera Left'},
    {'name': 'serial_no2',   'default': '_230322275601', 'description': 'Serial number for Camera Left'},

    {'name': 'camera_name3', 'default': 'camera_r', 'description': 'Unique name for Camera Right'},
    {'name': 'serial_no3',   'default': '_',        'description': 'Serial number for Camera Right'},
]

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def set_configurable_parameters(local_params):
    return {
        param['original_name']: LaunchConfiguration(param['name'])
        for param in local_params
    }

def duplicate_params(general_params, suffix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += suffix
    return local_params

def generate_launch_description():
    camera_ids = ['1', '2', '3']
    launch_description = []

    launch_description += rs_launch.declare_configurable_parameters(local_parameters)

    local_names = {p['name'] for p in local_parameters}

    for cam_id in camera_ids:
        serial_key = f'serial_no{cam_id}'
        serial_value = LaunchConfiguration(serial_key)

        params_all = duplicate_params(rs_launch.configurable_parameters, cam_id)

        params = [p for p in params_all if p['name'] not in local_names]

        launch_description.append(
            GroupAction(
                actions=[
                    *rs_launch.declare_configurable_parameters(params),
                    OpaqueFunction(
                        function=rs_launch.launch_setup,
                        kwargs={
                            'params': set_configurable_parameters(params),
                            'param_name_suffix': cam_id,
                        }
                    )
                ],
                condition=IfCondition(PythonExpression(["'", serial_value, "' != '_'"]))
            )
        )

    return LaunchDescription(launch_description)

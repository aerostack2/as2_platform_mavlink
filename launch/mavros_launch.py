#!/usr/bin/env python3

# Copyright 2023 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Launch px4 node."""

__authors__ = 'Miguel Fernández Cortizas, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_node(context, *args, **kwargs) -> list:
    """
    Get node.

    :param context: Launch context
    :type context: LaunchContext
    :return: List with node
    :rtype: list
    """
    # Get plugin name
    config_yaml = PathJoinSubstitution(
        [FindPackageShare('mavros'), 'launch', 'px4_config.yaml'])
    pluginlists_yaml = PathJoinSubstitution(
        [FindPackageShare('mavros'), 'launch', 'px4_pluginlists.yaml'])
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace == 'mavros':
        ns = 'mavros'
    else:
        ns = namespace + '/mavros'
    print(f'Namespace: {ns}')

    return [
        Node(
            package='mavros',
            executable='mavros_node',
            namespace=ns,
            output=LaunchConfiguration('log_output'),
            parameters=[{'fcu_url': LaunchConfiguration('fcu_url'),
                         'gcs_url': LaunchConfiguration('gcs_url'),
                         'tgt_system': LaunchConfiguration('tgt_system'),
                         'tgt_component': LaunchConfiguration('tgt_component'),
                         'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                         'respawn_mavros': LaunchConfiguration('respawn_mavros')},
                        pluginlists_yaml, config_yaml],
            remappings=[('imu/data', f'/{namespace}/sensor_measurements/imu'),
                        ('gps/fix', f'/{namespace}/sensor_measurements/gps_raw'),
                        ('odometry/in', f'/{namespace}/sensor_measurements/odometry'),
                        ],
        )]


def generate_launch_description():
    """Entrypoint."""
    # Declare the launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:57600',
            description='URL for the FCU'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='URL for the GCS'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),
        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='Logging output type'
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='Protocol version for the FCU'
        ),
        DeclareLaunchArgument(
            'respawn_mavros',
            default_value='false',
            description='Respawn mavros node if it dies'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='mavros',
            description='Namespace for the mavros node'
        ),

        OpaqueFunction(function=get_node),

        # Include the node launch file
    ])

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = 'robot1'
    model_path = PathJoinSubstitution([
        FindPackageShare('rokey_pjt'),
        'models',
        'my_best_v2.pt',
    ])

    return LaunchDescription([
        Node(
            package='rokey_pjt',
            executable='mission_controller',
            namespace=namespace,
            name='mission_controller',
            output='screen',
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'),
            ],
            parameters=[
                {
                    'model_path': model_path,
                    'target_class_id': 1,
                }
            ]
        ),

        Node(
            package='rokey_pjt',
            executable='webcam_trigger',
            name='webcam_trigger',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'target_class_id': 1,
            }],
        ),
    ])

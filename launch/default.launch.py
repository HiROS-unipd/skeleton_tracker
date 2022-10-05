from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node=Node(
        package = 'hiros_skeleton_tracker',
        executable = 'hiros_skeleton_tracker',
        name = 'skeleton_tracker',
        namespace='hiros',
        output = 'screen',
        parameters = [
            {'input_topics': ['/input/topic1', '/input/topic2']},
            {'output_topic': '/output/topic'},
            {'fixed_delay': 0.},
            {'min_skeleton_confidence': -1.},
            {'min_marker_confidence': -1.},
            {'min_link_confidence': -1.},
            {'min_markers': 0},
            {'min_links': 0},
            {'min_linear_distance': -1.},
            {'max_linear_distance': -1.},
            {'min_angular_distance': -1.},
            {'max_angular_distance': -1.},
            {'max_delta_t': 0.},
            {'use_positions': True},
            {'use_linear_velocities': False},
            {'use_orientations': False},
            {'use_angular_velocities': False},
            {'velocity_weight': 0.},
            {'weight_distances_by_confidences': False},
            {'weight_distances_by_velocities': False},
        ]
    )

    ld.add_action(node)
    return ld

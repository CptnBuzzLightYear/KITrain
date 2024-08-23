from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'rgb_camera', 
            executable = 'rgb_camera_receiver_main', 
            name = 'screen'),
            
        Node(
            package = 'lidar', 
            executable = 'lidar_udp_publisher', 
            name = 'screen2'), 

        Node(
            package = 'gnss', 
            executable = 'gnss_udp_publisher', 
            name = 'screen5'),     

        Node(
            package = 'order_definer',
            executable = 'order_definer_node',
            name = 'order',
        ),
         Node(
            package = 'signal_detector',
            executable = 'signal_detector_node',
            name = 'order',
        ),

        Node(
            package = 'yard_describer',
            executable = 'yard_describer',
            name = 'map'
        ),
        Node(
            package = 'track_setter',
            executable = 'track_setter',
            name = 'positioning'
        ),
        Node(
            package = 'localizer',
            executable = 'localizer',
            name = 'positioning',
        ),
        Node(
            package = 'clearance_profile_setter',
            executable = 'clearance_profile_setter',
            name = 'CPS'
        ),
        Node(
            package = 'clearance_profile_observer',
            executable = 'clearance_profile_observer',
            name = 'observer'
        ),

        Node(
            package='static_transform_broadcaster',
            executable='static_transform_broadcaster',
            name='tatic_transform_broadcaster'),

         Node(
            package='odometry',
            executable='odometry_receiver',
            name='odometry_receiver'),

        Node(
            package='actor',
            executable='actor',
            name='actor'),


       Node(
            package = 'track_detector', 
            executable = 'track_detect_main', 
            name = 'screen3'), 

        Node(
            package = 'voxel_filter', 
            executable = 'voxel_filter', 
            name = 'screen3'),   

              Node(
            package = 'clustering', 
            executable = 'clustering', 
            name = 'screen3'),  


        Node(
            package = 'foxglove_bridge',
            executable = 'foxglove_bridge',
            name = 'foxglove_bridge',
            output = 'screen',
            parameters = [
              {'foxglove_bridge/topics':['image_topic_']}
            ]  
        )
        
    ])
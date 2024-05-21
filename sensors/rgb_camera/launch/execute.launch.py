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

        #Node(
         #   package = 'camera_simulator',
          #  executable = 'camera_simulator',
           # name = 'camera'
        #),

       Node(
            package = 'track_detector', 
            executable = 'track_detect_main', 
            name = 'screen2'), 

     #   Node(
       #     package = 'live_position', 
        #    executable = 'live_position_main', 
         #   name = 'screen3'),   


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
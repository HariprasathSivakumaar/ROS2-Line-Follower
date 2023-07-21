from launch import LaunchDescription
from launch.actions import  ExecuteProcess


def generate_launch_description():
 
  return LaunchDescription([
   
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/hariprasath/ros2_ws/src/prius_line_following/world/prius_on_track.world'],
            output='screen'),
  
  ])

#!/usr/bin/env python3
"""Observe the ROS waypoint follower climbing the first Unity ramp; publishes no commands."""
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init();node=rclpy.create_node('unity_patrol_observer');poses=[];camera=[None];captured=False
sub=node.create_subscription(Odometry,'/odom',lambda m:poses.append(m),qos_profile_sensor_data)
image_sub=node.create_subscription(Image,'/camera/image',lambda m:camera.__setitem__(0,m),qos_profile_sensor_data)
end=time.monotonic()+75
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.1)
    if not poses:continue
    p=poses[-1].pose.pose.position
    if not captured and p.x>8 and camera[0] is not None:
        img=camera[0];pixels=np.frombuffer(bytes(img.data),dtype=np.uint8).reshape((img.height,img.width,3));cv2.imwrite('/tmp/unity-ros-patrol-camera.jpg',cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR));captured=True
    if p.x>14 and p.z>.7:break
max_x=max((m.pose.pose.position.x for m in poses),default=0);max_z=max((m.pose.pose.position.z for m in poses),default=-999)
result={'odom_samples':len(poses),'furthest_forward_m':max_x,'height_gain_m':max_z,'single_ros_command_source':node.count_publishers('/cmd_vel')==1,'ros_patrol_climbs_unity_ramp':max_x>13 and max_z>.7,'camera_captured':captured}
result['passed']=result['ros_patrol_climbs_unity_ramp'] and result['single_ros_command_source'] and captured
Path('/tmp/unity-ros-patrol-verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()
if not result['passed']:raise SystemExit(1)

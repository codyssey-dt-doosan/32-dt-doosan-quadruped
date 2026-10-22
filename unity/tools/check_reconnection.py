#!/usr/bin/env python3
"""After restarting rosbridge, check that Unity reconnects and stays stopped."""
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init(); node=rclpy.create_node('unity_reconnection_check'); samples=[]
def receive(msg): samples.append(msg)
sub=node.create_subscription(Odometry,'/odom',receive,qos_profile_sensor_data)
end=time.monotonic()+15
while len(samples)<10 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
if not samples:raise RuntimeError('Unity did not reconnect')
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
a,b=samples[0].pose.pose.position,samples[-1].pose.pose.position
span=math.dist((a.x,a.y),(b.x,b.y)); twist=samples[-1].twist.twist
result={'reconnected':True,'samples':len(samples),'idle_displacement_m':span,'stale_command_not_replayed':span<.03 and abs(twist.linear.x)<.03 and abs(twist.angular.z)<.03}
Path('/tmp/unity-reconnection.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()
if not result['stale_command_not_replayed']:raise SystemExit(1)

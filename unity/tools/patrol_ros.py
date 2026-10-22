#!/usr/bin/env python3
"""ROS 2 baseline waypoint follower for the exported Unity route (not MPC).
Unity only executes cmd_vel; route decisions are made in this ROS node.
"""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default='/workspace/unity-config/ros_route.json')
    parser.add_argument('--speed', type=float, default=.5)
    args = parser.parse_args()
    route = json.loads(Path(args.route).read_text())
    if route['frame_id'] != 'odom' or not route['waypoints']:
        raise ValueError('Expected nonempty odom-frame Unity route')
    rclpy.init(); node = rclpy.create_node('unity_ros_patrol')
    state = {'pose': None, 'received': 0., 'index': 0, 'wait': 0., 'stamp': -1}
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    def receive(msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp < state['stamp']:
            state.update(index=0, wait=0.)
        state.update(pose=msg, received=time.monotonic(), stamp=stamp)
    sub = node.create_subscription(Odometry, '/odom', receive, 10)
    def tick():
        cmd = Twist(); now = time.monotonic(); msg = state['pose']
        if msg is not None and now-state['received'] < 1 and now >= state['wait']:
            target = route['waypoints'][state['index']]; p = msg.pose.pose.position; q = msg.pose.pose.orientation
            dx,dy = target['x']-p.x,target['y']-p.y; distance = math.hypot(dx,dy)
            if distance < .12:
                state['wait'] = now+target.get('dwell_s',0); state['index'] = (state['index']+1)%len(route['waypoints'])
                node.get_logger().info('Reached '+target['name'])
            else:
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                error = math.atan2(math.sin(math.atan2(dy,dx)-yaw), math.cos(math.atan2(dy,dx)-yaw))
                cmd.angular.z = max(-.8,min(.8,1.8*error))
                cmd.linear.x = min(max(0,args.speed),distance)*max(0.,math.cos(error)) if abs(error)<.8 else 0.
        pub.publish(cmd)
    timer = node.create_timer(.05,tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    node.get_logger().info('Unity ROS waypoint follower ready; waiting for /odom. Do not run another cmd_vel publisher.')
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        for _ in range(3): pub.publish(Twist())
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()

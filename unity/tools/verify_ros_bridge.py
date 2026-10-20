#!/usr/bin/env python3
"""Exercise actual ROS 2 <-> Unity traffic. Run while Plant_ROS2 is in Play.
Sends a short forward/left-turn/reverse sequence and checks command timeout.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/unity-ros-verification.json')
    parser.add_argument('--image', default='/tmp/unity-ros-camera.jpg')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('unity_ros_verification')
    latest, counts, frames = {}, {}, set()
    subscriptions = []
    for topic, kind in [('/odom', Odometry), ('/camera/image', Image), ('/camera/camera_info', CameraInfo), ('/points', PointCloud2), ('/imu', Imu), ('/clock', Clock), ('/tf', TFMessage)]:
        def receive(msg, topic=topic):
            latest[topic] = msg
            counts[topic] = counts.get(topic, 0)+1
            if topic == '/tf':
                frames.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)
        subscriptions.append(node.create_subscription(kind, topic, receive, qos_profile_sensor_data))
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    result = {'checks': {}, 'message_counts': counts}

    def spin(seconds, v=None, w=0):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            if v is not None:
                cmd = Twist(); cmd.linear.x = float(v); cmd.angular.z = float(w); pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=.025)
            time.sleep(.015)

    try:
        until = time.monotonic()+90
        while len(latest) < 7 and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        missing = sorted(set(['/odom','/camera/image','/camera/camera_info','/points','/imu','/clock','/tf'])-latest.keys())
        if missing:
            raise RuntimeError('Missing Unity topics: '+', '.join(missing))
        spin(1, 0)
        start = latest['/odom'].pose.pose.position
        start_xy = (start.x, start.y)
        start_yaw = yaw(latest['/odom'])
        spin(4, .3)
        p = latest['/odom'].pose.pose.position
        forward = (p.x-start_xy[0])*math.cos(start_yaw)+(p.y-start_xy[1])*math.sin(start_yaw)
        result['forward_distance_m'] = forward
        result['checks']['ros_command_moves_unity_forward'] = .6 < forward < 1.8
        before_turn = yaw(latest['/odom'])
        spin(2, 0, .4)
        turned = angle(yaw(latest['/odom'])-before_turn)
        result['left_turn_rad'] = turned
        result['checks']['positive_angular_z_turns_left'] = .35 < turned < 1.3
        # No velocity publications: command timeout must stop movement on Unity's side.
        spin(1.2)
        stopped_yaw = yaw(latest['/odom'])
        spin(.8)
        stop_turn = abs(angle(yaw(latest['/odom'])-stopped_yaw))
        twist = latest['/odom'].twist.twist
        result['timeout_extra_turn_rad'] = stop_turn
        result['checks']['timeout_stops_robot'] = stop_turn < .04 and abs(twist.angular.z) < .05 and abs(twist.linear.x) < .03
        p = latest['/odom'].pose.pose.position
        reverse_start = (p.x, p.y); reverse_yaw = yaw(latest['/odom'])
        spin(2, -.2)
        p = latest['/odom'].pose.pose.position
        reverse = (p.x-reverse_start[0])*math.cos(reverse_yaw)+(p.y-reverse_start[1])*math.sin(reverse_yaw)
        result['reverse_distance_m'] = reverse
        result['checks']['reverse_command_works'] = -.8 < reverse < -.15
        spin(1, 0)
        image = latest['/camera/image']; raw = np.frombuffer(bytes(image.data), dtype=np.uint8)
        pixels = raw.reshape((image.height, image.width, 3))
        cv2.imwrite(args.image, cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
        result['camera'] = {'width': image.width, 'height': image.height, 'encoding': image.encoding, 'pixel_std': float(pixels.std())}
        result['checks']['real_rgb_camera_data'] = image.encoding == 'rgb8' and image.step == image.width*3 and pixels.std() > 2
        info = latest['/camera/camera_info']
        result['checks']['camera_intrinsics_and_optical_frame'] = info.width == image.width and info.height == image.height and info.k[0] > 0 and info.header.frame_id == image.header.frame_id == 'camera_optical_frame'
        cloud = latest['/points']; xyz = np.frombuffer(bytes(cloud.data), dtype='<f4').reshape((-1,3)); finite = xyz[np.isfinite(xyz).all(axis=1)]
        result['lidar'] = {'width': cloud.width, 'height': cloud.height, 'finite_returns': len(finite), 'point_step': cloud.point_step}
        result['checks']['real_lidar_returns'] = len(finite) > 100 and cloud.point_step == 12 and cloud.row_step == cloud.width*12 and np.max(np.linalg.norm(finite,axis=1)) <= 20.01 and np.min(finite[:,2]) < -.4
        result['checks']['single_odometry_source'] = node.count_publishers('/odom') == 1
        result['checks']['tf_chain'] = {('map','odom'),('odom','base_link'),('base_link','lidar'),('base_link','camera_link'),('camera_link','camera_optical_frame')} <= frames
        imu = latest['/imu']; result['checks']['synthetic_imu_has_gravity_support'] = 8 < imu.linear_acceleration.z < 12
        result['checks']['simulation_clock_and_timestamp'] = latest['/clock'].clock.sec > 0 and abs(latest['/clock'].clock.sec-latest['/odom'].header.stamp.sec) < 2
        result['checks'] = {key: bool(value) for key, value in result['checks'].items()}
        result['passed'] = all(result['checks'].values())
    except Exception as ex:
        result['passed'] = False; result['error'] = str(ex)
    finally:
        for _ in range(3): pub.publish(Twist()); rclpy.spin_once(node, timeout_sec=.02)
        Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

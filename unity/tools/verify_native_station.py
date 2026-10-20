#!/usr/bin/env python3
"""Read-only verification of the original three gauges at the patrol inspection stop."""
import json
import math
from pathlib import Path
import time
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init();node=rclpy.create_node('native_gauge_station_observer');checks={};values={};phase=[''];last_phase=['']
    def mission(msg):
        phase[0]=msg.data
        if msg.data!=last_phase[0]:print(msg.data,flush=True);last_phase[0]=msg.data
        if msg.data.startswith('inspect:'):checks['inspection_stop']=True
    def inspection(msg):
        s=json.loads(msg.data)
        if not phase[0].startswith('inspect:'):return
        if s.get('gauge_valid') and len(s.get('gauges_bar',[]))==3:
            values['gauges_bar']=s['gauges_bar'];checks['all_three_original_gauges_read']=all(math.isfinite(v) and 0<=v<=100 for v in s['gauges_bar'])
        if s.get('thermal_valid'):
            values['temperature_c']=s['temperature_c'];checks['native_equipment_temperature']=abs(s['temperature_c']-95)<.01
    def odom(msg):
        if phase[0].startswith('inspect:'):
            q=msg.pose.pose.orientation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            values['inspection_yaw_rad']=angle;checks['facing_native_equipment']=abs(angle+1.51945)<.08
    subs=[node.create_subscription(String,'/mission/status',mission,10),node.create_subscription(String,'/inspection/status',inspection,10),node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)]
    deadline=time.monotonic()+180
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if phase[0]=='goto:Machine_East' and all(checks.get(k) for k in ['inspection_stop','all_three_original_gauges_read','native_equipment_temperature','facing_native_equipment']):break
    checks['single_cmd_vel_source']=node.count_publishers('/cmd_vel')==1
    result={'checks':checks,'values':values,'passed':all(checks.get(k,False) for k in ['inspection_stop','all_three_original_gauges_read','native_equipment_temperature','facing_native_equipment','single_cmd_vel_source'])}
    Path('/tmp/native-station-verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown();return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

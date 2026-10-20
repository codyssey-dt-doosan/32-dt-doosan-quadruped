#!/usr/bin/env python3
"""Read-only live ROS observer; never publishes velocity or supplies simulated detections."""
import json
import math
from pathlib import Path
import time
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init();node=rclpy.create_node('unity_inspection_observer');checks={};values={};phases=[];latest={}
    def mission(msg):
        phase=msg.data;latest['phase']=phase
        if not phases or phases[-1]!=phase:phases.append(phase);print(phase,flush=True)
        if phase.startswith('inspect:'):checks['inspection_stop']=True
        if phase.startswith('return:'):checks['gas_alarm_triggers_return']=True
    def inspect(msg):
        s=json.loads(msg.data)
        if s.get('gauge_valid'):
            values['gauge_bar']=s['gauge_bar'];checks['rgb_gauge_accuracy']=abs(s['gauge_bar']-6.2)<.15
        if s.get('thermal_valid') and s['temperature_c']>80:
            values['temperature_c']=s['temperature_c'];checks['radiometric_temperature']=abs(s['temperature_c']-95)<.01
            checks['thermal_alert']=s['thermal_alert']
        if s.get('gas_alert'):
            values.setdefault('first_alarm_ppm',s['gas_ppm'])
            values['max_observed_alarm_ppm']=max(values.get('max_observed_alarm_ppm',0),s['gas_ppm'])
            values['dose_ppm_s']=s['exposure_ppm_s'];checks['gas_exposure_alarm']=True
    def odom(msg):latest['pose']=msg.pose.pose.position
    def cmd(msg):latest['cmd']=msg
    subs=[node.create_subscription(String,'/mission/status',mission,10),node.create_subscription(String,'/inspection/status',inspect,10),node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data),node.create_subscription(Twist,'/cmd_vel',cmd,10)]
    deadline=time.monotonic()+600;home_since=None
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if latest.get('phase','').startswith('home_safe'):
                if home_since is None:home_since=time.monotonic()
                if time.monotonic()-home_since>1:
                    p=latest['pose'];c=latest['cmd'];values['home_distance_m']=math.hypot(p.x,p.y)
                    checks['returned_home']=values['home_distance_m']<.2
                    checks['stopped_at_home']=abs(c.linear.x)+abs(c.angular.z)<1e-6
                    checks['single_cmd_vel_publisher']=node.count_publishers('/cmd_vel')==1
                    break
        required=['inspection_stop','rgb_gauge_accuracy','radiometric_temperature','thermal_alert','gas_exposure_alarm','gas_alarm_triggers_return','returned_home','stopped_at_home','single_cmd_vel_publisher']
        result={'checks':{k:bool(checks.get(k,False)) for k in required},'values':values,'phases':phases}
        result['passed']=all(result['checks'].values())
        Path('/tmp/inspection-verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
        return 0 if result['passed'] else 1
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())

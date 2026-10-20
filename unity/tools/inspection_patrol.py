#!/usr/bin/env python3
"""Single cmd_vel owner: inspect the station, then retreat along visited waypoints on gas alarm."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


class Patrol(Node):
    def __init__(self):
        super().__init__('unity_inspection_patrol')
        self.route=json.loads(Path('/workspace/unity-config/inspection_route.json').read_text())['waypoints']
        config=json.loads(Path('/workspace/unity-config/inspection.json').read_text())
        self.cruise_speed=float(config.get('cruise_speed_mps',.85))
        if not math.isfinite(self.cruise_speed) or not .05<=self.cruise_speed<=2.:
            raise ValueError('cruise_speed_mps must be between 0.05 and 2.0 m/s')
        self.pub=self.create_publisher(Twist,'/cmd_vel',10);self.mission=self.create_publisher(String,'/mission/status',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(String,'/inspection/status',self.on_inspection,10)
        self.pose=None;self.odom_at=0.;self.sensor_at=0.;self.sensor={};self.stamp=-1
        self.reset();self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def reset(self):
        self.index=0;self.mode='patrol';self.phase='waiting_for_sensors';self.visited=[];self.retreat=[]
        self.dwell_until=0.;self.return_after=0.;self.station={};self.events=[];self.gas_latched=False;self.inspection_failed=False
        self.sensor_at=0.;self.sensor={}

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp<self.stamp:self.reset()
        self.stamp=stamp;self.pose=msg;self.odom_at=time.monotonic()

    def on_inspection(self,msg):
        try:self.sensor=json.loads(msg.data);self.sensor_at=time.monotonic()
        except (ValueError,TypeError):return

    def set_phase(self,phase):
        if phase!=self.phase:
            self.phase=phase;self.events.append({'phase':phase,'sim_s':self.stamp});self.get_logger().info(phase)
        self.mission.publish(String(data=phase))
        Path('/tmp/inspection-patrol.json').write_text(json.dumps({'phase':phase,'index':self.index,'gas_latched':self.gas_latched,'station':self.station,'events':self.events},indent=2))

    def navigate(self,target,cmd):
        p=self.pose.pose.pose.position;q=self.pose.pose.pose.orientation
        dx,dy=target['x']-p.x,target['y']-p.y;distance=math.hypot(dx,dy)
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        error=wrap(math.atan2(dy,dx)-yaw)
        cmd.angular.z=max(-1.,min(1.,2.2*error))
        cmd.linear.x=min(self.cruise_speed,distance*1.4)*max(0.,math.cos(error)) if abs(error)<.65 else 0.
        if distance<.15:cmd.linear.x=cmd.angular.z=0.;return True,yaw
        return False,yaw

    def tick(self):
        cmd=Twist();now=time.monotonic();s=self.sensor
        fresh=self.pose is not None and now-self.odom_at<1 and now-self.sensor_at<2 and s.get('thermal_valid') and s.get('gas_valid')
        if not fresh:self.set_phase('hold:sensor_data_missing');self.pub.publish(cmd);return
        if self.mode=='home':self.set_phase('home_safe:gas_alarm_latched');self.pub.publish(cmd);return
        if s.get('gas_alert') and not self.gas_latched:
            self.gas_latched=True;self.mode='return';self.return_after=self.stamp+2
            self.retreat=[self.route[i] for i in reversed(self.visited)]+[{'name':'Home','x':0.,'y':0.}]
            self.set_phase('alarm:gas_stop_then_return')
        if self.mode=='return':
            if self.stamp<self.return_after:self.pub.publish(cmd);return
            target=self.retreat[0];arrived,_=self.navigate(target,cmd)
            self.set_phase('return:'+target['name'])
            if arrived:
                self.retreat.pop(0)
                if not self.retreat:self.mode='home';self.set_phase('home_safe:gas_alarm_latched')
        elif self.inspection_failed:
            self.set_phase('hold:inspection_failed');cmd=Twist()
        else:
            target=self.route[self.index];arrived,yaw=self.navigate(target,cmd)
            if arrived and target.get('inspection'):
                error=wrap(target['yaw']-yaw)
                if abs(error)>.035:
                    cmd.angular.z=max(-.7,min(.7,2.2*error));self.set_phase('align:'+target['name'])
                else:
                    if not self.dwell_until:self.dwell_until=self.stamp+target.get('dwell_s',12)
                    self.set_phase('inspect:'+target['name'])
                    if s.get('gauge_valid'):
                        self.station['gauge_bar']=s['gauge_bar'];self.station['gauge_sim_s']=self.stamp
                    if s.get('thermal_valid'):
                        self.station['temperature_c']=max(self.station.get('temperature_c',-273),s['temperature_c'])
                    if self.stamp>=self.dwell_until:
                        if 'gauge_bar' not in self.station or 'temperature_c' not in self.station:
                            self.inspection_failed=True;self.set_phase('hold:inspection_failed')
                        else:self.advance()
            elif arrived:self.advance()
            else:self.set_phase('goto:'+target['name'])
        self.pub.publish(cmd)

    def advance(self):
        if self.index not in self.visited:self.visited.append(self.index)
        self.index+=1;self.dwell_until=0.
        if self.index>=len(self.route):self.index=0;self.visited=[];self.station={}


def main():
    rclpy.init();node=Patrol()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.pub.publish(Twist());node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Read the Unity RGB gauge, radiometric 32FC1 temperature map and synthetic gas sensor in ROS 2."""
from collections import deque
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from gauge_ocr.reader import GaugeSpec, read_gauge
from inspection_logic import accumulate_exposure, alarm_gas


class Inspection(Node):
    def __init__(self):
        super().__init__('unity_plant_inspection')
        self.cfg=json.loads(Path('/workspace/unity-config/inspection.json').read_text())
        self.clock=0.;self.last_clock=-1.;self.phase='';self.speed=0.
        self.values={'gauge_bar':0.,'temperature_c':0.,'gas_ppm':0.,'exposure_ppm_s':0.}
        self.received={};self.window=deque(maxlen=7);self.previous_gas_time=None;self.previous_ppm=0.
        self.events=[];self.ever={'gauge':False,'thermal':False,'gas_alarm':False};self.counts={'rgb':0,'thermal':0,'gas':0}
        self.pub_status=self.create_publisher(String,'/inspection/status',10)
        self.pub_gauge=self.create_publisher(Float32,'/inspection/gauge',10)
        self.pub_temp=self.create_publisher(Float32,'/inspection/temperature_c',10)
        self.pub_hot=self.create_publisher(Bool,'/inspection/thermal_alert',10)
        self.pub_dose=self.create_publisher(Float32,'/gas/exposure',10)
        self.pub_alarm=self.create_publisher(Bool,'/gas/alert',10)
        self.pub_debug=self.create_publisher(Image,'/inspection/gauge_debug',10)
        self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(String,'/mission/status',self.on_mission,10)
        self.create_subscription(Image,'/camera/image',self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(Image,'/thermal/image',self.on_thermal,qos_profile_sensor_data)
        self.create_subscription(Float32,'/gas/concentration',self.on_gas,10)
        self.create_timer(.2,self.tick)

    def on_clock(self,msg):
        now=msg.clock.sec+msg.clock.nanosec*1e-9
        if now < self.last_clock:
            self.received.clear();self.window.clear();self.previous_gas_time=None;self.values['exposure_ppm_s']=0.
            self.ever={'gauge':False,'thermal':False,'gas_alarm':False};self.events=[]
        self.last_clock=self.clock=now;self.received['clock']=time.monotonic()

    def on_odom(self,msg):
        v=msg.twist.twist.linear;self.speed=math.hypot(v.x,v.y)

    def on_mission(self,msg):
        if msg.data!=self.phase:self.window.clear();self.received.pop('gauge',None)
        self.phase=msg.data;self.received['mission']=time.monotonic()

    def mark(self,key,value):
        self.received[key]=time.monotonic()
        if not self.ever[key]:
            self.ever[key]=True;self.events.append({'event':key,'sim_s':self.clock,'value':float(value)})
            self.get_logger().info(f'{key}: {value:.2f}')

    def on_rgb(self,msg):
        self.counts['rgb']+=1
        if not self.phase.startswith('inspect:') or self.speed>.03 or not self.fresh('mission'):return
        if msg.encoding!='rgb8' or msg.step!=msg.width*3:return
        try: bgr=cv2.cvtColor(np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.width,3),cv2.COLOR_RGB2BGR)
        except ValueError:return
        value,dbg=read_gauge(bgr,GaugeSpec())
        if not math.isfinite(value):return
        self.window.append(value);self.values['gauge_bar']=float(np.median(self.window));self.mark('gauge',value)
        from gauge_ocr.overlay import draw_overlay
        debug=draw_overlay(bgr,value,dbg)
        out=Image();out.header=msg.header;out.width=msg.width;out.height=msg.height;out.encoding='bgr8';out.step=msg.width*3;out.data=debug.tobytes();self.pub_debug.publish(out)
        cv2.imwrite('/tmp/inspection-gauge.jpg',debug)

    def on_thermal(self,msg):
        if msg.encoding!='32FC1' or msg.step!=msg.width*4:return
        try: pixels=np.frombuffer(bytes(msg.data),dtype='>f4' if msg.is_bigendian else '<f4').reshape(msg.height,msg.width)
        except ValueError:return
        if not np.isfinite(pixels).all():return
        self.counts['thermal']+=1;self.values['temperature_c']=float(pixels.max());self.received['thermal']=time.monotonic()
        if pixels.max()>=self.cfg['temperature_alarm_c']:
            self.mark('thermal',pixels.max())
            heat=cv2.applyColorMap(np.uint8(np.clip((pixels-22)/80,0,1)*255),cv2.COLORMAP_INFERNO)
            cv2.imwrite('/tmp/inspection-thermal.jpg',cv2.resize(heat,(320,240),interpolation=cv2.INTER_NEAREST))

    def on_gas(self,msg):
        ppm=float(msg.data)
        if not math.isfinite(ppm) or ppm<0 or not self.fresh('clock'):return
        self.counts['gas']+=1
        self.values['exposure_ppm_s']=accumulate_exposure(self.previous_gas_time,self.clock,self.previous_ppm,ppm,self.values['exposure_ppm_s'])
        self.previous_gas_time=self.clock;self.previous_ppm=ppm;self.values['gas_ppm']=ppm;self.received['gas']=time.monotonic()
        if alarm_gas(ppm,self.values['exposure_ppm_s'],self.cfg):self.mark('gas_alarm',ppm)

    def fresh(self,key):return time.monotonic()-self.received.get(key,-1e9)<self.cfg['fresh_seconds']

    def tick(self):
        clock_ok=self.fresh('clock')
        status=dict(self.values,sim_s=self.clock,gauge_valid=clock_ok and self.fresh('gauge'),thermal_valid=clock_ok and self.fresh('thermal'),gas_valid=clock_ok and self.fresh('gas'))
        status.update(gauge_alert=status['gauge_valid'] and status['gauge_bar']>=self.cfg['gauge_alarm_bar'],thermal_alert=status['thermal_valid'] and status['temperature_c']>=self.cfg['temperature_alarm_c'],gas_alert=status['gas_valid'] and alarm_gas(status['gas_ppm'],status['exposure_ppm_s'],self.cfg))
        self.pub_status.publish(String(data=json.dumps(status,allow_nan=False)))
        if status['gauge_valid']:self.pub_gauge.publish(Float32(data=status['gauge_bar']))
        if status['thermal_valid']:
            self.pub_temp.publish(Float32(data=status['temperature_c']));self.pub_hot.publish(Bool(data=status['thermal_alert']))
        if status['gas_valid']:
            self.pub_dose.publish(Float32(data=status['exposure_ppm_s']));self.pub_alarm.publish(Bool(data=status['gas_alert']))
        Path('/tmp/inspection-results.json').write_text(json.dumps({'ever':self.ever,'events':self.events,'counts':self.counts,'latest':status},indent=2))


def main():
    rclpy.init();node=Inspection()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

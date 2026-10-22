#!/usr/bin/env python3
"""Docking regression checks, using ROS message classes without creating publishers."""
import unittest
import time
from types import SimpleNamespace
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from inspection_patrol import Patrol


class HomeDocking(unittest.TestCase):
    def robot(self,x=1,speed=0):
        pose=Odometry();pose.pose.pose.position.x=x
        pose.pose.pose.orientation.z=1.;pose.pose.pose.orientation.w=0.  # Heading toward the origin.
        pose.twist.twist.linear.x=speed
        bot=SimpleNamespace(pose=pose,cruise_speed=1.7,stamp=0.,home_since=None,mode='dock')
        bot.navigate=lambda target,cmd:Patrol.navigate(bot,target,cmd)
        bot.set_phase=lambda phase:setattr(bot,'phase',phase)
        return bot

    def test_home_approach_slows_but_cruise_is_preserved(self):
        bot=self.robot();cmd=Twist()
        Patrol.navigate(bot,{'name':'Home','x':0.,'y':0.},cmd)
        self.assertAlmostEqual(cmd.linear.x,.35)
        Patrol.navigate(bot,{'name':'Transit','x':-2.,'y':0.},cmd)
        self.assertAlmostEqual(cmd.linear.x,1.7)

    def test_near_home_proportional_braking(self):
        bot=self.robot(.3);cmd=Twist()
        Patrol.navigate(bot,{'name':'Home','x':0.,'y':0.},cmd)
        self.assertAlmostEqual(cmd.linear.x,.18)

    def test_arrival_while_moving_is_not_home_safe(self):
        bot=self.robot(.1,.35);Patrol.dock_home(bot,Twist())
        self.assertEqual(bot.mode,'dock');self.assertIsNone(bot.home_since)

    def test_two_stationary_seconds_required(self):
        bot=self.robot(.1);Patrol.dock_home(bot,Twist())
        bot.stamp=1.9;Patrol.dock_home(bot,Twist());self.assertEqual(bot.mode,'dock')
        bot.stamp=2.;Patrol.dock_home(bot,Twist());self.assertEqual(bot.mode,'home')

    def test_overshoot_resets_settling_and_commands_correction(self):
        bot=self.robot(.1);Patrol.dock_home(bot,Twist())
        bot.pose.pose.pose.position.x=.4;bot.stamp=2.1;cmd=Twist();Patrol.dock_home(bot,cmd)
        self.assertIsNone(bot.home_since);self.assertEqual(bot.mode,'dock');self.assertGreater(cmd.linear.x,0.)

    def test_sensor_gap_cannot_count_as_stationary_time(self):
        bot=self.robot(.1);bot.home_since=0.;bot.sensor={};bot.sensor_at=bot.odom_at=0.
        bot.pub=SimpleNamespace(publish=lambda cmd:None)
        Patrol.tick(bot)
        self.assertIsNone(bot.home_since);self.assertEqual(bot.phase,'hold:sensor_data_missing')

    def test_home_drift_restarts_docking(self):
        bot=self.robot(.3);bot.mode='home';bot.sensor={'thermal_valid':True,'gas_valid':True}
        bot.sensor_at=bot.odom_at=time.monotonic();bot.pub=SimpleNamespace(publish=lambda cmd:None)
        bot.dock_home=lambda cmd:Patrol.dock_home(bot,cmd)
        Patrol.tick(bot)
        self.assertEqual(bot.mode,'dock');self.assertEqual(bot.phase,'return:Home')


if __name__=='__main__':unittest.main()

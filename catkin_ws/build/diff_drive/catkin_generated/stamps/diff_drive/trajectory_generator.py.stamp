#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np

class TrajectoryController:
    def __init__(self):
        rospy.init_node('trajectory_controller')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(50)  # 50 Hz
        
    def circle_trajectory(self, radius=1.0, speed=0.5):
        """Genera un movimento circolare"""
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec()
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = speed / radius  # v = ω*r
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tc = TrajectoryController()
        tc.circle_trajectory(radius=1.5, speed=0.3)
    except rospy.ROSInterruptException:
        pass

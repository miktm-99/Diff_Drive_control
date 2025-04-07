#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PointNavigationController:
    def __init__(self):
        rospy.init_node('point_navigation_controller')
        
        # Controller gains
        self.k1 = rospy.get_param('~k1', 0.1)
        self.k2 = rospy.get_param('~k2', 0.1)
        self.k3 = rospy.get_param('~k3', 0.1)
        
        # Robot state
        self.x = 2.0
        self.y = 2.0
        self.theta = 0.0
        
        # Desired goal point
        self.x_goal = rospy.get_param('~x_goal', 10.0)
        self.y_goal = rospy.get_param('~y_goal', 10.0)
        
        # Data logging
        self.x_log = []
        self.y_log = []
        
        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def odom_callback(self, msg):
        # Update current position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Update current orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)
        
        # Debugging print
        rospy.loginfo(f"Odometry - x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}")

    def posture_regulation_control(self):
        # Compute error in world frame
        xe = self.x_goal - self.x
        ye = self.y_goal - self.y
        
        # Convert to polar coordinates
        rho = np.sqrt(xe**2 + ye**2)
        gamma = np.arctan2(ye, xe) - self.theta
        gamma = np.arctan2(np.sin(gamma), np.cos(gamma))  # Normalize gamma
        delta = gamma + self.theta 
        
        # Control law
        v = self.k1 * rho * np.cos(gamma)
        if abs(gamma) > 0.001:
            omega = self.k2 * gamma + self.k1 * np.sin(gamma) * np.cos(gamma) * (1 + self.k3 * delta) / gamma
        else:
            omega = self.k2 * gamma + self.k1 * (1 + self.k3 * delta)  # Approximation near zero
        
        # Debugging print
        rospy.loginfo(f"Control - v: {v:.2f}, omega: {omega:.2f}, rho: {rho:.2f}, gamma: {gamma:.2f}")
        
        return v, omega, rho

    def control_loop(self, event):
        # Calculate control commands
        v, omega, rho = self.posture_regulation_control()
        
        # Stop if close to the goal
        if rho < 0.05:
            v = 0.0
            omega = 0.0
            rospy.loginfo("Goal reached!")
        
        # Publish command
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = omega
        self.cmd_pub.publish(cmd_msg)
        
        # Log data
        self.x_log.append(self.x)
        self.y_log.append(self.y)

    def plot_trajectory(self):
        plt.figure()
        plt.plot(self.x_log, self.y_log, 'b-', label='Actual Path')
        plt.plot(self.x_goal, self.y_goal, 'ro', label='Goal Point')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.title('Robot Trajectory')
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == '__main__':
    try:
        controller = PointNavigationController()
        rospy.spin()
        controller.plot_trajectory()
    except rospy.ROSInterruptException:
        pass

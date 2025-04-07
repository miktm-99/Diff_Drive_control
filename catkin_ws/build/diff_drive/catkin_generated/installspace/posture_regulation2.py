#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TrajectoryFollowingController:
    def __init__(self):
        rospy.init_node('trajectory_following_posture_regulation')
        
        # Controller gains (same as posture_regulation0.py)
        self.k1 = rospy.get_param('~k1', 2.0)
        self.k2 = rospy.get_param('~k2', 0.3)
        self.k3 = rospy.get_param('~k3', 1.0)
        
        # Robot state
        self.x = 2.0
        self.y = 2.0
        self.theta = 0.0
        
        # Trajectory parameters (from diff_drive_io_linearization.py)
        self.R = 3.0
        self.w_d = 1/15.0
        self.t_start = rospy.Time.now().to_sec()
        
        # Data logging
        self.time_log = []
        self.x_log = []
        self.y_log = []
        self.xd_log = []
        self.yd_log = []
        self.v_log = []
        self.omega_log = []
        
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

    def ref_trajectory(self, t):
        """Generate reference trajectory (from diff_drive_io_linearization.py)"""
        xd = self.R * np.sin(2 * self.w_d * t)
        yd = self.R * np.sin(self.w_d * t)
        return xd, yd

    def posture_regulation_control(self, xd, yd):
        """Exact same control law as posture_regulation0.py"""
        # Compute error in world frame
        xe = xd - self.x
        ye = yd - self.y
        
        # Convert to polar coordinates
        rho = np.sqrt(xe**2 + ye**2)
        gamma = np.arctan2(ye, xe) - self.theta
        gamma = np.arctan2(np.sin(gamma), np.cos(gamma))  # Normalize gamma
        delta = gamma + self.theta 
        
        # Control law (identical to original)
        v = self.k1 * rho * np.cos(gamma)
        if abs(gamma) > 0.001:
            omega = self.k2 * gamma + self.k1 * np.sin(gamma) * np.cos(gamma) * (1 + self.k3 * delta) / gamma
        else:
            omega = self.k2 * gamma + self.k1 * (1 + self.k3 * delta)
        
        return v, omega

    def control_loop(self, event):
        # Calculate current time
        t = rospy.Time.now().to_sec() - self.t_start
        
        # Get current desired point on trajectory
        xd, yd = self.ref_trajectory(t)
        
        # Calculate control commands using posture regulation
        v, omega = self.posture_regulation_control(xd, yd)
        
        # Publish command
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = omega
        self.cmd_pub.publish(cmd_msg)
        
        # Log data
        self.time_log.append(t)
        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.xd_log.append(xd)
        self.yd_log.append(yd)
        self.v_log.append(v)
        self.omega_log.append(omega)

    def plot_results(self):
        plt.figure(figsize=(12, 8))
        
        # Plot trajectory
        plt.subplot(2, 2, 1)
        plt.plot(self.xd_log, self.yd_log, 'r--', label='Desired')
        plt.plot(self.x_log, self.y_log, 'b-', label='Actual')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.title('Trajectory Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot x position
        plt.subplot(2, 2, 2)
        plt.plot(self.time_log, self.xd_log, 'r--', label='X desired')
        plt.plot(self.time_log, self.x_log, 'b-', label='X actual')
        plt.xlabel('Time [s]')
        plt.ylabel('X Position [m]')
        plt.title('X Position Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y position
        plt.subplot(2, 2, 3)
        plt.plot(self.time_log, self.yd_log, 'r--', label='Y desired')
        plt.plot(self.time_log, self.y_log, 'b-', label='Y actual')
        plt.xlabel('Time [s]')
        plt.ylabel('Y Position [m]')
        plt.title('Y Position Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot commands
        plt.subplot(2, 2, 4)
        plt.plot(self.time_log, self.v_log, 'g-', label='Linear vel')
        plt.plot(self.time_log, self.omega_log, 'm-', label='Angular vel')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity')
        plt.title('Control Commands')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        controller = TrajectoryFollowingController()
        rospy.spin()
        controller.plot_results()
    except rospy.ROSInterruptException:
        pass

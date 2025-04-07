#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class DiffDrive_Controller:
    def __init__(self):
        rospy.init_node('diff_drive_posture_regulation_control')
        
        # Parametri del controller (originali)
        self.k1 = rospy.get_param('~k1', 1.5)
        self.k2 = rospy.get_param('~k2', 2.5)
        self.k3 = rospy.get_param('~k3', 1.5)
        
        # Parametri traiettoria (come nella ref_trajectory originale)
        self.R = 3
        self.v_d_val = 0.5 # m/s
        self.w_d_val = self.v_d_val/self.R

        
        
        # Stato del robot
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0
        
        # Logging dati
        self.x_log = []
        self.y_log = []
        self.v_log = []
        self.omega_log = []
        self.x_goal_log = []
        self.y_goal_log = []
        self.time_log = []
        self.t_start = rospy.Time.now().to_sec()
        # ROS
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def ref_trajectory(self,t):

        
        xd = self.R * np.cos(self.w_d_val * t)-self.R
        yd = self.R * np.sin(self.w_d_val * t)
        
        return xd, yd

    def odom_callback(self, msg):
        """Callback esattamente identico all'originale"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)

    def posture_regulation_control(self, x_goal, y_goal):
        """Controller di postura IDENTICO all'originale"""
        xe = x_goal - self.x
        ye = y_goal - self.y
        
        rho = np.sqrt(xe**2 + ye**2)
        gamma = np.arctan2(ye, xe) - self.theta
        gamma = np.arctan2(np.sin(gamma), np.cos(gamma))  # Normalize
        delta = gamma + self.theta 
        
        v = self.k1 * rho * np.cos(gamma)
        if abs(gamma) > 0.001:
            omega = self.k2 * gamma + self.k1 * np.sin(gamma) * np.cos(gamma) * (1 + self.k3 * delta) / gamma
        else:
            omega = self.k2 * gamma + self.k1 * (1 + self.k3 * delta)
        
        return v, omega

    def control_loop(self, event):
        """Loop di controllo"""
       
        # Calcola tempo corrente
        t = rospy.Time.now().to_sec() - self.t_start
        # Ottieni le coordinate del punto goal corrente
        x_goal, y_goal = self.ref_trajectory(t)
        
        # Calcola i comandi di controllo USANDO L'ORIGINALE
        v, omega = self.posture_regulation_control(x_goal, y_goal)
        
        # Pubblica il comando
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = omega
        self.cmd_pub.publish(cmd_msg)
        
        # Log dei dati
        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.v_log.append(v)
        self.omega_log.append(omega)
        self.x_goal_log.append(x_goal)
        self.y_goal_log.append(y_goal)
        self.time_log.append(t)

    def plot_results(self):
        """Visualizzazione dei risultati"""
        plt.figure(figsize=(12, 8))
        
        # Plot traiettoria
        plt.subplot(2, 2, 1)
        plt.plot(self.x_goal_log, self.y_goal_log, 'r--', label='Desired')
        plt.plot(self.x_log, self.y_log, 'b-', label='Actual')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Trajectory Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y1 e y1d
        plt.subplot(2, 2, 2)
        plt.plot(self.time_log, self.x_goal_log, 'r--', label='x desired')
        plt.plot(self.time_log, self.x_log, 'b-', label='x actual')
        plt.xlabel('Time [s]')
        plt.ylabel('x [m]')
        plt.title('x Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y2 e y2d
        plt.subplot(2, 2, 3)
        plt.plot(self.time_log, self.y_goal_log, 'r--', label='y desired')
        plt.plot(self.time_log, self.y_log, 'b-', label='y actual')
        plt.xlabel('Time [s]')
        plt.ylabel('y [m]')
        plt.title('y Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot comandi
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
        controller = DiffDrive_Controller()
        rospy.spin()
        controller.plot_results()
    except rospy.ROSInterruptException:
        pass

# -*- coding: utf-8 -*-
"""
Created on Sat Apr  5 16:49:33 2025

@author: michela
"""



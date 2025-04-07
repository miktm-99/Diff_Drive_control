#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class DiffDriveController:
    def __init__(self):
        rospy.init_node('diff_drive_io_linearization')
        
        # Parametri del robot
        self.b = rospy.get_param('~b', 0.1)  # Distanza punto B dal centro

        
        # Parametri del controller
        self.k1 = rospy.get_param('~k1', 1.0)
        self.k2 = rospy.get_param('~k2', 1.0)
        
        # Stato del robot
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0
        self.y1 = 0.0
        self.y2 = 0.0
        
        # Traiettoria desiderata
        self.R = 3.0
        self.v_d_val=0.5
        self.w_d_val = self.v_d_val/self.R
        self.t_start = rospy.Time.now().to_sec()
        
        # Inizializza publisher e subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Dati per logging
        self.time_log = []
        self.y1_log = []
        self.y2_log = []
        self.x_log = []
        self.y_log = []
        self.y1d_log = []
        self.y2d_log = []
        self.v_log = []
        self.omega_log = []
        
        # Timer per controllo
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def odom_callback(self, msg):
        # Estrai posizione e orientamento
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Converti quaternione in angolo di Eulero (yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)
        
        # Calcola y1 e y2 (punto B davanti al robot)
        self.y1 = self.x + self.b * np.cos(self.theta)
        self.y2 = self.y + self.b * np.sin(self.theta)

    def ref_trajectory(self, t):
        y1d = self.R * np.cos(self.w_d_val * t)-self.R
        y2d = self.R * np.sin(self.w_d_val * t)
        doty1d = -self.R * self.w_d_val * np.sin(self.w_d_val * t)
        doty2d = self.R * self.w_d_val * np.cos(self.w_d_val * t)
        return y1d, y2d, doty1d, doty2d

    def control_law(self, y1, y2, y1d, y2d, doty1d, doty2d):
        u1 = doty1d + self.k1 * (y1d - y1)
        u2 = doty2d + self.k2 * (y2d - y2)
        return u1, u2

    def control_loop(self, event):
        # Calcola tempo corrente
        t = rospy.Time.now().to_sec() - self.t_start
        
        # Genera traiettoria desiderata
        y1d, y2d, doty1d, doty2d = self.ref_trajectory(t)
        
        # Calcola legge di controllo
        u1, u2 = self.control_law(self.y1, self.y2, y1d, y2d, doty1d, doty2d)
        
        # Converti in comandi v e omega
        v = np.cos(self.theta) * u1 + np.sin(self.theta) * u2
        omega = (-np.sin(self.theta) * u1 + np.cos(self.theta) * u2) / self.b
        
        # Pubblica comando
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = omega
        self.cmd_pub.publish(cmd_msg)
        
        # Logga dati
        self.time_log.append(t)
        self.y1_log.append(self.y1)
        self.y2_log.append(self.y2)
        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.y1d_log.append(y1d)
        self.y2d_log.append(y2d)
        self.v_log.append(v)
        self.omega_log.append(omega)

    def plot_results(self):
        plt.figure(figsize=(12, 8))
        
        # Plot traiettoria
        plt.subplot(2, 2, 1)
        plt.plot(self.y1d_log, self.y2d_log, 'r--', label='Desired')
        plt.plot(self.y1_log, self.y2_log, 'b-', label='Point B')
        plt.plot(self.x_log, self.y_log, 'g-', label='Actual')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Trajectory Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y1 e y1d
        plt.subplot(2, 2, 2)
        plt.plot(self.time_log, self.y1d_log, 'r--', label='y1 desired')
        plt.plot(self.time_log, self.y1_log, 'b-', label='y1 actual')
        plt.plot(self.time_log, self.x_log, 'g-', label='x actual')
        plt.xlabel('Time [s]')
        plt.ylabel('x [m]')
        plt.title('X Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y2 e y2d
        plt.subplot(2, 2, 3)
        plt.plot(self.time_log, self.y2d_log, 'r--', label='y2 desired')
        plt.plot(self.time_log, self.y2_log, 'b-', label='y2 actual')
        plt.plot(self.time_log, self.y_log, 'g-', label='y actual')
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
        controller = DiffDriveController()
        rospy.spin()
        
        # Quando il nodo viene fermato, plotta i risultati
        controller.plot_results()
        
    except rospy.ROSInterruptException:
        pass

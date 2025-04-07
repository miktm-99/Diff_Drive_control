#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PostureRegulationController:
    def __init__(self):
        rospy.init_node('posture_regulation_polar')
        
        # Parametri del controller
        self.k1 = rospy.get_param('~k1', 1.0)
        self.k2 = rospy.get_param('~k2', 2.0)
        self.k3 = rospy.get_param('~k3', -1.0)
        
        # Stato del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Parametri della traiettoria desiderata
        self.R = 3.0
        self.w_d = 1/15.0
        self.t_start = rospy.Time.now().to_sec()
        
        # Variabili per il logging
        self.x_log = []
        self.y_log = []
        self.x_goal_log = []
        self.y_goal_log = []
        self.v_log = []
        self.omega_log = []
        
        # Inizializza publisher e subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Timer per il controllo
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def odom_callback(self, msg):
        # Estrai posizione e orientamento
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Converti quaternione in angolo di Eulero (yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)
    
    def ref_trajectory(self, t):
        y1d = self.R * np.sin(2 * self.w_d * t)
        y2d = self.R * np.sin(self.w_d * t)
        return y1d, y2d
    
    def compute_polar_coordinates(self, x_goal, y_goal):
        c_theta = np.cos(self.theta)
        s_theta = np.sin(self.theta)
        A = np.array([[c_theta, s_theta, 0, -c_theta * x_goal - s_theta * y_goal],
                      [-s_theta, c_theta, 0, -c_theta * y_goal + s_theta * x_goal],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        p_prime = np.dot(A, np.array([[self.x], [self.y], [0], [1]]))
        x = float(p_prime[0][0])
        y = float(p_prime[1][0])
        theta = self.theta
        
        rho = np.sqrt(x**2 + y**2)
        gamma = np.arctan2(y, x) - theta + np.pi
        delta = gamma + theta
        
        return rho, gamma, delta
    
    def compute_control(self, x_goal, y_goal):
        rho, gamma, delta = self.compute_polar_coordinates(x_goal, y_goal)
        v = self.k1 * rho * np.cos(gamma)
        try:
            omega = self.k2 * gamma + self.k1 * np.sin(gamma) * np.cos(gamma) * (1 + self.k3 * delta / gamma)
        except ZeroDivisionError:
            omega = 0
        return v, omega
    
    def control_loop(self, event):
        # Calcola tempo corrente
        t = rospy.Time.now().to_sec() - self.t_start
        
        # Genera traiettoria desiderata
        x_goal, y_goal = self.ref_trajectory(t)
        
        # Calcola controlli
        v, omega = self.compute_control(x_goal, y_goal)
        
        # Pubblica comando
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = omega
        self.cmd_pub.publish(cmd_msg)
        
        # Salva dati per il plot
        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.x_goal_log.append(x_goal)
        self.y_goal_log.append(y_goal)
        self.v_log.append(v)
        self.omega_log.append(omega)
    
    def plot_results(self):
        plt.figure()
        plt.plot(self.x_goal_log, self.y_goal_log, 'r--', label='Traiettoria desiderata')
        plt.plot(self.x_log, self.y_log, 'b-', label='Traiettoria seguita')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Traiettoria del robot')
        plt.grid()
        
        plt.figure()
        plt.plot(self.v_log, label='Velocità lineare (v)')
        plt.plot(self.omega_log, label='Velocità angolare (omega)')
        plt.xlabel('Tempo (iterazioni)')
        plt.ylabel('Valore')
        plt.legend()
        plt.title('Velocità del robot')
        plt.grid()
        
        plt.show()
    
if __name__ == '__main__':
    try:
        controller = PostureRegulationController()
        rospy.spin()
        controller.plot_results()
    except rospy.ROSInterruptException:
        pass

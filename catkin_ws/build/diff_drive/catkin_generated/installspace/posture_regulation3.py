#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TrajectoryFollowingController:
    def __init__(self):
        rospy.init_node('trajectory_following_controller')
        
        # Parametri del controller (originali)
        self.k1 = rospy.get_param('~k1', 0.3)
        self.k2 = rospy.get_param('~k2', 0.4)
        self.k3 = rospy.get_param('~k3', 0.0)
        
        # Parametri traiettoria (come nella ref_trajectory originale)
        self.R = 3.0          # Ampiezza traiettoria
        self.w_d = 1/15.0     # Frequenza
        self.duration = 105.0   # Durata traiettoria (secondi)
        self.dt = 3.0         # Intervallo tra waypoint (secondi)
        
        # Genera waypoint automaticamente
        self.trajectory_points = self.generate_trajectory_points()
        self.current_point_idx = 0
        self.reached_threshold = 0.1  # Soglia per considerare raggiunto un punto
        
        # Stato del robot
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0
        
        # Logging dati
        self.x_log = []
        self.y_log = []
        self.x_goal_log = []
        self.y_goal_log = []
        
        # ROS
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def generate_trajectory_points(self):
        """Genera waypoint lungo la traiettoria desiderata"""
        points = []
        for t in np.arange(0, self.duration, self.dt):
            xd = self.R * np.sin(2 * self.w_d * t)
            yd = self.R * np.sin(self.w_d * t)
            points.append((xd, yd))
        return points

    def odom_callback(self, msg):
        """Callback esattamente identico all'originale"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)

    def get_current_goal(self):
        """Restituisce il punto corrente della traiettoria"""
        return self.trajectory_points[self.current_point_idx]

    def check_if_reached(self):
        """Verifica se il punto corrente è stato raggiunto"""
        x_goal, y_goal = self.get_current_goal()
        distance = np.sqrt((x_goal - self.x)**2 + (y_goal - self.y)**2)
        return distance < self.reached_threshold

    def advance_to_next_point(self):
        """Passa al punto successivo della traiettoria"""
        if self.current_point_idx < len(self.trajectory_points) - 1:
            self.current_point_idx += 1
            rospy.loginfo(f"Moving to point {self.current_point_idx}: {self.trajectory_points[self.current_point_idx]}")
        else:
            rospy.loginfo("Trajectory completed!")
            self.control_timer.shutdown()

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
        # Verifica se il punto corrente è stato raggiunto
        if self.check_if_reached():
            self.advance_to_next_point()
        
        # Ottieni le coordinate del punto goal corrente
        x_goal, y_goal = self.get_current_goal()
        
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
        self.x_goal_log.append(x_goal)
        self.y_goal_log.append(y_goal)

    def plot_trajectory(self):
        """Visualizzazione dei risultati"""
        plt.figure(figsize=(10, 8))
        
        # Traiettoria desiderata
        points = np.array(self.trajectory_points)
        plt.plot(points[:,0], points[:,1], 'ro-', markersize=4, label='Waypoints')
        
        # Percorso effettivo
        plt.plot(self.x_log, self.y_log, 'b-', linewidth=2, label='Actual Path')
        
        # Punti goal attivi
        plt.plot(self.x_goal_log, self.y_goal_log, 'g--', label='Active Goals')
        
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.title('Trajectory Following (Lemniscate)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

if __name__ == '__main__':
    try:
        controller = TrajectoryFollowingController()
        rospy.spin()
        controller.plot_trajectory()
    except rospy.ROSInterruptException:
        pass

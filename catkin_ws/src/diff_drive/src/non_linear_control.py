#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class DiffDriveController:
    def __init__(self):
        rospy.init_node('diff_drive_nonlinear_control')
        
        # Parametri del controller
        self.a = rospy.get_param('~a', 1.0)  # Distanza punto B dal centro
        self.zeta = rospy.get_param('~zeta', 0.7)
       
        
        # Stato del robot
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0
        
        
        # Traiettoria desiderata
        self.R = 3
        self.v_d_val = 0.5
        self.w_d_val = self.v_d_val/self.R
    
        self.t_start = rospy.Time.now().to_sec()
        
        # Inizializza publisher e subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Dati per logging
        self.time_log = []
        self.x_log = []
        self.y_log = []
        self.xd_log = []
        self.yd_log = []
        self.v_log = []
        self.omega_log = []
        self.e1_log = []
        self.e2_log = []
        self.e3_log = []

        
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

        #self.theta=np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
        
    def ref_trajectory(self, t):
        x_d = self.R * np.cos(self.w_d_val * t) - self.R
        y_d = self.R * np.sin(self.w_d_val * t)
        dotx_d = -self.R*self.w_d_val*np.sin(self.w_d_val*t) # time derivative of x_d
        doty_d =  self.R*self.w_d_val*np.cos(self.w_d_val*t) # time derivative of y_d
        
        v_d = np.sqrt(dotx_d**2 + doty_d**2)
        theta_d = np.arctan2(doty_d, dotx_d)
        #theta_d=np.arctan2(np.sin(theta_d), np.cos(theta_d))

        w_d =(t*0+1)*self.w_d_val
        return x_d, y_d, theta_d, v_d, w_d
    
    
    def k1_circ(self, v_d, w_d):
        
        return 2*self.zeta*self.a
        
    
    def k3_circ(self,v_d, w_d):
        
        return 2*self.zeta*self.a
        #return 1.0

    
    def control(self, e, v_d, w_d):
        k2 = (self.a**2 -w_d**2)/(v_d)
        
        u_1 = -self.k1_circ(v_d, w_d) * e[0]
        
        
        if e[2] == 0:
            u_2 = -k2 * v_d * e[1] - self.k3_circ(v_d, w_d) * e[2]
        else:
            u_2 = -k2 * v_d * np.sin(e[2]) / e[2] * e[1] - self.k3_circ(v_d, w_d) * e[2] 
      
        return u_1, u_2
    
    def compute_tracking_error(self, desired_pose):
        theta = self.theta
        R = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        
        current_pose = np.array([self.x, self.y, self.theta])
        
        pose_error = desired_pose - current_pose
        e = np.dot(R, pose_error)
        
        e[2]=np.arctan2(np.sin(e[2]), np.cos(e[2]))
        return e
    
    def control_loop(self, event):
        # Calcola tempo corrente
        t = rospy.Time.now().to_sec() - self.t_start
        
        # Genera traiettoria desiderata
        x_d, y_d, theta_d, v_d, w_d = self.ref_trajectory(t)
        
        
        #calcola errore
        desired_pose=np.array([x_d, y_d, theta_d])
   

        e=self.compute_tracking_error(desired_pose)
        
        # Calcola legge di controllo
        u1, u2 = self.control(e, v_d, w_d)
        
        # Converti in comandi v e omega
        v = v_d*np.cos(e[2])-u1
        w = w_d-u2
        
        # Pubblica comando
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = w
        self.cmd_pub.publish(cmd_msg)
        
        # Logga dati
        self.time_log.append(t)
        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.xd_log.append(x_d)
        self.yd_log.append(y_d)
        self.v_log.append(v)
        self.omega_log.append(w)
        self.e1_log.append(e[0])
        self.e2_log.append(e[1])
        self.e3_log.append(e[2])

    def plot_results(self):
        plt.figure(num=1,figsize=(12, 8))
        
        # Plot traiettoria
        plt.subplot(2, 2, 1)
        plt.plot(self.xd_log, self.yd_log, 'r--', label='Desired')
        plt.plot(self.x_log, self.y_log, 'b-', label='Actual')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Trajectory Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y1 e y1d
        plt.subplot(2, 2, 2)
        plt.plot(self.time_log, self.xd_log, 'r--', label='x desired')
        plt.plot(self.time_log, self.x_log, 'b-', label='x actual')
        plt.xlabel('Time [s]')
        plt.ylabel('x')
        plt.title('x Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot y2 e y2d
        plt.subplot(2, 2, 3)
        plt.plot(self.time_log, self.yd_log, 'r--', label='y desired')
        plt.plot(self.time_log, self.y_log, 'b-', label='y actual')
        plt.xlabel('Time [s]')
        plt.ylabel('y')
        plt.title('y Tracking')
        plt.legend()
        plt.grid(True)
        
        # Plot comandi
        plt.subplot(2, 2, 4)
        plt.plot(self.time_log, self.v_log, 'g-', label='Linear vel')
        plt.plot(self.time_log, self.omega_log, 'm-', label='Angular vel')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity')
        plt.legend()
        plt.title('Control Commands')
        plt.grid(True)



        plt.figure(num=2,figsize=(12, 8))
        plt.plot(self.time_log, self.e1_log, 'g-', label='Errore lungo x - relativo')
        plt.plot(self.time_log, self.e2_log, 'm-', label='Errore lungo y - relativo')
        plt.plot(self.time_log, self.e3_log, 'r-', label='Errore theta') 
        plt.xlabel('Time [s]')
        plt.ylabel('Errore')
        plt.legend()
        plt.title('Evoluzione errore')
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




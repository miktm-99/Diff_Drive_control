#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class LinearizationController:
    def __init__(self):
        rospy.init_node('linearization_controller', anonymous=True)
        
        # Publisher per inviare comandi di velocità
        self.cmd_pub = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)

        # Subscriber per leggere l'odometria
        self.odom_sub = rospy.Subscriber('/Diff_Drive/diff_drive_controller/odom', Odometry, self.odom_callback)

        # Stato attuale del robot
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.k_rho, self.k_alpha, self.k_beta = 1.0, 2.0, -1.0  # Guadagni

        # Obiettivo (da cambiare in base alla destinazione desiderata)
        self.x_goal, self.y_goal, self.theta_goal = 5.0, 5.0, 1.0  

    def odom_callback(self, msg):
        # Aggiorna lo stato attuale del robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = self.get_yaw(msg.pose.pose.orientation)
        
        # Applica la legge di controllo
        self.control_law()

    def control_law(self):
        ex = self.x_goal - self.x
        ey = self.y_goal - self.y
        rho = math.sqrt(ex**2 + ey**2)
        alpha = math.atan2(ey, ex) - self.theta
        beta = self.theta_goal - self.theta

        # Controllo basato sulla regolazione della postura polare
        v = self.k_rho * rho
        omega = self.k_alpha * alpha + self.k_beta * beta

        # Pubblica i comandi di velocità
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_pub.publish(twist)

    def get_yaw(self, orientation):
        """ Estrai yaw dall'orientazione quaternion. """
        import tf.transformations
        _, _, yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        return yaw

if __name__ == '__main__':
    try:
        LinearizationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sys
import numpy as np
from numpy import pi as PI
import math

EPSILON_theta = 0.15
EPSILON_d = 0.05
d_star_o = 1
d_star_g = 1.5
Ka = 400
Kr = 1
Kp = 5
Kd = 5
max_w = 2.5
max_v = 0.22
N_NBRS = 40
R_NBR = 0.1
STEP_THETA = 2 * PI/N_NBRS

def wrap_angle(psi):
    return (psi + PI) % (2 * PI) - PI

def quat_to_eul(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class Planner:
    def __init__(self, x, y):
        rospy.init_node('dynamic_obstacle_avoidance')
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(60)  # 10 Hz
        self.move_cmd = Twist()
        self.x_goal = x 
        self.y_goal = y
        self.x = None
        self.y = None
        self.psi = None
        self.scan_data = None
        self.v_attract = np.zeros(2)
        self.v_repel = np.zeros(2)
        self.last_delta = 0
        self.nbr_potentials = np.zeros(N_NBRS)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.psi = quat_to_eul(quat)

    def laser_callback(self, scan_msg):
        self.scan_data = scan_msg

    def controller(self):
        theta = wrap_angle(self.psi + STEP_THETA * np.argmin(self.nbr_potentials))
        print(np.argmin(self.nbr_potentials))
        delta = wrap_angle(theta - self.psi)
        d = (self.x_goal - self.x)**2 + (self.y_goal - self.y)**2
        steer = Kp * delta + Kd * (delta - self.last_delta)
        steer = np.clip(steer,-max_w, max_w)
        # print(steer)
        # print(theta)
        self.last_delta = delta
        if delta > EPSILON_theta :
            self.move_cmd.angular.z = steer
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0
        elif delta < -EPSILON_theta :
            self.move_cmd.angular.z = steer # - 0.6
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0
        else :
            self.move_cmd.angular.z = delta * 0.15
            self.move_cmd.linear.x = max_v
            self.move_cmd.linear.y = 0 #min(math.sin(delta), math.sin(delta)) 

        if math.sqrt(d) < EPSILON_d :
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0

    def generate_attraction(self):
        for i in range(N_NBRS):
            alpha = self.psi + i * STEP_THETA
            x,y = self.x + R_NBR * math.cos(alpha), self.y + R_NBR * math.sin(alpha)
            u_a = ((self.x_goal - x)**2 + (self.y_goal - y)**2) * Ka
            self.nbr_potentials[i] = u_a
        

    def generate_repulsion(self):
        scan_data = self.scan_data
        psi = self.psi
        angle_min = scan_data.angle_min
        step = scan_data.angle_increment
        scan = scan_data.ranges
        N = len(scan)
        count = 0
        x_r, y_r = 0.0, 0.0
        real_min = 1000
        for i in range(N):
            d = scan[i] - 0.12
            if d < d_star_o and scan[i] > 0.12 :
                beta = wrap_angle(angle_min + psi + step * i)
                for j in range(N_NBRS):
                    theta = wrap_angle(psi + STEP_THETA * j)
                    alpha = wrap_angle(beta - theta)
                    l = math.sqrt(d**2 + R_NBR**2 - 2 * d * R_NBR * math.cos(alpha))
                    repulsion = Kr * ((1/l) - (1/d_star_o)) **2
                    self.nbr_potentials[j] += repulsion
            else :
                count += 1
        
        if count == 360 :
            return

    def planner(self):
        if self.scan_data is None :
            return
        if self.x is None :
            return
        self.generate_attraction()
        self.generate_repulsion()
        self.controller()

    def run(self):
        while not rospy.is_shutdown():
            self.planner()  # Call the obstacle avoidance function
            self.velocity_publisher.publish(self.move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])

        Planner = Planner(x_goal, y_goal)
        Planner.run()
    except rospy.ROSInterruptException:
        pass
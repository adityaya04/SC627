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
        self.rate = rospy.Rate(30)  # 10 Hz
        self.move_cmd = Twist()
        self.x_goal = x 
        self.y_goal = y
        self.x = None
        self.y = None
        self.psi = None
        self.scan_data = None
        self.v_attract = np.zeros(2)
        self.v_repel = np.zeros(2)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.psi = quat_to_eul(quat)

    def laser_callback(self, scan_msg):
        self.scan_data = scan_msg

    def controller(self):
        Vf = self.v_attract + self.v_repel
        theta = math.atan2(Vf[1], Vf[0])
        delta = (theta - self.psi + PI) % (2*PI) - PI
        d = (self.x_goal - self.x)**2 + (self.y_goal - self.y)**2
        # print(np.linalg.norm(self.v_attract), np.linalg.norm(self.v_repel))
        print(delta)

        if delta > EPSILON_theta :
            self.move_cmd.angular.z = 1
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0
        elif delta < -EPSILON_theta :
            self.move_cmd.angular.z = -1
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0
        else :
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x = min(math.cos(delta), math.cos(delta)) * 3
            self.move_cmd.linear.y = min(math.sin(delta), math.sin(delta)) * 3

        if math.sqrt(d) < EPSILON_d :
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0

    def generate_attraction(self):
        d = (self.x_goal - self.x)**2 + (self.y_goal - self.y)**2
        self.v_attract[0] = (self.x_goal - self.x) * 40
        self.v_attract[1] = (self.y_goal - self.y) * 40

    def generate_repulsion(self):
        scan_data = self.scan_data
        psi = self.psi
        angle_min = scan_data.angle_min
        step = scan_data.angle_increment
        scan = scan_data.ranges
        N = len(scan)
        count = 0
        x_r, y_r = 0.0, 0.0
        for i in range(N):
            if scan[i] < 100 and scan[i] > 0.1 :
                repulsion = 0.2 / scan[i]
                x_r -= repulsion * math.cos(angle_min + psi + step*i)
                y_r -= repulsion * math.sin(angle_min + psi + step*i)
            else :
                count += 1
        
        if count == 360 :
            self.v_repel = np.zeros(2)
        else :
            self.v_repel = np.array([x_r, y_r])

    def planner(self):
        if self.scan_data is None :
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

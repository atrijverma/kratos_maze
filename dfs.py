#!/usr/bin/env python3

import rclpy
from rclpy.node import Node                     #THIS ONE WENT THE FURTHEST 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        #self.timer1 = self.create_timer(0.1, self.move_forward)
        self.twist = Twist()
        self.position = (0, 0)
        self.stack = []
        self.visited =set()
        self.regions = {
            "right": float("inf"),
            "front": float("inf"),
            "left": float("inf"),
            "back": float("inf"),
            "rightfront": float("inf"),
            "leftfront": float("inf"),
            
        }
        self.state = "inside_maze" 
        
        self.wall_found_side = None  # 'left' or 'right'
        self.node =[]

    def odom_callback(self, msg):
        x = round(msg.pose.pose.position.x, 1)
        y = round(msg.pose.pose.position.y, 1)
        self.position = (x, y)

    def scan_callback(self, msg):
        # LaserScan.ranges has clockwise 360 readings from 0 (front) to 359 (just before front again)
        ranges = msg.ranges

        self.regions = {
            "right": min(min(ranges[273:276]), 10.0),
            "front": min(min(ranges[0:10] + ranges[350:360]), 10.0),
            "left": min(min(ranges[85:88]), 10.0),
            "back": min(min(ranges[179:182]), 10.0),
            "rightfront": min(min(ranges[295:300]),10.0),
            "leftfront": min(min(ranges[55:60]),10.0),
        }

    def control_loop(self):
        # get current readings
        front = self.regions["front"]
        left = self.regions["left"]
        right = self.regions["right"]
        back = self.regions["back"]
        rightfront = self.regions["rightfront"]
        leftfront = self.regions["leftfront"]
        if (right > 7 and left > 7):
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.update_position()
            time.sleep(0.6)

        elif self.state =="inside_maze":
            
            if(left > 1.6 and right > 1.6 and back > 1.5) or(left >1.6 and front >1.6)or(right > 1.6 and front >1.6)or( front >1.6 and rightfront > 1.5)or(leftfront > 1.4 and front > 1.5): # 1.6 ALSO WORKS FINE WITH VEL LINEAR 0.23
                self.get_logger().info("node detected")
                if self.position not in self.visited:
                    self.visited.add(self.position)
                    self.stack.append(self.position)
                
                if(left > 1.5):
                    self.turn_left()
                    self.update_position()
                    self.wall_found_side = "left"
                    time.sleep(0.5)
                    #self.move_forward()
                elif(rightfront > 1.5):
                    self.turn_right()
                    self.update_position()
                    self.wall_found_side = "right"
                    time.sleep(0.5)
                    #self.move_forward()
                elif(front > 1.0):
                    self.move_forward()
                    
                elif (left < 1.0 and right < 1.0 and front < 1.5):
                    self.turn_around()
                    time.sleep(1.5)
                    self.update_position()
            else:
                if self.wall_found_side =="left":
                    self.get_logger().info("following LEFT wall")
                    if (self.regions["front"] < 0.55):
                        self.get_logger().info("wall ahead")
                        if(left == right or abs(left - right) < 0.05):
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = 0.4
                            time.sleep(0.2)
                        elif(left >right):
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = 0.4
                        elif(right > left):
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = -0.4
                        

                    elif (self.regions["left"]> 0.5):
                        self.twist.linear.x = 0.23    #OR 0.23 WORKS FINE TOO
                        self.twist.angular.z = 0.23
                    elif (self.regions["left"] < 0.4):
                        self.twist.linear.x = 0.23    #OR 0.23 WORKS FINE TOO
                        self.twist.angular.z = -0.23
                    else:
                        self.get_logger().info("going straight")
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                elif self.wall_found_side == "right":
                    self.get_logger().info("following RIGHT wall")
                    if (self.regions["front"] < 0.55):
                        self.get_logger().info("wall ahead")
                        if(left == right or abs(left - right) < 0.05):
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = -0.4
                            time.sleep(0.2)
                        elif(left >right):
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = 0.4
                        elif(right > left):
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = -0.4
                        

                    elif (self.regions["right"]> 0.5):
                        self.twist.linear.x = 0.23   #OR 0.23 WORKS FINE TOO
                        self.twist.angular.z = -0.23
                    elif (self.regions["right"] < 0.4):
                        self.twist.linear.x = 0.23   #OR 0.23 WORKS FINE TOO
                        self.twist.angular.z = 0.23
                    else:
                        self.get_logger().info("going straight")
                        self.twist.linear.x = 0.15
                        self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.twist.linear.x = 0.15
                    self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                #self.wall_found_side = None

    
    def update_position(self ):
        x, y = self.position
        if self.regions== 'left':
            self.position = (x - 1, y)
            time.sleep(0.5)
        elif self.regions == 'right':
            self.position = (x + 1, y)
            time.sleep(0.5)

    def move_forward(self):
        self.get_logger().info("moving forward")
        if(self.regions["left"] < self.regions["right"]):
            self.wall_found_side =="right"
        elif(self.regions["left"] > self.regions["right"]):
            self.wall_found_side =="left"    
        if self.wall_found_side =="left":
            if (self.regions["front"] < 0.4):
                self.get_logger().info("wall ahead")
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.4
            elif (self.regions["left"]> 0.45):
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0.2
            elif (self.regions["left"] < 0.45):
                self.twist.linear.x = 0.15
                self.twist.angular.z = -0.2
            else:
                self.get_logger().info("going straight")
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
        elif self.wall_found_side == "right":
            if (self.regions["front"] < 0.4):
                self.get_logger().info("wall ahead")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.4
            elif (self.regions["right"]> 0.45):
                self.twist.linear.x = 0.15
                self.twist.angular.z = -0.2
            elif (self.regions["right"] < 0.45):
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0.2
            else:
                self.get_logger().info("going straight")
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0.15
            self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)
        # if (self.regions["front"] < 0.7 and self.regions["left"]> 1.5):
        #     self.twist.linear.x = min(self.regions["front"],0.15)
        #     self.turn_left()
        # elif(self.regions["front"] < 0.7 and self.regions["right"]> 1.5):
        #     self.twist.linear.x = min(self.regions["front"],0.15)
        #     self.turn_right()
        # else:
        #     self.twist.linear.x = 0.15
        # self.cmd_vel_pub.publish(self.twist)
        # if(self.regions["left"]< 0.3):
        #     self.turn_right()
        # elif(self.regions["right"] < 0.3):
        #     self.turn_left()
        # self.cmd_vel_pub.publish(self.twist)
        

    def turn_left(self):
        self.get_logger().info("turning left")
        self.twist.linear.x = 0.17
        self.twist.angular.z = 0.32
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(0.8)
        

    def turn_right(self):
        self.get_logger().info("turning right")
        self.twist.linear.x = 0.17
        self.twist.angular.z = -0.32
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(0.8)
        

    def turn_around(self):
        self.get_logger().info("turning around")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.5
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(1.0)
    def stop(self):
        self.get_logger().info("stopping ")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
def main(args=None):
   
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

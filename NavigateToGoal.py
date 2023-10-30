#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import math

MAX_SPEED = 0.2
GOAL_TOLERANCE = 0.1
PROPORTIANAL_COEFFICIENT = 0.5


class NavigateToGoal:
    def __init__(self):
        rospy.init_node('navigate_to_goal', anonymous=True)
        
        # set up a Twist publisher for velocity commands of robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # set up a Bool publisher for status of if robot is at goal
        self.goal_status_pub = rospy.Publisher('/goal_status', Bool, queue_size=0)
        
        # set up subscriber to get robot's current position and orientation
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        
        # set up subscriber to get robot's current goal
        rospy.Subscriber('/goal', Point, self.goal_callback)
        
        # initialize points
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.goal = Point()
        self.done = False
        
        # default desired goal
        # self.goal.x = 4.0  
        # self.goal.y = 0.0

        # control rate
        self.rate = rospy.Rate(10)  


    def goal_callback(self, msg):
        # update robots goal
        rospy.loginfo(f"New goal recieved! ({self.goal.x}, {self.goal.y})")
        self.goal = msg


    def odometry_callback(self, msg:Odometry):
        # Update the robot's current position and orientation
        self.current_pose = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.current_yaw = euler_from_quaternion(quaternion)


    def get_angle_to_goal(self) -> float:
        delta_x = self.goal.x - self.current_pose.x
        delta_y = self.goal.y - self.current_pose.y
        angle = math.atan2(delta_y, delta_x)
        return angle
            
            
    def get_angle_error(self, angle) -> float:      
        error = angle - self.current_yaw # is bounded between [-2pi, 2pi]
        return (error + math.pi) % (2*math.pi) - math.pi # bounds error between [-pi, pi]
    
    
    def navigate(self):
        rospy.loginfo("Node is running")
        
        while not rospy.is_shutdown():

            # calculate distance to goal
            distance_to_goal = math.sqrt((self.goal.x - self.current_pose.x) ** 2 + (self.goal.y - self.current_pose.y) ** 2)
            
            if distance_to_goal < GOAL_TOLERANCE:
                # if robot is at goal
                rospy.loginfo(f"Reached the goal ({self.goal.x}, {self.goal.y})!")
                
                # send move command with 0 values 
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                
                # update status paramater to true
                done_status = True
                
            else:
                # update status paramater to false 
                done_status = False
                
                # calculate angle to goal
                angle_to_goal = self.get_angle_to_goal()
                
                # calculate angle error 
                angle_error = self.get_angle_error(angle_to_goal)
                
                # Define Twist message (cmd) and populate with necessary values
                cmd = Twist()


                #-- adjust angular velocity to face goal --#
                cmd.angular.z = PROPORTIANAL_COEFFICIENT * angle_error 
                
                
                #-- adjust linear velocity to move towards goal --#
                
                # the if statement here allows the 
                # if abs(angle_error) > math.pi / 3:
                #     linear_speed = 0
                # else:
                # the min function slows it down as it approaches goal
                linear_speed = min(MAX_SPEED, distance_to_goal)
                
                cmd.linear.x = linear_speed  

                # publish command msg
                self.cmd_vel_pub.publish(cmd)

            
            # publish goal status
            status = Bool(done_status)
            self.goal_status_pub.publish(status)
            
            # sleep rate
            self.rate.sleep()

                
            
if __name__ == '__main__':
    navigator = NavigateToGoal()
    navigator.navigate()

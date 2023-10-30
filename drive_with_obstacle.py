#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import math

PROPORTIONAL_COEFFICENT = .5
ANGLE_TOLERANCE = 0.1

OBSTACLE_POSITION = Point(3, 3, 0)
safety_radius = 1

class DriveWithObstacle:
    def __init__(self) -> None:
        rospy.init_node('avoid_goal', anonymous=True)
        
        # set up a Twist publisher for velocity commands of robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # set up a Bool publisher for status of if robot is at goal
        self.goal_status_pub = rospy.Publisher('/goal_status', Bool, queue_size=0)
        
        # set up subscriber to get robot's current position and orientation
        rospy.Subscriber('/odom', Odometry, self.odometry_callback, queue_size=1)
        
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.goal = Point()
        
        self.linear_velocity = 0.2
        
        self.rate = rospy.Rate(10)  # 10Hz
        
        rospy.sleep(0.5)
        
        self.goal.x = 8
        self.goal.y = 8
        
        rospy.loginfo("Node is running")
        
        
    def odometry_callback(self, msg:Odometry) -> None:
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
        angle_to_goal = math.atan2(delta_y, delta_x)
        return angle_to_goal

    def get_angle_error(self, angle) -> float:      
        error = angle - self.current_yaw # is bounded between [-2pi, 2pi]
        return (error + math.pi) % (2*math.pi) - math.pi # bounds error between [-pi, pi]

    def get_distance(self, goal):
        return math.sqrt(
            (goal.x - self.current_pose.x) ** 2 + 
            (goal.y - self.current_pose.y) ** 2
        )

    def navigate(self) -> None:
        rospy.loginfo("Node is navigating")
        while not rospy.is_shutdown():
            # to update goal_status
            done_status = False
            
            # calculate distance to goal
            distance_to_goal = self.get_distance(self.goal)
            
            # calculate angle to goal
            angle_to_goal = self.get_angle_to_goal()
            
            # calculate angle error 
            angle_error = self.get_angle_error(angle_to_goal)

            # Check if robot is oriented toward the goal
            if (self.get_distance(OBSTACLE_POSITION) < safety_radius):
                self.stop_robot()
                break
            if (abs(angle_error) > ANGLE_TOLERANCE) :
                # Rotate the robot to face the goal
                cmd = Twist()
                print(f"angle: {angle_error} = {angle_to_goal} - {self.current_yaw}")
                # direction = 1 if angle_error > 0 else -1
                cmd.angular.z = PROPORTIONAL_COEFFICENT * angle_error
            elif (distance_to_goal > 0.1):
                # move the robot forward towards the goal
                cmd = Twist()
                cmd.linear.x = self.linear_velocity
            else:
                self.stop_robot()
                break
                
            # publish cmd
            self.cmd_vel_pub.publish(cmd)

            # publish goal status
            done_status = Bool(done_status)
            self.goal_status_pub.publish(done_status)
            
            self.rate.sleep()


    def compute_and_drive_angular(self, radius, theta):
        rospy.loginfo("Node is calculating angular")
        # Calculate angular velocity based on radius and linear velocity
        angular_velocity = self.linear_velocity / radius
        
        self.drive_angular(theta, angular_velocity, self.linear_velocity)
    
    
    def drive_angular(self, theta, angular_velocity, linear_velocity):
        rospy.loginfo("Node is driving angular")
        theta = math.radians(theta)
        
        # Calculate the total time needed to complete the arc
        total_time = abs(theta / angular_velocity)
        
        print("rotate time", total_time)
        
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start_time) < total_time:
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
        self.stop_robot()
        
        
    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)
        
        
if __name__ == '__main__':
    navigator = DriveWithObstacle()
    navigator.navigate()
    
    navigator.drive_angular(theta=90, angular_velocity=-0.1, linear_velocity=0)
    
    navigator.compute_and_drive_angular(radius=safety_radius, theta=180)
    
    navigator.navigate()

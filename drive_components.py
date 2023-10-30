import rospy
from geometry_msgs.msg import Twist
import math

class DriveComponents:
    def __init__(self, linear_velocity):
        rospy.init_node('drive_components', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
        self.linear_velocity = linear_velocity
        
        
        
    def drive_straight(self, duration):
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start_time) < duration:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity 
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
        


    def drive_circular(self, radius, theta):
        # Calculate angular velocity based on radius and linear velocity
        angular_velocity = self.linear_velocity  / radius
        
        # Calculate the total time needed to complete the arc
        total_time = abs(theta / angular_velocity)
        
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start_time) < total_time:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity 
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
        


    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)


if __name__ == '__main__':
    try:
        controller = DriveComponents(linear_velocity=0.2)
        
        # Example 1: Drive straight for 5 seconds
        controller.drive_straight(duration=3)
        
        # Example 2: Drive in a 1-meter radius circular arc for 90 degrees at 0.2 m/s
        controller.drive_circular(radius=2.0, theta= math.radians(90))
        
        # Stop the robot
        controller.stop_robot()
        
    except rospy.ROSInterruptException:
        pass

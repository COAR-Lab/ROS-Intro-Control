import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import numpy as np

# constants
CONTINOUS_TOTAL = 3

total_pts = 20
# Goal points list
x_goal = np.linspace(4, 8, total_pts)
y_goal = np.linspace(2, 7, total_pts)
goal_pts = 

# status variable
goal_status = False


# callback function
def goal_status_callback(msg : Bool):
    global goal_status
    goal_status = msg.data


# Initialize for the publisher node
rospy.init_node("Navigate_to_Goal")


# publisher for the the /goal topic
goal_pub = rospy.Publisher("/goal",Point,queue_size=0)


# subscriber to the /goal_status topic
goal_sub = rospy.Subscriber("/goal_status",Bool,goal_status_callback,queue_size=1)


# rate of node
rate = rospy.Rate(5)  


stayed_in_goal = 0
rospy.loginfo("Node is running")


while (not rospy.is_shutdown()) & (goal_pts != []):
    if goal_status:
        stayed_in_goal += 1
    else:
        stayed_in_goal = 0    
    
        
    if stayed_in_goal > CONTINOUS_TOTAL:
        # Goal point message to be sent
        goal_msg = Point()
        
        # Constructing goal msg to send
        goal = goal_pts.pop(0)
        goal_msg.x = goal[0]
        goal_msg.y = goal[1]
        
        # Publish the message.
        goal_pub.publish(goal_msg)
        rate.sleep()
        
    rate.sleep()
    
rospy.loginfo("Finished publishing all goal points")

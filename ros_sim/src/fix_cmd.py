#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    # Create a new message to send to the robot
    new_msg = Twist()
    
    # --- THE HACK ---
    # Map the Keyboard "Forward" (x) to the Robot's "Left" (y)
    # If 'i' moves you Right instead of Left, change this to: -msg.linear.x
    new_msg.linear.y = msg.linear.x  
    
    # Map the Keyboard "Sideways" (y) to the Robot's "Forward" (x)
    new_msg.linear.x = -msg.linear.y 
    
    # Keep rotation the same
    new_msg.angular.z = msg.angular.z

    # Send it to the simulation
    pub.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_fixer')
    
    # Listen to the keyboard (we remap this in the launch file)
    rospy.Subscriber('key_vel', Twist, callback)
    
    # Talk to the simulation
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    rospy.spin()
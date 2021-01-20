#!/usr/bin/env python
import rospy

import sys  
import math 
import tf
import numpy as np
from collections import deque
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
# topic definition
TOPIC_ODOM = 'odometry/filtered'
TOPIC_CTRL = 'mavros/rc/override'


# global config
noDOF = 4 # no DOF for AUV
DEFAULT_RATE = 70


class Controller(object):
    """Class to implement the sequential predictors controller."""

    def __init__(self, name):
        """Class constructor."""
        # Init all variables
       
        # Current robot position  (not sure if you want them separate or as an array)
        T_sim = 40
        

        
        
        # Control output as a vector
        

        # throttle values to send to the thrusters
        self.throttle = np.zeros(8)

        #
        self.delay = 1
        self.velocity = np.zeros(4)
        

        self.s = 70



        self.h0 = 0


        # Define pulisher: topic name, message type
        self.pub = rospy.Publisher('/BlueRov2/servo1/set_pwm', TwistStamped, queue_size = 20)
        # Define subscriber: topic name, message type, function callback
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_callback)


       

    def joy_callback(self, msg):
        """Saves the AUV position when a message is received."""
        temp = np.array(msg.axes)
        self.velocity = np.array([temp[0],temp[1],temp[4],temp[5]])/5.

      




    def send_ctrl(self):
        """Send the values to the thrusters.""" #TODO: This will have to be modified
        msg = TwistStamped()
        self.force_to_throttle()
        self.throttle = self.throttle*200. + 1500.
       



        msg.twist.linear.x = self.velocity[0]
        msg.twist.linear.y = self.velocity[1]
        msg.twist.linear.z = self.velocity[2]
        msg.twist.angular.z = self.velocity[3]

        self.pub.publish(msg)





    def force_to_throttle(self):
        """Convert from forces/torques in body coordinates to thrust for the 8 motors."""
        T = np.array([[0.707,  0.707, -0.707, -0.707,  0,     0,     0,     0],
              [-0.707, 0.707, -0.707,  0.707,  0,     0,     0,     0],
                      [ 0,     0,      0,      0,     -1,     1,     1,    -1],
                      [0.06,  -0.06,   0.06, -0.06,  -0.218, -0.218, 0.218, 0.218],
              [0.06,   0.06,  -0.06, -0.06,   0.120, -0.120, 0.120, -0.120],
              [-0.1888, 0.1888, 0.1888,  -0.1888,  0,     0,     0,     0]    
                                                                 ])
        self.throttle = np.dot(np.linalg.pinv(T),self.velocity) #TODO modify this accordingly



    def joy_ctrl(self):
        """Here is the implementation of the controller."""
        
        # while not rospy.is_shutdown():

        print('self.velocity', self.velocity)
        self.send_ctrl()
        

def main():
    
    # ROS init
    rospy.init_node('joy_controller')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)
    
    # load controller params
    # config = rospy.get_param('controller')
    verbose = False

    # parse args
    args = rospy.myargv()

    if '-v' in args:
        verbose = True

    rate = rospy.Rate(100)
    # start controller
    vc = Controller(name)
    

    # execute the controller while ros core is active    
    while not rospy.is_shutdown():
            try:
                vc.joy_ctrl()
                rate.sleep()
            except rospy.ROSInterruptException:
                    rospy.loginfo('%s shutdown requested ...', name)



if __name__ == '__main__':
    main()
    
   
        



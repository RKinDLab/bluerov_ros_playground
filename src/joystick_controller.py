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
from std_msgs.msg import UInt16, Bool, String

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

        self.joy_control = 0

        self.h0 = 0


        # Define pulisher: topic name, message type

        
        self.joy_l1 = 0
        self.joy_r1 = 0  

       

        self.pub_pitch = rospy.Publisher('/BlueRov2/rc_channel1/set_pwm', UInt16, queue_size = 10)
        self.pub_roll = rospy.Publisher('/BlueRov2/rc_channel2/set_pwm', UInt16, queue_size = 10)
        self.pub_vz = rospy.Publisher('/BlueRov2/rc_channel3/set_pwm', UInt16, queue_size = 10)
        self.pub_yaw = rospy.Publisher('/BlueRov2/rc_channel4/set_pwm', UInt16, queue_size = 10)
        self.pub_vx = rospy.Publisher('/BlueRov2/rc_channel5/set_pwm', UInt16, queue_size = 10)
        self.pub_vy = rospy.Publisher('/BlueRov2/rc_channel6/set_pwm', UInt16, queue_size = 10)
        self.pub_arm = rospy.Publisher('/BlueRov2/arm', Bool, queue_size = 10)
        self.pub_manual = rospy.Publisher('/BlueRov2/mode/set', String, queue_size = 10)
        
        # Define subscriber: topic name, message type, function callback
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_callback)


       

    def joy_callback(self, msg):
        """Saves the AUV position when a message is received."""
        temp = np.array(msg.axes)
        self.velocity = np.array([-temp[0]/3.,temp[1]/5.,temp[3]/5.,temp[4]/5.])
        # self.velocity = np.array(-[temp[0]/3.,temp[1]/3.,temp[3]/5.,temp[4]/4. - 0.15])
        # self.velocity = np.array([-temp[0]/3.,temp[1]/4.,temp[3]/4.,temp[4]/5. - 0.15])
        temp = np.array(msg.buttons)
        self.joy_control = temp[0]

        self.joy_l1 = temp[4]
        self.joy_r1 = temp[5]




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




    def send_command(self,command):
        print('commands vx, vy, vyaw', round(command[1],3),round(command[0],3), round(command[2],3), round(command[3],3))
        msg = UInt16()
        # msg.header.stamp = rospy.Time.now()
        msg.data = command[1]
        self.pub_vx.publish(msg.data)
        msg.data = command[0]
        self.pub_vy.publish(msg.data)
        msg.data = command[3]
        self.pub_vz.publish(msg.data)
        msg.data = np.uint16(1500)
        self.pub_roll.publish(msg.data)
        msg.data = np.uint16(1500)
        self.pub_pitch.publish(msg.data)
        msg.data = command[2]
        self.pub_yaw.publish(msg.data)

        # ,self.pub2,self.pub3,self.pub4,self.pub5,self.pub6,self.pub7,self.pub8

    def arm_mother(self):
        msg = String()
        msg.data = "manual"
        self.pub_manual.publish(msg)
        msg = Bool()
        msg.data = True
        self.pub_arm.publish(msg)




    def joy_ctrl(self):
        """Here is the implementation of the controller."""
        
        # while not rospy.is_shutdown():

        
         # Get joystick data
        
        # print('joy', joy, joy[0])
        if self.joy_l1 == 1 and self.joy_r1 == 1:  
            
            #joy = self.sub.get_data()['joy']['axes']
            # print('joy axes', self.velocity)
            self.send_command(np.uint16(self.velocity*300 + 1500))
            # rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
            #override = [int(val*200 + 1500) for val in joy]
            #for _ in range(len(override), 8):
            #    override.append(0)
            # Send joystick data as rc output into rc override topic
            # (fake radio controller)
            #self.pub.set_data('/mavros/rc/override', override)
        if self.joy_control == 1:
            self.arm_mother()



        #self.send_ctrl()
        

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

    rate = rospy.Rate(10)
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
    
   
        



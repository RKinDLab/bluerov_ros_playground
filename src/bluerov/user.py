#!/usr/bin/env python

import cv2
import rospy
import time

try:
    import pubs
    import subs
    import video
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs
    import bluerov.video as video

from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import JointState, Joy

from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut
from nav_msgs.msg import Odometry

class Code(object):

    """Class to provide user access

    Attributes:
        cam (Video): Video object, get video stream
        pub (Pub): Pub object, do topics publication
        sub (Sub): Sub object, subscribe in topics
    """

    def __init__(self):
        super(Code, self).__init__()

        # Do what is necessary to start the process
        # and to leave gloriously
        self.arm()

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()

        self.pub.subscribe_topic('/mavros/rc/override', OverrideRCIn)
        self.pub.subscribe_topic('/mavros/setpoint_velocity/cmd_vel', TwistStamped)
        self.pub.subscribe_topic('/BlueRov2/body_command', JointState)
        self.pub.subscribe_topic('/BlueRov2/imu/data', Imu)
        self.pub.subscribe_topic('/BlueRov2/depth', Odometry)

        self.sub.subscribe_topic('/joy', Joy)
        self.sub.subscribe_topic('/mavros/battery', BatteryState)
        self.sub.subscribe_topic('/mavros/rc/in', RCIn)
        self.sub.subscribe_topic('/mavros/rc/out', RCOut)
        self.sub.subscribe_topic('/mavros/imu/data', Imu)

        self.ROV_name = 'BlueRov2'
        self.model_base_link = 'base_link'
        self.water_type = rospy.get_param('/user_node/density_water')

        self.cam = None
        try:
            video_udp_port = rospy.get_param("/user_node/video_udp_port")
            rospy.loginfo("video_udp_port: {}".format(video_udp_port))
            self.cam = video.Video(video_udp_port)
        except Exception as error:
            rospy.loginfo(error)
            self.cam = video.Video()


    def arm(self):
        """ Arm the vehicle and trigger the disarm
        """
        rospy.wait_for_service('/mavros/cmd/arming')

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm_service(True)

        # Disarm is necessary when shutting down
        rospy.on_shutdown(self.disarm)

    def _create_depth_msg(self):
        if 'SCALED_PRESSURE2' not in self.get_data():
            raise Exception('no Bar30 depth')

        msg = Odometry()
        self._create_header(msg)
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link' 

        depth_data = self.get_data()['SCALED_PRESSURE2']
        #  'press_diff': -12.65999984741211, 'time_boot_ms': 8357069, 'temperature': 2462, 'press_abs': 1022.0 
        # preassure comes in hPA apparently (1022.0 )
        pressure = depth_data['press_abs']
        temperature = depth_data['temperature']
        # need to convert
        # https://github.com/bluerobotics/ms5837-python    
        depth = (100.*pressure-101300)/(self.water_type*9.80665)
        # print('altitude', (1-pow((pressure/1013.25),.190284))*145366.45*.3048 )
        
        msg.pose.pose.position.z = depth
        self.pub.set_data('/depth', msg)

    def _create_header(self, msg,link=None):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    @staticmethod
    def pwm_to_thrust(pwm):
        """Transform pwm to thruster value
        The equation come from:
            https://colab.research.google.com/notebook#fileId=1CEDW9ONTJ8Aik-HVsqck8Y_EcHYLg0zK

        Args:
            pwm (int): pwm value

        Returns:
            float: Thrust value
        """
        return -3.04338931856672e-13*pwm**5 \
            + 2.27813523978448e-9*pwm**4 \
            - 6.73710647138884e-6*pwm**3 \
            + 0.00983670053385902*pwm**2 \
            - 7.08023833982539*pwm \
            + 2003.55692021905


    def run(self):
        """Run user code
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            # Try to get data
            '''
            try:
                rospy.loginfo(self.sub.get_data()['mavros']['battery']['voltage'])
                rospy.loginfo(self.sub.get_data()['mavros']['rc']['in']['channels'])
                rospy.loginfo(self.sub.get_data()['mavros']['rc']['out']['channels'])
            except Exception as error:
                print('Get data error:', error)
            '''
            # self._create_depth_msg()
            try:
                # Get joystick data
                joy = self.sub.get_data()['joy']['buttons']
                # print('joy', joy, joy[0])
                if joy[0] == 1:

                    joy = self.sub.get_data()['joy']['axes']
                    print('joy axes', joy, joy[0])
                    # rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
                    #override = [int(val*200 + 1500) for val in joy]
                    #for _ in range(len(override), 8):
                    #    override.append(0)
                    # Send joystick data as rc output into rc override topic
                    # (fake radio controller)
                    #self.pub.set_data('/mavros/rc/override', override)
            except Exception as error:
                print('joy error:', error)

            '''
            try:
                # Get pwm output and send it to Gazebo model
                rc = self.sub.get_data()['mavros']['rc']['out']['channels']
                joint = JointState()
                joint.name = ["thr{}".format(u + 1) for u in range(5)]
                joint.position = [self.pwm_to_thrust(pwm) for pwm in rc]

                self.pub.set_data('/BlueRov2/body_command', joint)
            except Exception as error:
                print('rc error:', error)
            
            try:
                if True:
                # if not self.cam.frame_available():
                    continue

                # Show video output
                frame = self.cam.frame()
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
            except Exception as error:
                print('imshow error:', error)
            '''

            rate.sleep()

    def disarm(self):
        self.arm_service(False)


if __name__ == "__main__":
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Code()
    code.run()

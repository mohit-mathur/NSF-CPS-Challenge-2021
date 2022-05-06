#!/usr/bin/env python
"""
	Sample boiler-plate code for phase 1b
	Cyber Physical System Virtual Organization Challenge 2021 : SoilScope Lunar Lander ExoCam -- Earth Analog
	Team Name :
	Members :
"""

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, ActuatorControl, RCOut
from sensor_msgs.msg import Imu, Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Bool, Float64
import numpy as np
from numpy import sqrt, pi, sin, cos, arctan2, array, linalg, tan, dot

from std_srvs.srv import Empty
import time

g = 9.80665


class OffboardControl:
    """ Controller for PX4-UAV offboard mode """

    def __init__(self):
        rospy.init_node('OffboardControl', anonymous=True)

        # define your class variables here

        self.curr_pose = PoseStamped()  # current pose of the drone
        self.des_pose = PoseStamped()  # desired pose of the drone in position control mode
        self.is_ready_to_fly = False
        self.mode = "ASCEND"
        self.arm = False
        self.att = AttitudeTarget()
        self.attach = False
        self.orientation = [0] * 3
        self.attachFlag = True

        self.initializeVisionVariables()

        for i in reversed(range(1, 4)):
            print
            "Launching node in {}...".format(i)
            rospy.sleep(1)

        # define ros services, subscribers and publishers here
        # arm or disarm the UAV
        self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        # attach any two objects in Gazebo
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        # detach any two attached objects
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        # pause the Gazebo simulation if needed (could be used to debug the movement of the UAV)
        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        # example call:
        #		self.pause_physics_client.call()
        # could be used to reset the probe if needed
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.parachute_pub = rospy.Publisher('parachute_plugin/sample_probe', Bool, queue_size=10)

        # command your attitude target to the UAV
        self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        # command a position setpoint to the UAV
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # publish the debug image for reference if needed
        self.debugImgPub = rospy.Publisher('/debug_cam', Image, queue_size=10)

        # get the current state of the UAV
        self.state_sub = rospy.Subscriber('mavros/state', State, callback=self.state_callback)
        # self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        # get the visual from the onboard camera
        self.img_sub = rospy.Subscriber('/uav_camera_down/image_raw', Image, self.img_cb)

        # call the state machine
        self.controller()

    def initializeVisionVariables(self):
        self.bridge = CvBridge()
        self.debug = False
        self.imgSize = array([640, 640, 3])

    def pose_callback(self, msg):
        self.curr_pose = msg
        # gets the euler angles (roll, pitch, yaw) from the quaternion values
        # Note: The angles you get doesn't map the [-pi,pi] range properly and might require some conversion
        self.orientation = euler_from_quaternion(
            (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

    def img_cb(self, msg):
        try:
            if self.curr_pose.pose.position.z > 0:
                # access the visual from 'frame' to get the rover coordinates
                self.pixel_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # add your image processing code here
                # print(self.pixel_img.shape)
                self.debug = False
                image_blur = cv2.GaussianBlur(self.pixel_img, (3, 3), 0)
                image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
                # Use the two masks to create double mask
                min_red = np.array([0, 0, 0])
                max_red = np.array([40, 40, 40])
                mask = cv2.inRange(image_blur_hsv, min_red, max_red)

                # output = cv2.bitwise_and(self.pixel_img, self.pixel_img, mask=mask)
                edged = cv2.Canny(mask, 200, 500, L2gradient=True)
                result = cv2.bitwise_and(self.pixel_img, self.pixel_img, mask=mask)
                # edged = cv2.Canny(image=image_blur_hsv, threshold1=100, threshold2=200)

                contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]
                # print(contours)
                # cv2.drawContours(self.pixel_img, contours, -1, (0, 255, 0), 3)
                if len(contours) != 0:
                    c = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(self.pixel_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    y_a = 320 - y
                    x_a = 320 - x
                    self.prob_ang = np.arctan2(y_a, x_a)
                    cv2.imshow("Output",self.pixel_img)
                    cv2.waitKey(3)

                if self.debug:
                    # could be used to debug your detection logic
                    data = self.bridge.cv2_to_imgmsg(self.pixel_img, "bgr8")
                    data.header.stamp = msg.header.stamp
                    self.debugImgPub.publish(data)
        except CvBridgeError as e:
            #			print(e)
            pass

    def state_callback(self, msg):
        if msg.mode != 'OFFBOARD' or self.arm != True:
            # take_off
            self.set_offboard_mode()
            self.set_arm()

    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self.armService(True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def attach_models(self, model1, link1, model2, link2):
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        self.attach_srv.call(req)

    def detach_models(self, model1, link1, model2, link2):
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        self.detach_srv.call(req)

    def retro(self):
        while self.mode == 'RETRO' and not rospy.is_shutdown():
            # using the following line of code to detach the probe
            self.detach_models('if750a', 'base_link', 'sample_probe', 'base_link')
            self.mode = "LAND"

    def land(self):
        rate = rospy.Rate(15)  # Hz
        while self.mode == "LAND" and not rospy.is_shutdown():
            try:
                flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                isModeChanged = flightModeService(custom_mode='AUTO.LAND')
            except rospy.ServiceException as e:
                print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

            # use the following code to pull up the parachute on the probe so that it lands on the spot!
            rospy.sleep(1.2)
            self.parachute_pub.publish(1)

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def ascend(self):
        self.des_pose.pose.position.z = 15
        rate = rospy.Rate(15)
        set_ang = False
        t = time.time()
        while self.mode == "ASCEND" and not rospy.is_shutdown():
            self.pos_setpoint_pub.publish(self.des_pose)
            if self.curr_pose.pose.position.z > 0.5 and self.attachFlag:
                # Use the following line of code to attach the probe to the drone...
                self.attach_models('if750a', 'base_link', 'sample_probe', 'base_link')
                self.attachFlag = False
            # if the drone is ready for the throw, change mode to "BELLY-FLOP"
            if self.curr_pose.pose.position.z > 15:
                if set_ang is False:
                    angle = quaternion_from_euler(0, 0, self.prob_ang)
                    self.des_pose.pose.orientation.x = angle[0]
                    self.des_pose.pose.orientation.y = angle[1]
                    self.des_pose.pose.orientation.z = angle[2]
                    self.des_pose.pose.orientation.w = angle[3]
                    set_ang = True
                    belly = False
                if time.time() - t > 15:
                    self.mode = "BELLY-FLOP"
                continue
            rate.sleep()

    def belly_flop(self):

        # add your flip code here

        rate = rospy.Rate(15)  # Hz
        self.set_offboard_mode()
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.attach = True

        while self.mode == "BELLY-FLOP" and not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            # use AttitudeTarget.thrust to lift your quadcopter
            self.att.thrust = 9
            # use AttitudeTarget.body_rate.y to provide the angular velocity to your quadcopter
            self.att.body_rate.y = 16
            # self.att.body_rate.x = 9 #phase 1b
            # type_msk = 128 is used for controlling the rate exclusively, you may explore other values too
            self.att.type_mask = 128

            # if (you think the drone is ready to detach the probe):
            if self.orientation[1] > 0.6:
                self.att_setpoint_pub.publish(self.att)
                rate.sleep()
                self.mode = "RETRO"

            self.att_setpoint_pub.publish(self.att)

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def controller(self):
        """ A state machine developed to have UAV states """
        while not rospy.is_shutdown():
            # control your UAV states and functionalities here...
            if self.mode == "ASCEND":
                print("Ascending!")
                self.ascend()
            if self.mode == "BELLY-FLOP":
                print("belly flop!")
                self.belly_flop()
            if self.mode == "RETRO":
                self.retro()
            if self.mode == "LAND":
                self.land()


if __name__ == "__main__":
    OffboardControl()

#!/usr/bin/env python

import rospy
import numpy as np
from threading import Thread
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamGet, ParamSet

class UAV_control():
    """Control UAV by sending commands for arming, takeoff, landing and set points"""

    def __init__(self):
        try:
            #Intialize node, subscribers and publishers
            rospy.init_node('uav_control')
            self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=10)
            self.local_pose_subscriber = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_pose_cb, queue_size=10)
            self.mode_publisher = rospy.Publisher('/uav_mode', String, queue_size=10)
            self.set_pose_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

            #Get node params and initialize attributes
            self.rate = rospy.get_param('rate', 10)
            self.local_pose = PoseStamped()
            self.set_pose = PoseStamped()
            self.goal_radius = 0.25

            #Intialize service proxies
            rospy.wait_for_service('/mavros/cmd/arming')
            self.arming_proxy = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            rospy.wait_for_service('/mavros/cmd/takeoff')
            self.takeoff_proxy = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            rospy.wait_for_service('/mavros/cmd/land')
            self.land_proxy = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            rospy.wait_for_service('/mavros/set_mode')
            self.set_mode_proxy = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            rospy.wait_for_service('/mavros/param/get')
            self.get_param_proxy = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            rospy.wait_for_service('/mavros/param/set')
            self.set_param_proxy = rospy.ServiceProxy('/mavros/param/set', ParamSet)

            # Send setpoints in a seperate thread
            self.spin_pose_thread = Thread(target=self.pub_set_pose, args=())
            self.spin_pose_thread.daemon = True
            self.spin_pose_thread.start()
        except Exception as e:
            rospy.loginfo(e)
        else:
            rospy.loginfo("{0} node initialized successfully".format(rospy.get_name()))

    def state_cb(self, data):
        """Callback for state subscriber to store state data and publish mode"""
        try:
            if self.state.connected != data.connected:
                rospy.loginfo("connected changed from {0} to {1}".format(self.state.connected, data.connected))

            if self.state.armed != data.armed:
                rospy.loginfo("armed state changed from {0} to {1}".format(self.state.armed, data.armed))

            if self.state.mode != data.mode:
                rospy.loginfo("mode changed from {0} to {1}".format(self.state.mode, data.mode))

            if self.state.system_status != data.system_status:
                rospy.loginfo("system_status changed from {0} to {1}".format(self.state.system_status, data.system_status))
        except:
            pass
        self.state = data
        #Publish mode
        self.pub_mode()

    def local_pose_cb(self, data):
        """Callback for local pose subscriber to store pose data"""
        self.local_pose = data

    def arm_uav(self):
        """Send command to arm the UAV"""
        try:
            res = self.arming_proxy(True)
        except rospy.ServiceException:
            rospy.logerr("Failed to call arming service")
            return False
        else:
            if res.success:
                rospy.loginfo("UAV arming command successfull")
                return True
            else:
                rospy.logerr("Response from arming service failed")
                return False

    def takeoff(self, altitude):
        """Send command to take off the UAV to an altitude"""
        try:
            res = self.takeoff_proxy(altitude = altitude)
        except rospy.ServiceException:
            rospy.logerr("Failed to call takeoff service")
            return False
        else:
            if res.success:
                rospy.loginfo("UAV takeoff command successfull")
                return True
            else:
                rospy.logerr("Response from takeoff service failed")
                return False

    def land(self):
        """Send command to land the UAV at the current position"""
        try:
            res = self.land_proxy()
        except rospy.ServiceException:
            rospy.logerr("Failed to call landing service")
            return False
        else:
            if res.success:
                rospy.loginfo("UAV land command successfull")
                return True
            else:
                rospy.logerr("Response from landing service failed")
                return False

    def wait_before_next_cmd(self, sec=5.0):
        """Create a delay before sending next command"""
        rospy.loginfo("Waiting {0} seconds before next command".format(sec))
        rospy.sleep(sec)

    def pub_set_pose(self):
        """Looped publisher for setpoint position"""
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                self.set_pose.header.stamp = rospy.Time.now()
                self.set_pose_publisher.publish(self.set_pose)
                rate.sleep()
            except rospy.ROSInterruptException:
                return

    def update_set_pose(self, pose):
        """Update setpoint position"""
        self.set_pose = pose
        rospy.loginfo("Local setpoint changed to ({0},{1},{2})".format(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))

    def is_set_pose_reached(self):
        """Check if the current position is within a radius of setpoint position"""
        current_position = np.array([self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z])
        set_position = np.array([self.set_pose.pose.position.x, self.set_pose.pose.position.y, self.set_pose.pose.position.z])
        return np.linalg.norm(set_position - current_position) <= self.goal_radius

    def wait_to_reach_set_pose(self):
        """Wait till the current set position is reached"""
        while not rospy.is_shutdown():
            if self.is_set_pose_reached():
                return True

    def update_goal_radius(self, radius):
        """Update radius (in meters) used to check if setpoint is reached"""
        self.goal_radius = radius

    def pub_mode(self):
        """Publish current mode to a seperate topic"""
        self.mode_publisher.publish(self.state.mode)

    def set_mode(self, mode):
        """Set a new mode for the UAV"""
        try:
            res = self.set_mode_proxy(0, mode)
        except rospy.ServiceException:
            rospy.logerr("Failed to call set mode service")
            return False
        else:
            if res.mode_sent:
                rospy.loginfo("UAV mode changed to {0}".format(mode))
                return True
            else:
                rospy.logerr("Response from set mode service failed")
                return False

    def get_param(self, param):
        """Get parameter value from PX4 configuration"""
        try:
            res = self.get_param_proxy(param)
        except rospy.ServiceException:
            rospy.logerr("Failed to call get param service")
            return False
        else:
            if res.success:
                return res.value.integer if res.value.integer != 0 else res.value.real
            else:
                rospy.logerr("Response from get param service failed")
                return False

    def set_param(self, param, value):
        """Set parameter value for PX4 configuration"""
        param_value = ParamValue()
        if isinstance(value, int):
            param_value.integer = value
        elif isinstance(value, float):
            param_value.real = value
        else:
            rospy.logerr("Set param value must be an integer or a real number")
            return False
        try:
            res = self.set_param_proxy(param, param_value)
        except rospy.ServiceException:
            rospy.logerr("Failed to call set param service")
            return False
        else:
            if res.success:
                rospy.loginfo("{0} param set to {1}".format(param, value))
                return True
            else:
                rospy.logerr("Response from set param service failed")
                return False

def main():
    """Main function"""
    uav_control = UAV_control()
    #Disable RC Loss Failsafe
    rospy.loginfo("Disabling RC Loss Failsafe")
    uav_control.set_param('NAV_RCL_ACT', 0)
    #Setting mode to LOITER
    uav_control.set_mode('AUTO.LOITER')
    #Arm UAV
    uav_control.arm_uav()
    uav_control.wait_before_next_cmd(5)
    #Send command to a Takeoff to a height of 3 meters
    uav_control.takeoff(3)
    uav_control.wait_before_next_cmd(5)
    #Arm UAV and wait for Takeoff
    uav_control.arm_uav()
    uav_control.wait_before_next_cmd(10)
    #Change to OFFBOARD mode and send new setpoint position
    uav_control.set_mode('OFFBOARD')
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 7
    pose.pose.position.y = 5
    pose.pose.position.z = 2
    uav_control.update_set_pose(pose)
    #Wait to reach setpoint
    uav_control.wait_to_reach_set_pose()
    rospy.loginfo("Setpoint position reached")
    #Hover for 5 seconds
    uav_control.wait_before_next_cmd(5)
    #Land UAV
    uav_control.land()
    uav_control.wait_before_next_cmd(5)

if __name__ == '__main__':
    main()

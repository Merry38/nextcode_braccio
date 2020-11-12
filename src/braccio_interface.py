#!/usr/bin/env python

import sys
import time

import rospy
from rospy.rostime import Time
from rospy.rostime import Duration 
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray


class DirectJointInterface:
    def __init__(self):
        self.camera_width = rospy.get_param('~camera_width', 640)
        self.camera_height = rospy.get_param('~camera_height', 480)
        self.camera_radius = rospy.get_param('~camera_radius', 10)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.05)
        self.planning_time = rospy.get_param('~planning_time', 2.0)
        self.p_gain_yaw = rospy.get_param('~p_gain_yaw', 0.001)
        self.p_gain_pitch = rospy.get_param('~p_gain_pitch', 0.001)
        self.initial_pose = rospy.get_param('~initial_pose', [])

        rospy.loginfo("camera_width: %d" % self.camera_width)
        rospy.loginfo("camera_height: %d" % self.camera_height)
        rospy.loginfo("camera_radius: %d" % self.camera_radius)
        rospy.loginfo("goal_tolerance: %f" % self.goal_tolerance)
        rospy.loginfo("planning_time: %f" % self.planning_time)
        rospy.loginfo("p_gain_yaw: %f" % self.p_gain_yaw)
        rospy.loginfo("p_gain_pitch: %f" % self.p_gain_pitch)

        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0

        self.js_publisher = rospy.Publisher("position", Float32MultiArray, queue_size=10)

        self.box_subscriber = rospy.Subscriber("bounding_box", Int16MultiArray, self.bounding_box_callback, queue_size=10)
        self.state_subscriber = rospy.Subscriber("current_state", JointState, self.braccio_state_callback, queue_size=10)


    def initialize_position(self):
        if (len(self.initial_pose) > 0) :
            rospy.loginfo("Waiting for reaching initial pose")

            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                try:
                    goal = self.goal_reached()
                except:
                    # rospy.loginfo("State not found")
                    continue

                if goal:
                    break

                js = Float32MultiArray()

                for p in self.initial_pose:
                    js.data.append(p)

                self.js_publisher.publish(js)

                rate.sleep()

            if goal:
                rospy.loginfo("Initial position reached!");


    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.loopBody()
            rate.sleep()


    def loopBody(self):
        try:
            delta_x = self.center_x - self.camera_current_x
            delta_y = self.center_y - self.camera_current_y

            if (self.bounding_box_updated):
                self.planJointToDelta(delta_x, delta_y);
                self.bounding_box_updated = False;
        except:
            rospy.loginfo("Uninitialized position")


    def braccio_state_callback(self, joint_data):
        self.current_js_state = joint_data;

        rospy.loginfo("Received position %f %f %f %f %f %f",
                 self.current_js_state.position[0], self.current_js_state.position[1],
                 self.current_js_state.position[2], self.current_js_state.position[3],
                 self.current_js_state.position[4], self.current_js_state.position[5]);


    def bounding_box_callback(self, bounding_box):
        if (len(bounding_box.data) == 0):
            return;

        label_id = bounding_box.data[0]
        startX = bounding_box.data[1]
        startY = bounding_box.data[2]
        endX = bounding_box.data[3]
        endY = bounding_box.data[4]

        rospy.loginfo("Received bounding_box %d %d %d %d %d",
                label_id, startX, startY, endX, endY);

        self.camera_current_x = (endX + startX) / 2.0
        self.camera_current_y = (endY + startY) / 2.0

        image_area = self.camera_width * self.camera_height

        self.box_area = ((endX - startX) * (endY - startY)) / image_area;
        self.bounding_box_updated = True;


    def planJointToDelta(self, delta_x, delta_y):
        js = Float32MultiArray()

        for p in self.current_js_state.position:
            js.data.append(p)

        index_joint_x = 0
        index_joint_y = 3

        p_x = self.p_gain_yaw * (1 - self.box_area)
        p_y = self.p_gain_pitch * (1 - self.box_area)

        js.data[index_joint_x] -= (delta_x * p_x)
        js.data[index_joint_y] += (delta_y * p_y)

        rospy.loginfo("Publishing position %f %f", js.data[index_joint_x], js.data[index_joint_y])

        self.js_publisher.publish(js)


    def goal_reached(self):
        # rospy.loginfo("Checking goal")

        if (len(self.current_js_state.position) == 0):
            return False

        for i in range(len(self.current_js_state.position)):
            rospy.loginfo("Comparing %f with %f", self.initial_pose[i], self.current_js_state.position[i])

            if (abs(self.initial_pose[i] - self.current_js_state.position[i]) > self.goal_tolerance):
                return False

        return True


if __name__ == "__main__":
    rospy.init_node("braccio_interface")

    interface = DirectJointInterface()
    interface.initialize_position();
    interface.loop();


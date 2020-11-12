#!/usr/bin/env python

import sys

# COCO classes
from coco import *

# TensorRT
import jetson.inference
import jetson.utils

# ROS
import rospy
from rospy.rostime import Time
from rospy.rostime import Duration 
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

def main():
    # parse arguments
    inputURI = rospy.get_param('~inputURI', '/dev/video0')
    outputURI = rospy.get_param('~outputURI', '')
    target_class = rospy.get_param('~target_class', 'person')
    network = rospy.get_param('~network', 'ssd-mobilenet-v2')
    overlay = rospy.get_param('~overlay', 'box,labels,conf')
    threshold = rospy.get_param('~threshold', 0.5) 

    # prepare publisher
    box_publisher = rospy.Publisher("bounding_box", Int16MultiArray, queue_size=10)

    # load the object detection network
    net = jetson.inference.detectNet(network, threshold)

    # create video sources & outputs
    input = jetson.utils.videoSource(inputURI)
    output = jetson.utils.videoOutput(outputURI)

    # process frames until the user exits
    rate = rospy.Rate(10)

    rospy.loginfo("Requested tracking for class %s", target_class)

    if (target_class not in COCO_CLASSES_LIST):
        target_class = 'person'
        target_class_id = 1
    else:
        target_class_id = COCO_CLASSES_LIST.index(target_class)

    rospy.loginfo("Tracking target class %d : %s", target_class_id, target_class)

    while not rospy.is_shutdown():
        # capture the next image
        img = input.Capture()

        # detect objects in the image (with overlay)
        detections = net.Detect(img, overlay=overlay)

        # print the detections
        # rospy.loginfo("detected {:d} objects in image".format(len(detections)))

        target_detection = None

        for detection in detections:
            if (detection.ClassID == target_class_id):
                target_detection = detection

            # rospy.loginfo(detection)

        # render the image
        output.Render(img)

        # update the title bar
        output.SetStatus("{:s} | Network {:.0f} FPS".format(network, net.GetNetworkFPS()))

        # print out performance info
        # net.PrintProfilerTimes()

        # Publish bounding box
        if (target_detection is not None):
            # rospy.loginfo("Publishing detection box")

            box = Int16MultiArray()
            box.data.append(target_detection.ClassID)
            box.data.append(target_detection.Left)
            box.data.append(target_detection.Top)
            box.data.append(target_detection.Right)
            box.data.append(target_detection.Bottom)

            box_publisher.publish(box)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("detectnet")

    main()


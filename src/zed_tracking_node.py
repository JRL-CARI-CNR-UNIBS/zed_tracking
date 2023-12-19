#!/usr/bin/env python3

import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from utils import *
import SkeletonTracker


def main():

    # Initialize the ROS node    
    node_name = read_param('node_name', default_value='zed_tracking_node')
    rospy.init_node(node_name, anonymous=True)

    # Create a ROS publisher for the skeleton topic
    skeleton_pub = rospy.Publisher(node_name + '/skeleton', String, queue_size=10)

    # Create a SkeletonTracker object and initiazlize its parameters from the ROS parameter server
    # sk_tracker = SkeletonTracker.SkeletonTracker()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # rospy.loginfo("Publishing skeleton")
        rate.sleep()

        # if zed.grab() == sl.ERROR_CODE.SUCCESS:
        #     err = zed.retrieve_bodies(bodies, body_runtime_param)
        #     if bodies.is_new:
        #         body_array = bodies.body_list
        #         print(str(len(body_array)) + " Person(s) detected\n")
        #         if len(body_array) > 0:
        #             first_body = body_array[0]
        #             print("First Person attributes:")
        #             print(" Confidence (" + str(int(first_body.confidence)) + "/100)")
        #             if body_params.enable_tracking:
        #                 print(" Tracking ID: " + str(int(first_body.id)) + " tracking state: " + repr(
        #                     first_body.tracking_state) + " / " + repr(first_body.action_state))
        #             position = first_body.position
        #             velocity = first_body.velocity
        #             dimensions = first_body.dimensions
        #             print(" 3D position: [{0},{1},{2}]\n Velocity: [{3},{4},{5}]\n 3D dimentions: [{6},{7},{8}]".format(
        #                 position[0], position[1], position[2], velocity[0], velocity[1], velocity[2], dimensions[0],
        #                 dimensions[1], dimensions[2]))
        #             if first_body.mask.is_init():
        #                 print(" 2D mask available")

        #             print(" Keypoint 2D ")
        #             keypoint_2d = first_body.keypoint_2d
        #             for it in keypoint_2d:
        #                 print("    " + str(it))
        #             print("\n Keypoint 3D ")
        #             keypoint = first_body.keypoint
        #             for it in keypoint:
        #                 print("    " + str(it))
        #             print('\n\n\n')

    # Close the camera
    sk_tracker.camera.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
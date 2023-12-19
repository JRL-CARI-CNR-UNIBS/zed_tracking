#!/usr/bin/env python3

import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
import utils


def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = 1

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(err)+". Exit program.")
        exit()

    body_params = sl.BodyTrackingParameters()

    # Different model can be chosen, optimizing the runtime or the accuracy
    body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    body_params.enable_tracking = True
    body_params.image_sync = True
    body_params.enable_segmentation = False

    # Optimize the person joints position, requires more computations
    body_params.enable_body_fitting = True

    if body_params.enable_tracking:
        positional_tracking_param = sl.PositionalTrackingParameters()
        # positional_tracking_param.set_as_static = True
        positional_tracking_param.set_floor_as_origin = True
        zed.enable_positional_tracking(positional_tracking_param)

    print("Body tracking: Loading Module...")

    err = zed.enable_body_tracking(body_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable Body Tracking : "+repr(err)+". Exit program.")
        zed.close()
        exit()

    bodies = sl.Bodies()
    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    # For outdoor scene or long range, the confidence should be lowered to avoid missing detections (~20-30)
    # For indoor scene or closer range, a higher confidence limits the risk of false positives and increase the precision (~50+)
    body_runtime_param.detection_confidence_threshold = 40
    
    rospy.init_node('zed_tracking_node', anonymous=True)
    skeleton_pub = rospy.Publisher('zed_tracking_node/skeleton', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        skeleton_pub.publish(hello_str)
        rate.sleep()

        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            err = zed.retrieve_bodies(bodies, body_runtime_param)
            if bodies.is_new:
                body_array = bodies.body_list
                print(str(len(body_array)) + " Person(s) detected\n")
                if len(body_array) > 0:
                    first_body = body_array[0]
                    print("First Person attributes:")
                    print(" Confidence (" + str(int(first_body.confidence)) + "/100)")
                    if body_params.enable_tracking:
                        print(" Tracking ID: " + str(int(first_body.id)) + " tracking state: " + repr(
                            first_body.tracking_state) + " / " + repr(first_body.action_state))
                    position = first_body.position
                    velocity = first_body.velocity
                    dimensions = first_body.dimensions
                    print(" 3D position: [{0},{1},{2}]\n Velocity: [{3},{4},{5}]\n 3D dimentions: [{6},{7},{8}]".format(
                        position[0], position[1], position[2], velocity[0], velocity[1], velocity[2], dimensions[0],
                        dimensions[1], dimensions[2]))
                    if first_body.mask.is_init():
                        print(" 2D mask available")

                    print(" Keypoint 2D ")
                    keypoint_2d = first_body.keypoint_2d
                    for it in keypoint_2d:
                        print("    " + str(it))
                    print("\n Keypoint 3D ")
                    keypoint = first_body.keypoint
                    for it in keypoint:
                        print("    " + str(it))
                    print('\n\n\n')

    # Close the camera
    zed.disable_body_tracking()
    zed.close()


if __name__ == '__main__':
    try:
        # main()
        pass
    except rospy.ROSInterruptException:
        pass
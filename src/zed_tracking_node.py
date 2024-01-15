#!/usr/bin/env python3

import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from utils import *
import SkeletonTracker

def main():
    # Parse the config file containing the parameters
    base_path = os.path.dirname(os.path.dirname(__file__))
    camera_params = parse_config_file(os.path.join(base_path, 'config', 'camera_config.yaml'))
    tracking_params = parse_config_file(os.path.join(base_path, 'config', 'tracking_config.yaml'))

    # Initialize the ROS node    
    node_name = read_param('node_name', default_value='zed_tracking_node')
    rospy.init_node(node_name, anonymous=True)

    # Create a ROS publisher for the skeleton topic
    # skeleton_pub = rospy.Publisher(node_name + '/skeleton', String, queue_size=10)

    # Create a SkeletonTracker object and initialize its parameters from the ROS parameter server
    sk_tracker = SkeletonTracker.SkeletonTracker()
    sk_tracker.load_parameters(camera_params, tracking_params)

    # Open the camera and load the body tracking module
    sk_tracker.open_camera()
    sk_tracker.load_body_tracking()

    # Create a ROS rate object to maintain a 10Hz refresh rate
    rate = rospy.Rate(10) # 10hz

    keypoint_publisher = rospy.Publisher(node_name + '/keypoints', MarkerArray, queue_size = 100)
    skeleton_publisher = rospy.Publisher(node_name + '/skeleton', Marker, queue_size = 100)
    poses_publisher = rospy.Publisher(node_name + '/poses', PoseArray, queue_size = 100)

    while not rospy.is_shutdown():
        if sk_tracker.camera.grab() == sl.ERROR_CODE.SUCCESS:
            err = sk_tracker.camera.retrieve_bodies(sk_tracker.bodies, sk_tracker.body_runtime_params)

            if sk_tracker.bodies.is_new:
                body_array = sk_tracker.bodies.body_list
                rospy.loginfo(str(len(body_array)) + " Person(s) detected\n")

                if len(body_array) > 0:
                    for body in body_array:
                        rospy.loginfo("Person ID: " + str(int(body.id)) + 
                                      " | Confidence (" + str(int(body.confidence)) + "/100)")
                        
                        markerArray = MarkerArray()

                        if sk_tracker.body_params.enable_tracking:
                            rospy.loginfo("Tracking state: " + repr(body.tracking_state) + " / " + repr(body.action_state))
                        
                        position = body.position
                        velocity = body.velocity
                        dimensions = body.dimensions
                        rospy.loginfo("3D position: [{0},{1},{2}]\n Velocity: [{3},{4},{5}]\n 3D dimentions: [{6},{7},{8}]".format(
                            position[0], position[1], position[2],
                            velocity[0], velocity[1], velocity[2],
                            dimensions[0], dimensions[1], dimensions[2]))
                        
                        if body.mask.is_init():
                            rospy.loginfo("2D mask available")

                        rospy.loginfo("2D Keypoints")
                        keypoint_2d = body.keypoint_2d
                        for it in keypoint_2d:
                            rospy.loginfo("    " + str(it))
                        
                        rospy.loginfo('\n')
                        
                        rospy.loginfo("3D Keypoints")
                        keypoint = body.keypoint
                        for it in keypoint:
                            rospy.loginfo("    " + str(it))

                            # Marker msg definition
                            marker = Marker()
                            marker.type = Marker.SPHERE
                            marker.ns = "Skeleton"
                            # marker.header.frame_id = 'TODO' # TODO

                            marker.action = Marker.ADD
                            marker.scale.x = 0.03
                            marker.scale.y = 0.03
                            marker.scale.z = 0.03

                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0

                            marker.color.a =1.0
                            # marker.id = 0 # TODO
                            marker.header.stamp = rospy.Time.now()
                            marker.pose = Pose(Point(it[0], it[1], it[2]), Quaternion(0,0,0,1))

                            markerArray.markers.append(marker)
                        
                        keypoint_publisher.publish(markerArray)

                        rospy.loginfo('\n')

                        rospy.loginfo("3D Keypoints: Local orientation for each keypoint")
                        if sk_tracker.body_params.body_format in  [sl.BODY_FORMAT['BODY_34'], sl.BODY_FORMAT['BODY_38']]:
                            keypoint_rot = body.local_orientation_per_joint
                            for it in keypoint_rot:
                                rospy.loginfo("    " + str(it))

                        rospy.loginfo('\n')

                        rospy.loginfo("3D Keypoints: Local translation for each keypoint")
                        if sk_tracker.body_params.body_format in  [sl.BODY_FORMAT['BODY_34'], sl.BODY_FORMAT['BODY_38']]:
                            keypoint_rot = body.local_position_per_joint
                            for it in keypoint_rot:
                                rospy.loginfo("    " + str(it))

                        global_root_orientation = body.global_root_orientation

                        rospy.loginfo('\n\n\n')

                        skeleton = Marker()
                        skeleton.type = Marker.LINE_LIST
                        skeleton.ns = "skeleton_ID" + str(int(body.id))
                        skeleton.id = int(body.id)
                        skeleton.action = Marker.ADD
                        skeleton.header.frame_id = "zed2_camera_center"
                        skeleton.header.stamp = rospy.Time.now()
                        skeleton.scale.x = 0.03
                        skeleton.color.r = 0.0
                        skeleton.color.g = 1.0
                        skeleton.color.b = 0.0
                        skeleton.color.a = 0.7







        rate.sleep()

    # Close the camera
    sk_tracker.camera.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
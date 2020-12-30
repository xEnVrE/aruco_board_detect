#!/usr/bin/env python

# This script allows to use a second camera, detached from the robot.
# Since it is quite tricky to set up a multi-realsense setup, we first
# acquire the pose of a markerboard in front of the robot and then ask the user
# to hot-swap the cameras (i.e. unplug the first one and connect the second one).
# This, however, requires the user to disable any static transform broadcaster
# between the robot frames and the camera frame, otherwise there will be

import rospy

import tf
from tf import transformations
import numpy as np

BOARD_FRAME_NAME = "aruco_board"
ROOT_FRAME_NAME = "panda_link0"
CAMERA_FRAME_NAME = "camera_link"

if __name__ == "__main__":

    rospy.init_node("camera_locator")

    # Configure TF transform listener and broadcaster
    tf_listener = tf.TransformListener(True, rospy.Duration(10))
    tf_broadcaster = tf.TransformBroadcaster()

    root_to_board_matrix = None
    camera_to_board_matrix = None

    rospy.loginfo("Listening for transfom from {} to {}...".format(ROOT_FRAME_NAME, BOARD_FRAME_NAME))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (translation, rotation) = tf_listener.lookupTransform(ROOT_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time(0))
            root_to_board_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    raw_input("Plug in the new camera and press any key")

    rospy.loginfo("Listening for transform from {} to {}...".format(CAMERA_FRAME_NAME, BOARD_FRAME_NAME))

    while not rospy.is_shutdown():
        try:
            translation, rotation = tf_listener.lookupTransform(CAMERA_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time(0))
            camera_to_board_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    root_to_camera_matrix = np.dot(root_to_board_matrix, np.linalg.inv(camera_to_board_matrix))
    translation = transformations.translation_from_matrix(root_to_camera_matrix)
    rotation = transformations.quaternion_about_axis(transformations.rotation_from_matrix(root_to_camera_matrix)[0], transformations.rotation_from_matrix(root_to_camera_matrix)[1])

    rospy.loginfo("Transform computed. Broadcasting transform...")

    while not rospy.is_shutdown():
        tf_broadcaster.sendTransform(translation,
                                rotation,
                                rospy.Time.now(),
                                "camera_link",
                                "panda_link0")

















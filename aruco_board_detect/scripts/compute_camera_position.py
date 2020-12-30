#!/usr/bin/env python

# Find the pose of the camera by using the markerboard

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

    rospy.loginfo("Listening for transfom from {} to {}...".format(ROOT_FRAME_NAME, BOARD_FRAME_NAME))

    if not (tf_listener.frameExists(BOARD_FRAME_NAME) and self._tf_listener.frameExists(ROOT_FRAME_NAME)):
            rospy.logerr("Either tf transform {} or {} do not exist".format(BOARD_FRAME_NAME, ROOT_FRAME_NAME))
            return 0

    tf_listener.waitForTransform(ROOT_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time.now(), rospy.Duration(5.0))

    translation, rotation = tf_listener.lookupTransform(BOARD_FRAME_NAME, ROOT_FRAME_NAME, rospy.Time(0))

    root_to_board_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    raw_input("Plug in the new camera and press any key")

    rospy.loginfo("Listening for transfom from {} to {}...".format(CAMERA_FRAME_NAME, BOARD_FRAME_NAME))

    tf_listener.waitForTransform(CAMERA_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time.now(), rospy.Duration(5.0))

    translation, rotation = tf_listener.lookupTransform(BOARD_FRAME_NAME, CAMERA_FRAME_NAME, rospy.Time(0))

    camera_to_board_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    root_to_camera_matrix = np.dot(root_to_board_matrix, np.linalg.inv(camera_to_board_matrix))

    translation = transformations.translation_from_matrix(root_to_camera_matrix)

    rotation = transformations.rotation_from_matrix(root_to_camera_matrix)




















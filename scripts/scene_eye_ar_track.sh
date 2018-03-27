#!/bin/sh
roslaunch usbcam usbcam_eye.launch & 

sleep 1 && ROS_NAMESPACE=/eye/left roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/eye/left/image_rect_color cam_info_topic:=/eye/left/camera_info  output_frame:=/eye/left/camera_link  marker_size:=0.5  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_0.5.xml max_track_error:=0.1  max_new_marker_error:=0.04 &

sleep 1 && ROS_NAMESPACE=/eye/right roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/eye/right/image_rect_color cam_info_topic:=/eye/right/camera_info  output_frame:=/eye/right/camera_link  marker_size:=0.5  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_0.5.xml max_track_error:=0.1  max_new_marker_error:=0.04 &

sleep 1 && roslaunch eyetracking double_pupiltracking.launch & 

sleep 1 && roslaunch usbcam usbcam_stereo.launch &

sleep 1 && ROS_NAMESPACE=/stereo/left roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/stereo/left/image_rect_color cam_info_topic:=/stereo/left/camera_info  output_frame:=/stereo/left/camera_link  marker_size:=5.4  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_5.4.xml max_track_error:=0.1  max_new_marker_error:=0.04 &

sleep 1 && ROS_NAMESPACE=/stereo/right roslaunch ar_track_alvar pr2_bundle_no_kinect.launch  cam_image_topic:=/stereo/right/image_rect_color cam_info_topic:=/stereo/right/camera_info  output_frame:=/stereo/right/camera_link  marker_size:=5.4  bundle_files:=/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_5.4.xml max_track_error:=0.1  max_new_marker_error:=0.04 &

sleep 1 && rviz

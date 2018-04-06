#include <ros/ros.h>
#include <eyetracking_msgs/RotatedRect.h>
#include <eyetracking_msgs/ImagePoint.h>
#include <iostream>
#include <opencv2/opencv.hpp>


cv::Point2d left_pupil;
cv::Point2d right_pupil;
cv::Point2d left_gaze;
bool leftDone = false;
bool rightDone = false;
eyetracking_msgs::ImagePoint left_gaze_msg;


void leftCallback(const eyetracking_msgs::RotatedRectConstPtr msg) {
  left_pupil = cv::Point2d(msg->x,msg->y);
  if (left_pupil.x == 0.0 && left_pupil.y==0.0) leftDone = false;
  else {
    leftDone = true;
    left_gaze_msg.header = msg->header;
  } 
}

void rightCallback(const eyetracking_msgs::RotatedRectConstPtr msg) {
  right_pupil = cv::Point2d(msg->x,msg->y);
  if (right_pupil.x == 0.0 && right_pupil.y==0.0) rightDone = false;
  else rightDone = true;
}

void MapFunction(const cv::Point2d& left, const cv::Point2d& right, cv::Point2d& gaze) {
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "macaca_eyescene_map_node");
  ros::NodeHandle nh("~");

  std::string left_pupil_topic = "/eye/left/pupil_ellipse";
  std::string right_pupil_topic = "/eye/right/pupil_ellipse";
  std::string gaze_point_left = "/eye/left/gaze_point";
  ros::Subscriber left_sub = nh.subscribe(left_pupil_topic, 1, leftCallback);
  ros::Subscriber right_sub = nh.subscribe(right_pupil_topic, 1, rightCallback);
  ros::Publisher eye_mapTo_scene_pub = nh.advertise<eyetracking_msgs::ImagePoint>(gaze_point_left, 1);
  ros::Rate r(30);
  while(ros::ok()) {
    if (leftDone && rightDone) {
      MapFunction(left_pupil, right_pupil, left_gaze);
      left_gaze_msg.x = left_gaze.x;
      left_gaze_msg.y = left_gaze.y;
      eye_mapTo_scene_pub.publish(left_gaze_msg);
    }
    ros::spinOnce();
  }
  return 0;
}

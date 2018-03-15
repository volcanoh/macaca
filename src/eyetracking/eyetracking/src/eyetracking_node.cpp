#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pupiltracker/cvx.h>
#include <pupiltracker/PupilTracker.h>

#include <eyetracking/PupilParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <eyetracking_msgs/RotatedRect.h>

cv_bridge::CvImagePtr cv_ptr_;
cv::Mat image_;
cv::Point2f pupil_center_;
image_transport::Subscriber image_sub_;
ros::Publisher pupil_pub_;

eyetracking_msgs::RotatedRect pupil_ellipse_msg;

pupiltracker::TrackerParams params;

const std::string output_topic_="pupil_ellipse";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
    image_ = cv_ptr_->image;
    try {
      pupiltracker::findPupilEllipse_out out;
      pupiltracker::tracker_log log;
      pupiltracker::findPupilEllipse(params, image_, out, log);

      pupil_ellipse_msg.header = msg->header;
      pupil_ellipse_msg.x = out.elPupil.center.x;
      pupil_ellipse_msg.y = out.elPupil.center.y;
      pupil_ellipse_msg.angle = out.elPupil.angle;
      pupil_ellipse_msg.width = out.elPupil.size.width;
      pupil_ellipse_msg.height = out.elPupil.size.height;
      pupil_pub_.publish(pupil_ellipse_msg);

    }
    catch (cv::Exception e) {
      ROS_ERROR("Pupil Algorithm Error!");
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

void configCallback(eyetracking::PupilParamsConfig &config, uint32_t) {
  ROS_INFO("Reconfigure request : %i %i %i %i %i %i %i %i %s %i %s %i",
           config.Radius_Min,
           config.Radius_Max,
           config.CannyBlur,
           config.CannyThreshold1,
           config.CannyThreshold2,
           config.StarburstPoints,
           config.PercentageInliers,
           config.InlierIterations,
           config.ImageAwareSupport ? "True" : "False",
           config.EarlyTerminationPercentage,
           config.EarlyRejection ? "True" : "False",
           config.Seed
          );
  params.Radius_Min = config.Radius_Min;
  params.Radius_Max= config.Radius_Max;
  params.CannyBlur = config.CannyBlur;
  params.CannyThreshold1 = config.CannyThreshold2;
  params.CannyThreshold2 = config.CannyThreshold2;
  params.StarburstPoints= config.StarburstPoints;
  params.PercentageInliers = config.PercentageInliers;
  params.InlierIterations = config.InlierIterations;
  params.ImageAwareSupport = config.ImageAwareSupport;
  params.EarlyTerminationPercentage = config.EarlyTerminationPercentage;
  params.EarlyRejection = config.EarlyRejection;
  params.Seed = config.Seed;

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<eyetracking::PupilParamsConfig> dyn_server;
  dynamic_reconfigure::Server<eyetracking::PupilParamsConfig>::CallbackType dyn_callback;
  dyn_callback = boost::bind(&configCallback, _1, _2);
  dyn_server.setCallback(dyn_callback);

  pupil_pub_ = nh.advertise<eyetracking_msgs::RotatedRect>(output_topic_, 10);

  image_transport::ImageTransport it(nh);
  image_sub_ = it.subscribe("image_rect_color", 1, imageCallback);

  ros::spin();
  return 0;
}

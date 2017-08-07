#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

static const std::string SUB_TOPIC = "/image_feed";
static const std::string PUB_TOPIC = "/processed_image";

using namespace cv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub;

public:
  ImageConverter(std::string topic)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(SUB_TOPIC, 1,
      &ImageConverter::imageCb, this);

    image_pub = it_.advertise(PUB_TOPIC, 1);

  }

  ~ImageConverter() {}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Declare opencv mat pointer
    cv_bridge::CvImagePtr cv_ptr;
    // Try to convert recieved sensor_msgs/Image to opencv
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat copy = cv_ptr->image.clone();

    Mat frame_gray;

    GaussianBlur(copy, copy, Size(9,9), 2, 2);

    std::vector<Vec3f> circles;

    cvtColor(copy, frame_gray, cv::COLOR_BGR2GRAY);

    HoughCircles(frame_gray, circles, HOUGH_GRADIENT, 
                 1, frame_gray.rows/16, 100, 30, 20, 200);
    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 40, CV_RGB(255,0,0));

    for( size_t i =0; i< circles.size(); i++)
    {
      Point centre(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // draw centre
      circle(cv_ptr->image, centre, 3, Scalar(255,0,0), -1, 8, 0);
      // draw circle
      circle(cv_ptr->image, centre, radius, Scalar(0,255,0), 3, 8, 0);
    }


    image_pub.publish(cv_ptr->toImageMsg());
    ROS_INFO("Image processed and sent");

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic(argv[0]);
  ros::spin();
  return 0;
}
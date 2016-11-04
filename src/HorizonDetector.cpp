#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";

class HorizonDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  HorizonDetector()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/galago_simple/body_sensor/cam_a/image_raw", 1,
        &HorizonDetector::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW2);
  }

  ~HorizonDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv:Mat out;

    simpleDetector(cv_ptr->image, out);

    // Update GUI Window
          cv::imshow(OPENCV_WINDOW, cv_ptr->image);
          cv::imshow(OPENCV_WINDOW2, out);
          cv::waitKey(3);
  }

  void testLoop()
  {
    ros::Rate rate(1);

    cv::Mat img;

    while (ros::ok())
    {
      img = cv::imread("data/static_horison_horizontal.png", CV_LOAD_IMAGE_COLOR);
      if (!img.data)                              // Check for invalid input
      {
        std::cout << "Could not open or find the image" << std::endl;
        return;
      }

      Mat out;

      simpleDetector(img, out);

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, img);
      cv::imshow(OPENCV_WINDOW2, out);
      cv::waitKey(3);

      ros::spinOnce();
      rate.sleep();
    }
  }

  void simpleDetector(cv::Mat& src, cv::Mat& out)
  {
    Mat dst, cdst;

    cvtColor( src, dst, CV_BGR2GRAY );
    threshold( dst, dst, 50, 255, 0 );

//noise removal
 Mat kernel = Mat::ones(3,3, CV_8U);
 morphologyEx(dst,dst,MORPH_OPEN,kernel);

 //# sure background area
 //sure_bg = cv2.dilate(opening,kernel,iterations=3)

    Canny(dst, dst, 50, 200, 3);
    // cvtColor(src, src, CV_GRAY2BGR);

    vector < Vec2f > lines;
    HoughLines(dst, lines, 1, CV_PI / 180 / 10, 100, 0, 0);

    for (size_t i = 0; (i < 1) && (lines.size() > 0); i++)
    {
      float rho = lines[i][0], theta = lines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a * rho, y0 = b * rho;
      pt1.x = cvRound(x0 + 1000 * (-b));
      pt1.y = cvRound(y0 + 1000 * (a));
      pt2.x = cvRound(x0 - 1000 * (-b));
      pt2.y = cvRound(y0 - 1000 * (a));
      line(src, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
    }
    out = dst;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "horizon_detector");
  HorizonDetector hd;
  //hd.testLoop();
  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/superres/optical_flow.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace cv::superres;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
   // TV-L1アルゴリズムによるオプティカルフロー計算オブジェクトの生成
   Ptr<DenseOpticalFlowExt> opticalFlow = superres::createOptFlow_DualTVL1();

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Draw an example circle on the video stream
   // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
     // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    Mat prev;
   // 現在のフレームを保存
		Mat curr;
		
		curr = cv_ptr->image ;
		if( !prev ) prev = curr ;
		

		// オプティカルフローの計算
		Mat flowX, flowY;
		opticalFlow->calc(prev, curr, flowX, flowY);

		// オプティカルフローの可視化（色符号化）
		//  オプティカルフローを極座標に変換（角度は[deg]）
		Mat magnitude, angle;
		cartToPolar(flowX, flowY, magnitude, angle, true);
		//  色相（H）はオプティカルフローの角度
		//  彩度（S）は0～1に正規化したオプティカルフローの大きさ
		//  明度（V）は1
		Mat hsvPlanes[3];		
		hsvPlanes[0] = angle;
		normalize(magnitude, magnitude, 0, 1, NORM_MINMAX); // 正規化
		hsvPlanes[1] = magnitude;
		hsvPlanes[2] = Mat::ones(magnitude.size(), CV_32F);
		//  HSVを合成して一枚の画像にする
		Mat hsv;
		merge(hsvPlanes, 3, hsv);
		//  HSVからBGRに変換
		Mat flowBgr;
		cvtColor(hsv, flowBgr, cv::COLOR_HSV2BGR);

		// 表示
		cv::imshow("input", curr);
		cv::imshow("optical flow", flowBgr);

		// 前のフレームを保存
		prev = curr;

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


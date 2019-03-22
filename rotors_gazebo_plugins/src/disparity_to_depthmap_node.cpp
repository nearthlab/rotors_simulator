#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

using namespace std;
using namespace ros;
using namespace cv;

image_transport::Publisher pubDepthMap;
image_transport::Publisher pubDepthMapRaw;

float fDepthMax = 15.0f;
float fDepthMin = 1.5f;

void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& msgRaw)
{
  if(msgRaw->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0)
  {
    ROS_ERROR("Input type must be disparity=32FC1");
    return;
  }

  Mat mDisparity(msgRaw->image.height, msgRaw->image.width, CV_32FC1, const_cast<uchar*>(msgRaw->image.data.data()));
  cv::Size sizeVGA(msgRaw->image.width, msgRaw->image.height);

  Mat mDepth32f;
  Mat mDepth16u;
  mDepth32f = Mat::zeros(mDisparity.rows, mDisparity.cols, CV_32F);
  mDepth16u = Mat::zeros(mDisparity.rows, mDisparity.cols, CV_16U);  

  for (unsigned int i = 0; i < mDisparity.rows; i++)
  {
    for (unsigned int j = 0; j < mDisparity.cols; j++)
    {
      // baseline * focal / disparity
      float fVal = mDisparity.at<float>(i, j);
      float depth = 0.0f;
      if ((fVal > msgRaw->min_disparity)&&(fVal < msgRaw->max_disparity))
      {
        depth = (float)((msgRaw->T) * ((msgRaw->f) / (fVal + 1.0e-5)));

        if (depth > msgRaw->max_disparity)
          depth = msgRaw->max_disparity;
        else if (depth < fDepthMin)
          depth = fDepthMin;
      }
      else
        depth = msgRaw->max_disparity;
      mDepth32f.at<float>(i, j) = depth;
      mDepth16u.at<unsigned short>(i, j) = (unsigned short)(depth*1000.0f);
    }
  }

  Mat mDepth32fROI;
  Mat mDepth16uROI;
  int nShift = 32;
  mDepth32fROI = Mat::zeros(mDepth32f.rows, mDepth32f.cols, CV_32F);  
  mDepth16uROI = Mat::zeros(mDepth16u.rows, mDepth16u.cols, CV_16U);

  if (!mDepth32f.empty())
    mDepth32fROI = mDepth32f(Rect(nShift, 0, mDepth32f.cols - nShift, mDepth32f.rows));

  if (!mDepth16u.empty())
    mDepth16uROI = mDepth16u(Rect(nShift, 0, mDepth16u.cols - nShift, mDepth16u.rows));

  resize(mDepth32fROI, mDepth32fROI, sizeVGA);
  resize(mDepth16uROI, mDepth16uROI, sizeVGA);  

  cv_bridge::CvImage cvDepthMap32f(msgRaw->header, sensor_msgs::image_encodings::TYPE_32FC1, mDepth32fROI);
  sensor_msgs::Image msgDepthMap32f;
  cvDepthMap32f.toImageMsg(msgDepthMap32f);
  pubDepthMap.publish(msgDepthMap32f);

  cv_bridge::CvImage cvDepthMap16u(msgRaw->header, sensor_msgs::image_encodings::TYPE_16UC1, mDepth16uROI);
  sensor_msgs::Image msgDepthMap16u;
  cvDepthMap16u.toImageMsg(msgDepthMap16u);
  pubDepthMapRaw.publish(msgDepthMap16u);
}

int main (int argc, char **argv)
{
  // setting up ROS node
  init(argc, argv, "disparity_to_depthmap_node");

  NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pubDepthMap = it.advertise("/niv1/stereocam/depthmap", 1);
  pubDepthMapRaw = it.advertise("/niv1/stereocam/depthmap_raw", 1);  
  Subscriber subDisparity = nh.subscribe("/niv1/stereocam/disparity", 1, disparityImageCallback);

  // for initializing time info.
  Time::init();
 
  // Tell ROS how fast to run this node.
  Rate loop_rate(20);

  // main loop
  ROS_INFO("starting disparity_to_depthmap_node");
  while (ok())
  {
    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}  // end main()
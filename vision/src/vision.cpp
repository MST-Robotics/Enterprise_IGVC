/*
 * @file vision.cpp
 * @brief Take raw camera data and find lines in image
 * @author Ryan Loeffelman <rjlt3c@mst.edu>
 */

//File Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void prepareImage();
void edgeDetection();

cv_bridge::CvImagePtr frame;
image_transport::Subscriber sub;
image_transport::Publisher pub;

int main(int argc, char **argv)
{
	//Initialize all of the proper ROS thingys
	ros::init(argc, argv, "vision");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	pub = it.advertise("image_lines", 1);

	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	prepareImage();
	edgeDetection();

	pub.publish(frame->toImageMsg());

	return;
}

void prepareImage()
{
	//Down scale the image to half the size of the original image
	cv::pyrDown(frame->image, frame->image,
			cv::Size(frame->image.cols / 2, frame->image.rows / 2));

	//Converts the image to greyscale
	//CV_BGR2GREY = int 6
	//cv::cvtColor(frame->image, frame->image, 6, 0);
//ERROR couldn't convert to Greyscale

	//Blur Image
	cv::GaussianBlur(frame->image, frame->image, cv::Size(9, 9), 0, 0);

	//Threshold Image
	cv::threshold(frame->image, frame->image, 162, 255, CV_THRESH_BINARY);

	return;
}

void edgeDetection()
{
	//Skeletonize image
	cv::Mat element = cv::getStructuringElement(0, cv::Size(1, 1),
			cv::Point(0, 0));
	cv::Mat skel(frame->image.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;

	cv::erode(frame->image, eroded, element);
	cv::dilate(eroded, temp, element);
	cv::subtract(frame->image, temp, temp);
	cv::bitwise_or(skel, temp, skel);
	eroded.copyTo(frame->image);

	//Run Canny Edge Detector
	cv::Canny(frame->image, frame->image, 100, (100 * 3), 3);

	return;
}

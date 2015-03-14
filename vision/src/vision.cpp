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

/***********************************************************
 * @fn imageCallback(const sensor_msgs::ImageConstPtr& msg);
 * @brief gets the image from the camera
 * @pre has to have to get the raw camera data from the camera
 * @post Imports the camera Data, detects the lines,
 * and publishes the new image
 ***********************************************************/
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

/***********************************************************
 * @fn prepareImage();
 * @brief makes the line detection easier and more accurate
 * @pre must receive an image from the camera
 * @post pyramid scales the images, blurs the image,
 * thresholds the image
 ***********************************************************/
void prepareImage();

/***********************************************************
 * @fn edgeDetection();
 * @brief finds the lines in the image
 * @pre must be properly prepared to get good results
 * @post Skeletonizes the image, and runs a canny edge detector
 ***********************************************************/
void edgeDetection();

//A place to store the frame from the camera.
//Also the same Matrix that all of the changes are stored to
cv_bridge::CvImagePtr frame;

//A way to send and recieve the camera data from ROS
image_transport::Subscriber sub;
image_transport::Publisher pub;

int main(int argc, char **argv)
{
	//Initialize all of the proper ROS thingys
	ros::init(argc, argv, "vision");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	//Set the subscriber to listen to data from the camera1394 node
	sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	//Send data out with the tag image_lines
	pub = it.advertise("image_lines", 1);

	//Run as long as ROS is running and working
	while (ros::ok())
	{
		//Checks for updates from ROS
		ros::spinOnce();
	}

	return 0;
}

//Gets the image from the camera
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//Get the image from the camera, and store it in frame
	try
	{
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	//Throw an error message if the camera data isn't correct
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//Process the image
	prepareImage();
	edgeDetection();

	//Send the processed image back to ROS
	pub.publish(frame->toImageMsg());

	return;
}

//Makes the lines easier for canny to find
void prepareImage()
{
  //Scale the image down and back up, to smooth the image and blur the colors
	//Down scale the image to half the size of the original image
	cv::pyrDown(frame->image, frame->image,
			cv::Size(frame->image.cols / 2, frame->image.rows / 2));
	//Scale the image back to its origonal size
	cv::pyrUp(frame->image, frame->image,
			cv::Size(frame->image.cols * 2, frame->image.rows * 2));

	//Blur the image even further for better results
	//PARAMATERS
	cv::GaussianBlur(frame->image, frame->image, cv::Size(9, 9), 0, 0);

	//Threshold the image to reduce most unwanted noise
	//PARAMATERS
	cv::threshold(frame->image, frame->image, 162, 255, CV_THRESH_BINARY);

	return;
}

//Skeletonizes the image and runns canny
void edgeDetection()
{
	//A Mat of Size(1,1) filled with Points(0,0)
	cv::Mat element = cv::getStructuringElement(0, cv::Size(1, 1),
			cv::Point(0, 0));
	//A Mat the same size and type as the image, filled with Scalar(0)
	cv::Mat skel(frame->image.size(), CV_8UC1, cv::Scalar(0));
	//place to store the image so it doesn't msess up the real image
	cv::Mat temp;
	//place to store the eroded image
	cv::Mat eroded;

	//Reduce the amount of detail the image has
	cv::erode(frame->image, eroded, element);
	//Make the detail left in the image more pronounced
	cv::dilate(eroded, temp, element);
	//take the image, and take away the eroded and dilated image
	cv::subtract(frame->image, temp, temp);
	//Add all of the Mats together
	cv::bitwise_or(skel, temp, skel);
	//copy the edited Mats back onto frame
	eroded.copyTo(frame->image);

	//Run Canny Edge Detector
	//PARAMATERS
	cv::Canny(frame->image, frame->image, 100, (100 * 3), 3);

	return;
}

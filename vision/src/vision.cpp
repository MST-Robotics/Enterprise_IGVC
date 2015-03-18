/*
 * @file vision.cpp
 * @brief Take raw camera data and find lines in image
 * @author Ryan Loeffelman <rjlt3c@mst.edu>
 */

//File Includes
#include <iostream>
#include <fstream>
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

/***********************************************************
 * @fn callibration();
 * @brief callibrates the camera to compensate for fisheye
 * @pre Must get valid image from the camera
 * @post Returns the camera Matrix and the distance coeficents
 ***********************************************************/
void callibrate();

/***********************************************************
 * @fn fisheyeAdjust();
 * @brief undistores the fisheye camera lens
 * @pre Must get valid image from the camera
 * Must also have a valid camera matrix and distance coefficents
 * @post Returns an undistorted image
 ***********************************************************/
void fisheyeAdjust();

//A place to store the frame from the camera.
//Also the same Matrix that all of the changes are stored to
cv_bridge::CvImagePtr frame;

//A way to send and recieve the camera data from ROS
image_transport::Subscriber sub;
image_transport::Publisher pub;

//do you want to callibrate the image, or not
//If you want to do any image processing set this to 0
bool callibrateImage = 1;

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
		//Switch back to MONO8
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	//Throw an error message if the camera data isn't correct
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (callibrateImage)
	{
		//get the callibration settings for the camera
		callibrate();
	}
	else
	{
		//undistrote image
		//fisheyeAdjust();
		//Process the image
		prepareImage();
		edgeDetection();
	}

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

//undistores the image
void fisheyeAdjust()
{
	//create streams to text files
	std::ifstream cameraMatrix;
	std::ifstream distCoeffs;

	//Open the files
	cameraMatrix.open("cameraMatrix.txt");
	distCoeffs.open("distCoeffs.txt");

	//create the camera Matrix
	cv::Mat camMatrix;

	//cv::undistortImage(frame->image, frame->image,);

	//close the file streams
	cameraMatrix.close();
	distCoeffs.close();
	return;
}

//set the callibration settings for the Camera
void callibrate()
{
	//create streams to text files
	std::ofstream cameraMatrix;
	std::ofstream distCoeffs;

	//Open the files
	cameraMatrix.open("cameraMatrix.txt");
	distCoeffs.open("distCoeffs.txt");

	if(!cameraMatrix.is_open())
	{
		ROS_INFO("cameraMatrix.txt not opened");
	}
	if(!distCoeffs.is_open())
	{
		ROS_INFO("distCoeffs.txt not opened");
	}

	int numBoards = 1;

	int numRows = 5;
	int numCols = 4;
	int numSquares = (numRows * numCols);

	cv::Size boardSize(numRows, numCols);

	std::vector<std::vector<cv::Point3f> > objectPoints;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<cv::Point2f> corners;

	int successes = 0;

	std::vector<cv::Point3f> obj;
	for (int i = 0; i < numSquares; ++i)
	{
		obj.push_back(cv::Point3f(i / numCols, i % numCols, 0.0f));
	}

	ROS_INFO("This works!");

	while (successes < numBoards)
	{
		bool found = cv::findChessboardCorners(frame->image, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH);
		if(found)
		{
			ROS_INFO("This works!!");
			cv::cornerSubPix(frame->image, corners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(frame->image, boardSize, corners, found);

			imagePoints.push_back(corners);
			objectPoints.push_back(obj);
			ROS_INFO("This works!!!");

			++successes;
		}
	}

	cv::Mat cameraMat = cv::Mat(3,3,CV_32FC1);
	cv::Mat distanceCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;

	cameraMat.ptr<float>(0)[0] = 1;
	cameraMat.ptr<float>(1)[1] = 1;

	ROS_INFO("This works!!!!");

	cv::calibrateCamera(objectPoints, imagePoints, frame->image.size(), cameraMat, distanceCoeffs, rvecs, tvecs);

	ROS_INFO("This works!!!!!");

	cameraMatrix << "this Works";
	distCoeffs << "this Works also";
	ROS_INFO("This works!!!!!!");
	//close the file streams
	cameraMatrix.close();
	distCoeffs.close();

	callibrateImage = 0;
	ROS_INFO("This worksFinal!");
	return;
}

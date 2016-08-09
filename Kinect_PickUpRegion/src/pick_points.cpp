/*
 * image_converter.cpp
 *
 *  Created on: Aug 2, 2016
 *      Author: chentao
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const std::string OPENCV_RGB_WINDOW = "RGB Image window";
static const std::string OPENCV_DEPTH_WINDOW = "Depth Image window";
cv::Rect rectangle_;
bool drawBox = false;
std::vector<cv::Rect> rects;
std::vector<cv::Point> centroids;
static void mouseHandle(int event, int x, int y, int flags, void* param)
{
	cv::Mat& image = *(cv::Mat*)param;
	switch (event)
	{
	case cv::EVENT_MOUSEMOVE:
	{
		rectangle_.width = x - rectangle_.x;
		rectangle_.height = y - rectangle_.y;
	}
	break;
	case cv::EVENT_LBUTTONDOWN:
	{
		rectangle_ = cv::Rect(x, y, 0, 0);
		drawBox = true;
	}
	break;
	case cv::EVENT_LBUTTONUP:
	{
		drawBox = false;
		if (rectangle_.width < 0)
		{

			rectangle_.x += rectangle_.width;
			rectangle_.width *= -1;
		}
		if (rectangle_.height < 0)
		{
			rectangle_.y += rectangle_.height;
			rectangle_.height *= -1;
		}
		rects.push_back(rectangle_);
		cv::Point centerPos(rectangle_.x + rectangle_.width / 2.0, rectangle_.y + rectangle_.height / 2.0);
		centroids.push_back(centerPos);
	}
	}

}

class ImageConverter
{
	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_rgb_;
	image_transport::Subscriber image_sub_depth_;

public:
	cv::Mat rgbImage_;
	cv::Mat depthImage_;
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_rgb_ = it_.subscribe("rgbinput", 1,
		                               &ImageConverter::rgbCb, this);
		image_sub_depth_ = it_.subscribe("depthinput", 1,
		                                 &ImageConverter::depthCb, this);

		cv::namedWindow(OPENCV_RGB_WINDOW);
		cv::namedWindow(OPENCV_DEPTH_WINDOW);
		cv::setMouseCallback(OPENCV_RGB_WINDOW, mouseHandle, (void*)&rgbImage_);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_DEPTH_WINDOW);
		cv::destroyWindow(OPENCV_RGB_WINDOW);
	}


	void rgbCb(const sensor_msgs::ImageConstPtr& msg)
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
		cv_ptr->image.copyTo(rgbImage_);
		// // Draw an example circle on the video stream
		// if (rgbImage_.rows > 60 && rgbImage_.cols > 60)
		// 	cv::circle(rgbImage_, cv::Point(55, 55), 5, CV_RGB(255, 0, 0));
		cv::RNG rng(12345);
		for (int i = 0; i < rects.size(); i++)
		{

			cv::Rect tempRect = rects[i];
			cv::rectangle(rgbImage_, tempRect.tl(), tempRect.br(),
			              cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);
		}
		for (int i = 0; i < centroids.size(); i++)
		{
			cv::Point center = centroids[i];
			uint16_t depth = depthImage_.at<uint16_t>(center.x, center.y);
			float depthF = depth / 10.0;
			std::ostringstream ss;
			ss << "Depth" << std::fixed << std::setprecision(2) <<depthF;
			ss << "cm";
			std::string text = ss.str();
			// std::string text = "Depth: ";
			// text += boost::lexical_cast<std::string>(depthF);
			// text += "cm";
			cv::putText(rgbImage_, text, center, cv::FONT_HERSHEY_SIMPLEX , 1.0,
			            cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2);
		}


		// Update GUI Window
		cv::imshow(OPENCV_RGB_WINDOW, rgbImage_);
		cv::waitKey(10);

	}
	void depthCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg);
		}
		catch (cv_bridge::Exception& e)
		{

			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv_ptr->image.copyTo(depthImage_);
		// std::cout << "=============================================================" << std::endl;
		// std::cout << "Depth" << std::endl;
		// for (int i = 50; i < 60; i++)
		// {
		// 	for (int j = 50; j < 60; j++)
		// 	{
		// 		std::cout << (depthImage_.at< cv::Vec<uint16_t, 1> >(i, j)) << " ";
		// 	}
		// 	std::cout << std::endl;
		// }
		// std::cout << std::endl;
		// Update GUI Window
		cv::imshow(OPENCV_DEPTH_WINDOW, cv_ptr->image);
		cv::waitKey(10);
	}



};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;

	while (ros::ok())
	{

		ros::spinOnce();
		if (drawBox)
		{
			cv::Mat tempImage;
			ic.rgbImage_.copyTo(tempImage);
			cv::RNG rng(12345);
			for (int i = 0; i < rects.size(); i++)
			{

				cv::Rect tempRect = rects[i];
				cv::rectangle(tempImage, tempRect.tl(), tempRect.br(),
				              cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)),	3);
			}
			for (int i = 0; i < centroids.size(); i++)
			{
				cv::Point center = centroids[i];
				uint16_t depth = ic.depthImage_.at<uint16_t>(center.x, center.y);
				float depthF = depth / 10.0;
				std::ostringstream ss;
				ss << "Depth" << std::fixed << std::setprecision(2) <<depthF;
				ss << "cm";
				std::string text = ss.str();
				// std::string text = "Depth: ";
				// text += boost::lexical_cast<std::string>(depthF);
				// text += "cm";
				cv::putText(tempImage, text, center, cv::FONT_HERSHEY_SIMPLEX , 1.0,
				            cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2);
			}

			cv::rectangle(tempImage, rectangle_.tl(), rectangle_.br(),
			              cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)),3);
			cv::imshow(OPENCV_RGB_WINDOW, tempImage);
			cv::waitKey(10);
		}
	}

	return 0;
}



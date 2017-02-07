#include "stdafx.h"
#include "VirtualWhiteboard.h"

int minH = 0, maxH = 20, minS = 30, maxS = 150, minV = 60, maxV = 255;
cv::Mat fram;
int count1 = 0;

void CallbackFunc(int event, int x, int y, int flags, void* userdata)
{
	cv::Mat RGB = fram(cv::Rect(x, y, 1, 1));
	cv::Mat HSV;
	cv::cvtColor(RGB, HSV, CV_BGR2HSV);
	cv::Vec3b pixel = HSV.at<cv::Vec3b>(0, 0);
	if (event == cv::EVENT_LBUTTONDBLCLK) // on double left clcik
	{
		std::cout << "Click" << std::endl;
		int h = pixel.val[0];
		int s = pixel.val[1];
		int v = pixel.val[2];
		minH = h - 10 > 0 ? h - 10 : 0;
		maxH = h + 10 < 179 ? h + 10 : 179;
		minS = s - 10 > 0 ? s - 10 : 0;
		maxS = s + 10 < 255 ? s + 10 : 255;
		minV = v - 10 > 0 ? v - 10 : 0;
		maxV = v + 10 < 255 ? v + 10 : 255;
		maxV = maxV + 1;
	}
}

/////////////// Detect Finger
int Finger()
{
	cv::VideoCapture cap(0);
	const char* windowName = "Fingertip detection";
	cv::namedWindow(windowName);
	cv::setMouseCallback(windowName, CallbackFunc, NULL);
	int inAngleMin = 200, inAngleMax = 300, angleMin = 180, angleMax = 359, lengthMin = 10, lengthMax = 80;
	cv::createTrackbar("Inner angle min", windowName, &inAngleMin, 360);
	cv::createTrackbar("Inner angle max", windowName, &inAngleMax, 360);
	cv::createTrackbar("Angle min", windowName, &angleMin, 360);
	cv::createTrackbar("Angle max", windowName, &angleMax, 360);
	cv::createTrackbar("Length min", windowName, &lengthMin, 100);
	cv::createTrackbar("Length max", windowName, &lengthMax, 100);
	while (1)
	{
		cap >> fram;
		cv::Mat hsv;
		cv::cvtColor(fram, hsv, CV_BGR2HSV);
		cv::inRange(hsv, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), hsv);
		// Pre processing
		int blurSize = 5;
		int elementSize = 5;
		cv::medianBlur(hsv, hsv, blurSize);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Point(elementSize, elementSize));
		cv::dilate(hsv, hsv, element);

		cv::imshow("HSV", hsv);

		cv::imshow(windowName, fram);
		if (cv::waitKey(30) >= 0) break;
	}

	return 0;
}
// VirtualWhiteboard.cpp : définit le point d'entrée pour l'application console.
//

#include "stdafx.h"

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

using namespace cv;
using namespace std;

int _______main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 170;
	int iHighH = 179;

	int iLowS = 150;
	int iHighS = 255;

	int iLowV = 60;
	int iHighV = 255;

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	int iLastX = 0;
	int iLastY = 0;
	int tmpiLastX = -1;
	int tmpiLastY = -1;
	RNG rng(12345);

	//Capture a temporary image from the camera
	Mat imgTmp;
	cap.read(imgTmp);

	//Create a black image with the size as the camera output
	Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);;


	while (true)
	{
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video
		cv::flip(imgOriginal, imgOriginal, 1);



		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

																									  //morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (removes small holes from the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		//findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		/*Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3);
		for (int i = 0; i< contours.size(); i++)
		{
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		}*/
		/*
		#Find contours in the threshold image
		contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)


		#Finding contour with maximum area and store it as best_cnt
		max_area =0for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > max_area:
		max_area = area
		best_cnt = cnt

		#Finding centroids of best_cnt and draw a circle there
		M = cv2.moments(best_cnt)
		cx,cy =int(M['m10']/M['m00']), int(M['m01']/M['m00'])
		cv2.circle(frame,(cx,cy),10,255,-1)*/


		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;

			//cout << (posX - iLastX)*(posX - iLastX) << endl;
			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0 && (posX - iLastX)*(posX - iLastX) > 10 && (posY - iLastY)*(posY - iLastY) > 10)
			{
				//Draw a red line from the previous point to the current point
				if (posX > 550 && posY > 350) {
					line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 255, 255, 150), 3);
				}
				else
				{
					line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
				}
				//std::cout << posX << " - " << posY << std::endl;


				iLastX = posX;
				iLastY = posY;
			}

		}

		imshow("Thresholded Image", imgThresholded); //show the thresholded image

													 //font = cv2.FONT_HERSHEY_SIMPLEX
													 //cv2.putText(img, 'OpenCV', (10, 500), font, 4, (255, 255, 255), 2, cv2.LINE_AA)

		imgOriginal = imgOriginal + imgLines;
		imshow("Original", imgOriginal); //show the original image

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;
}

// Ball
int main(int argc, char* argv[])
{
	// Default capture size - 640x480
	CvSize size = cvSize(640, 480);
	// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
	/*CvCapture* capture = cvCaptureFromCAM(0);
	if (!capture)
	{
	fprintf(stderr,  "ERROR: capture is NULL \n" );
	getchar();
	return -1;
	}*/
	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	// Create a window in which the captured images will be presented
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("EdgeDetection", CV_WINDOW_AUTOSIZE);
	// Detect a red ball
	int iLowH = 0;
	int iHighH = 30;
	//Scalar hsv_min = Scalar(150, 84, 130, 0);
	//Scalar hsv_max = Scalar(358, 256, 255, 0);
	Scalar hsv_min;
	Scalar hsv_max;
	Scalar greenLower = Scalar(29, 86, 6);
	Scalar greenUpper = Scalar(64, 255, 255);
	Mat  hsv_frame;
	Mat  thresholded;
	Mat  thresholded2;
	Mat  thresholdedFinal;
	vector<Point> pts;
	vector<Point> clickedPts;
	Point pt, lastPt = Point(0,0);
	bool detection = false;
	bool alreadyClicked = false;
	bool alreadyChangedColor = false;
	RNG rng(12345);

	int timeNoDetection = 0;
	int currentTimeNoDetection = 0;

	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255), 150);

	while (1)
	{
		// Get one frame
		Mat frame;
		cap.read(frame);
		if (frame.empty())
		{
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}

		//frame.resize(600);
		// Pour le grain
		medianBlur(frame, frame, 3);
		// Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
		cvtColor(frame, hsv_frame, CV_BGR2HSV);
		// Filter out colors which are out of range.
		inRange(hsv_frame, Scalar(0, 100, 100, 0), Scalar(10, 255, 255, 0), thresholded);
		inRange(hsv_frame, Scalar(170, 100, 100, 0), Scalar(180, 255, 255, 0), thresholded2);
		addWeighted(thresholded, 1.0, thresholded2, 1.0, 0.0, thresholdedFinal);
		// Memory for hough circles
		vector<Vec3f> storage;
		

		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		// hough detector works better with some smoothing of the image
		GaussianBlur(thresholdedFinal, thresholdedFinal, Size(9, 9), 2, 2);

		HoughCircles(thresholdedFinal, storage, CV_HOUGH_GRADIENT, 2, thresholdedFinal.rows / 4, 100, 60, 1, 400);
		/*Mat result;
		findContours(thresholded, storage, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		//max(storage, contourArea(storage));
		Point2f center;
		float radius;
		minEnclosingCircle(storage, center, radius);*/

		if (storage.size() == 0) {
			currentTimeNoDetection++;
			cout << currentTimeNoDetection << endl;
			if (currentTimeNoDetection > 10) {
				detection = false;
				//currentTimeNoDetection = 0;
				alreadyClicked = false;
			}
		}
		if (storage.size() > 0 && pts.size() > 0)
		{
			if (!alreadyClicked && currentTimeNoDetection > 10 && currentTimeNoDetection < 30) {
				cout << " Clic !!!!!!!!!!!!!!!!!!!!" << pts[pts.size()-1].x << pts[pts.size()-1].y << endl;
				clickedPts.push_back(Point(pts[pts.size()].x, pts[pts.size()].y));
				alreadyClicked = true;
			}
			currentTimeNoDetection = 0;
			detection = true;
		}

		for (int i = 0; i < storage.size(); i++)
		{
			Vec3f p = storage[i];
			//printf("Ball!x = %f y = %f r = %f\n\r", p[0], p[1], p[2]);
			circle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),
				3, CV_RGB(0, 255, 0), -1, 8, 0);
			circle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),
				cvRound(p[2]), CV_RGB(255, 0, 0), 3, 8, 0);

			pt = Point(cvRound(p[0]), cvRound(p[1]));

			//cout << ((pt.x - lastPt.x)) << "  " << sqrt((pt.y - lastPt.y)) << endl;
			if (cv::pow((pt.x - lastPt.x), 2.0) > 20 && cv::pow((pt.y - lastPt.y), 2.0) > 20) {
				lastPt = pt;
				pts.push_back(pt);
			}
		}
		
		if (pts.size() > 0)
			if(pts[pts.size()-1].x > 550 && pts[pts.size()-1].y > 350) {
			if (!alreadyChangedColor) {
				color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255), 150);
				alreadyChangedColor = true;
			}
		}
		else {
			alreadyChangedColor = false;
		}

		for (size_t i = 1; i < pts.size(); i++)
		{
			int thichness = int(10/(pts.size()-i)+2);

			line(frame, pts[i - 1], pts[i], color, thichness);
			if (pts[i].x > 550 && pts[i].y > 350) {

			}
			else
			{


			}
		}
		if (pts.size() > 40) {
			pts.erase(pts.begin());
		}

		if(clickedPts.size() > 0 )
			for (size_t i = 0; i < clickedPts.size()-1; i+=2)
			{
				rectangle(frame, Point(cvRound(clickedPts[i].x), cvRound(clickedPts[i].y)), Point(cvRound(clickedPts[i + 1].x), cvRound(clickedPts[i + 1].y)), Scalar(255, 0, 0, 100), 3, 8, 0);
			}

		imshow("Camera", frame); // Original stream with detected ball overlay
		imshow("HSV", hsv_frame); // Original stream in the HSV color space
		imshow("After Color Filtering", thresholdedFinal); // The stream after color filtering
		createTrackbar("LowH", "Camera", &iLowH, 179); //Hue (0 - 179)
		createTrackbar("HighH", "Camera", &iHighH, 179);

		//storage.clear();

		// Do not release the frame!
		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
		//remove higher bits using AND operator

		if ((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();

	return 0;
}




// Test avec une image
int ___main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	// Get one frame
	Mat frame;


	Mat img, gray;
	//imwrite("truc", )
	img = imread("test.JPG");

	//cap.read(img);
	cvtColor(img, gray, CV_BGR2GRAY);
	// smooth it, otherwise a lot of false circles may be detected
	GaussianBlur(gray, gray, Size(9, 9), 2, 2);
	vector<Vec3f> circles;
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
		2, gray.rows / 4, 200, 100);
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// draw the circle center
		circle(img, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// draw the circle outline
		circle(img, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}
	namedWindow("circles", 1);
	imshow("circles", img);

	cvWaitKey(10000);

	return 0;
}
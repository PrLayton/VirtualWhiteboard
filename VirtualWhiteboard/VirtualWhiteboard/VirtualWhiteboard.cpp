#include "stdafx.h"

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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

// Projet de détection d'un marqueur (balle de couleur) comme outil de dessin et d'interaction
int main(int argc, char* argv[])
{
	// Default capture size - 640x480
	//CvSize size = cvSize(640, 480);

	VideoCapture cap(0); //capture the video from webcam
	// WebCam externe : cap(1)

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	// Tend au maximum
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	// Les fenêtres
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("After Color Filtering", CV_WINDOW_AUTOSIZE);

	// Pour la balle rouge
	Scalar hsv_min;
	Scalar hsv_max;
	// Données de la doc
	//Scalar(150, 84, 130, 0);
	//Scalar(358, 256, 255, 0);
	Scalar greenLower = Scalar(29, 86, 6);
	Scalar greenUpper = Scalar(64, 255, 255);
	// Hue modulable dans l'interface
	int iLowH = 0;
	int iHighH = 10;

	Mat  hsv_frame;
	Mat  thresholded;
	Mat  thresholded2;
	Mat  thresholdedFinal;

	// Positions récuperés
	vector<Point> pts;
	vector<Point> clickedPts;
	Point pt, lastPt = Point(0,0);
	Point tmpPt;

	bool detection = false;
	bool alreadyClicked = false;
	bool alreadyChangedColor = false;

	RNG rng(12345);
	// Couleur de dessin
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255), 150);

	// Pour le clic
	int timeNoDetection = 0;
	int currentTimeNoDetection = 0;

	while (1)
	{
		Mat frame;
		cap.read(frame);
		if (frame.empty())
		{
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}

		// Pour le grain
		medianBlur(frame, frame, 3);
		// HSV plus facile à traiter
		cvtColor(frame, hsv_frame, CV_BGR2HSV);
		//Orange
		inRange(hsv_frame, Scalar(0, 100, 100, 0), Scalar(10, 255, 255, 0), thresholded);
		inRange(hsv_frame, Scalar(170, 100, 100, 0), Scalar(180, 255, 255, 0), thresholded2);
		//Blue
		//inRange(hsv_frame, Scalar(60, 40, 40, 0), Scalar(100, 200, 200, 0), thresholded);
		//inRange(hsv_frame, Scalar(60, 40, 40, 0), Scalar(100, 255, 100, 0), thresholded2);
		//inRange(hsv_frame, Scalar(110, 100, 100, 0), Scalar(120, 200, 200, 0), thresholded2);//100 255
		// Fusion avec poids
		addWeighted(thresholded, 1.0, thresholded2, 1.0, 0.0, thresholdedFinal);

		// Amélioration de la qualité du résultat
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		GaussianBlur(thresholdedFinal, thresholdedFinal, Size(9, 9), 2, 2);

		// Stockage des points de reconnaissance
		vector<Vec3f> storage;
		HoughCircles(thresholdedFinal, storage, CV_HOUGH_GRADIENT, 2, thresholdedFinal.rows / 4, 100, 60, 1, 400);
		// Au méthode avec les contours
		/*Mat result;
		findContours(thresholded, storage, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		//max(storage, contourArea(storage));
		Point2f center;
		float radius;
		minEnclosingCircle(storage, center, radius);*/

		// Traitement du clic avec la balle
		if (storage.size() == 0) {
			if (currentTimeNoDetection == 0 && pts.size() > 0) {
				tmpPt = Point(pts[pts.size()].x, pts[pts.size()].y);
			}
			currentTimeNoDetection++;
			cout << "temps sans marqueur : " << currentTimeNoDetection << endl;
			if (currentTimeNoDetection > 15) {
				detection = false;
				//currentTimeNoDetection = 0;
				alreadyClicked = false;
			}
		}
		if (storage.size() > 0 && pts.size() > 0)
		{
			if (!alreadyClicked && currentTimeNoDetection > 15 && currentTimeNoDetection < 30) {
				cout << " Clic !!!!!!!!!!!!!!!!!!!!" << pts[pts.size()-1].x << pts[pts.size()-1].y << endl;
				clickedPts.push_back(tmpPt);
				alreadyClicked = true;
			}
			if (currentTimeNoDetection >= 40) {
				pts.clear();
			}
			currentTimeNoDetection = 0;
			detection = true;
		}

		// traitement des points détectés
		for (int i = 0; i < storage.size(); i++)
		{
			Vec3f p = storage[i];
			/*circle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),
				3, CV_RGB(0, 255, 0), -1, 8, 0);*/
			circle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),
				cvRound(p[2]), CV_RGB(255, 0, 0, 10), 1, 8, 0);

			pt = Point(cvRound(p[0]), cvRound(p[1]));

			//cout << ((pt.x - lastPt.x)) << "  " << sqrt((pt.y - lastPt.y)) << endl;
			// Pour éviter les tremblements
			if (cv::pow((pt.x - lastPt.x), 2.0) > 10 && cv::pow((pt.y - lastPt.y), 2.0) > 10) {
				lastPt = pt;
				// Stockage des points
				pts.push_back(pt);
			}
		}
		
		// Changement de couleur
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
		}
		if (pts.size() > 30) {
			pts.erase(pts.begin());
		}

		if(clickedPts.size() > 0 )
			for (size_t i = 0; i < clickedPts.size()-1; i+=2)
			{
				rectangle(frame, Point(cvRound(clickedPts[i].x), cvRound(clickedPts[i].y)), Point(cvRound(clickedPts[i + 1].x), cvRound(clickedPts[i + 1].y)), Scalar(255, 0, 0, 100), 3, 8, 0);
			}

		// Affichage des résultats
		//imshow("HSV", hsv_frame); 
		imshow("After Color Filtering", thresholdedFinal);
		imshow("Camera", frame);
		cv::moveWindow("Camera", 10, 10);
		//createTrackbar("LowH", "Camera", &iLowH, 179); //Hue (0 - 179)
		//createTrackbar("HighH", "Camera", &iHighH, 179);

		//storage.clear();

		if ((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();
	cap.release();

	return 0;
}


/////////////// Projet de calibration des couleurs
int __main(int argc, char** argv)
{
	VideoCapture cap(0);

	if (!cap.isOpened()) 
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE);

	int iLowH = 100;
	int iHighH = 179;

	int iLowS = 100;
	int iHighS = 255;

	int iLowV = 100;
	int iHighV = 255;

	// Hue
	createTrackbar("LowH", "Control", &iLowH, 179);
	createTrackbar("HighH", "Control", &iHighH, 179);

	// Saturation
	createTrackbar("LowS", "Control", &iLowS, 255);
	createTrackbar("HighS", "Control", &iHighS, 255);

	// Value
	createTrackbar("LowV", "Control", &iLowV, 255);
	createTrackbar("HighV", "Control", &iHighV, 255);

	Mat imgTmp;
	cap.read(imgTmp);

	Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);;

	while (true)
	{
		Mat imgOriginal;

		cap.read(imgOriginal);
		cv::flip(imgOriginal, imgOriginal, 1);

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

		imshow("Thresholded Image", imgThresholded); 

		imgOriginal = imgOriginal + imgLines;
		imshow("Original", imgOriginal); 

		if (waitKey(10) == 27)
		{
			cout << "Quit" << endl;
			break;
		}
	}

	return 0;
}

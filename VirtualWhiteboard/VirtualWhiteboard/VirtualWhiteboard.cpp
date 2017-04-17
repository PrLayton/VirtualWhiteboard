#include "stdafx.h"
#include "VirtualWhiteboard.h"

#include <iostream>
#include <cmath>
#include "opencv2\opencv.hpp"

int src = 0;

int cvPaint();

int main(int argc, char* argv[])
{
	//Finger();
	//Calibration();
	//DrawScreen();
	Detection();
	//cvPaint();
}

// Fonction pour détecter l'écran virtuelle - Marqueur est un circle vert
vector<Point> DetectScreen(Mat image, vector<Point> &lastCenters, double minDist)
{
	// Filtrer la couleur VERTE - Marqueur pour l'écran
	Mat green_hue_image;
	inRange(image, Scalar(50, 90, 50), Scalar(90, 255, 255), green_hue_image);
	//inRange(image, Scalar(20, 90, 0), Scalar(100, 255, 255), green_hue_image);
	GaussianBlur(green_hue_image, green_hue_image, Size(9, 9), 2, 2);
	imshow("GREEN Image", green_hue_image);

	// Stockage des points de reconnaissance
	vector<Vec3f> circles;
	HoughCircles(green_hue_image, circles, CV_HOUGH_GRADIENT, 2, green_hue_image.rows / 4, 100, 60, 1, 400);

	vector<Point> centers;
	// traitement des points détectés
	for (int i = 0; i < circles.size(); i++)
	{
		Vec3f p = circles[i];
		if (cvRound(p[2]) >= 15)
			centers.push_back(Point(cvRound(p[0]), cvRound(p[1])));
	}

	if (centers.size() == 2)
	{
		if (lastCenters.size() != 2)
			lastCenters = centers;
		else
		{
			double dist0 = cv::norm(centers[0] - lastCenters[0]);
			double dist1 = cv::norm(centers[1] - lastCenters[1]);
			if (dist0 < minDist && dist1 < minDist)
			//if (norm(lastCenters[0] - lastCenters[1]) > 80000)
				centers = lastCenters;
			else
				lastCenters = centers;
		}
		return centers;
	}
	return lastCenters;
}

// Projet de détection d'un marqueur (balle de couleur) comme outil de dessin et d'interaction
int Detection()
{
	// Default capture size - 640x480
	//CvSize size = cvSize(640, 480);

	VideoCapture cap(src); //capture the video from webcam
	// WebCam externe : cap(1)

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	// Taille camera
	int WC = 640, HC = 480;
	// Tend au maximum
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WC);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HC);

	// Les fenêtres
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("After Color Filtering", CV_WINDOW_AUTOSIZE);

	// Pour la balle rouge
	Scalar hsv_min;
	Scalar hsv_max;
	Scalar greenLower = Scalar(29, 86, 6);
	Scalar greenUpper = Scalar(64, 255, 255);

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

	// Resolution ecran
	int WS = 1600, HS = 900;
	// Video-projecteur
	// 800 * 600
	Mat screen(Size(WS, HS), CV_8UC3);

	vector<Point> lastCenters;
	lastCenters.push_back(Point(0, 0));
	lastCenters.push_back(Point(0, 0));
	Point centerDecalage(0,0);
	bool detected = false;
	vector<Point> centers;

	// Ecran de dessin
	Rect rec(Point(0, 0), Point(WS, HS));
	//rectangle(screen, rec, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
	// Coloration pour la detection
	rectangle(screen, rec, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);

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
		
		// Flipper l'image
		//flip(frame, frame, 1);
		// Pour le grain
		medianBlur(frame, frame, 3);
		// HSV plus facile à traiter
		cvtColor(frame, hsv_frame, CV_BGR2HSV);

		// Detection du ou des marqueurs
		//vector<Point> centers = DetectScreen(hsv_frame, lastCenters, 200);
		// Filtrer la couleur VERTE - Marqueur pour l'écran
		Mat green_hue_image;
		inRange(hsv_frame, Scalar(20, 100, 100), Scalar(60, 255, 255), green_hue_image);

		if (!detected) {
			centers.clear();

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			inRange(hsv_frame, Scalar(20, 100, 100), Scalar(60, 255, 255), green_hue_image);
			//inRange(image, Scalar(20, 90, 0), Scalar(100, 255, 255), green_hue_image);
			//GaussianBlur(green_hue_image, green_hue_image, Size(9, 9), 2, 2);
			erode(green_hue_image, green_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(green_hue_image, green_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(green_hue_image, green_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(green_hue_image, green_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			// Contours
			Canny(green_hue_image, green_hue_image, 100, 100 * 2, 3);
			// Find contours
			findContours(green_hue_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			// Approximate contours to polygons + get bounding rects
			vector<vector<Point> > contours_poly(contours.size());
			vector<Rect> boundRect(contours.size());

			// Enveloppe convexe
			for (int i = 0; i < contours.size(); i++)
			{
				approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
				boundRect[i] = boundingRect(Mat(contours_poly[i]));
			}

			// Draw polygonal contour + bonding rects
			//Mat drawing = Mat::zeros(green_hue_image.size(), CV_8UC3);
			for (int i = 0; i < contours.size(); i++)
			{
				Scalar color = Scalar(255, 255, 255);
				drawContours(green_hue_image, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
				// Plusieurs detectes, on prend que le premier
				if (i == 0)
					rectangle(green_hue_image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			}

			// Des qu'on en a un, alors c'est bon
			if (contours.size() > 0) {
				centers.push_back(boundRect[0].tl());
				centers.push_back(boundRect[0].br());
				rectangle(screen, rec, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
				detected = true;
			}

			// A faire
			//contours.clear();
			//contours_poly.clear();
			//boundRect.clear();
		}

		// Debug
		imshow("GREEN Image", green_hue_image);

		int W = 1, H = 1;
		if (centers.size() == 2)
		{
			centerDecalage = centers[0].y < centers[1].y ? centers[0] : centers[1];
			W = centers[0].x > centers[1].x ? centers[0].x - centers[1].x : centers[1].x - centers[0].x;
			H = centers[0].y > centers[1].y ? centers[0].y - centers[1].y : centers[1].y - centers[0].y;
		}
		//inRange(hsv_frame, Scalar(100, 100, 90), Scalar(130, 255, 255), thresholdedFinal);
		inRange(hsv_frame, Scalar(10, 100, 30), Scalar(40, 255, 255), thresholdedFinal);
		//Orange
		//inRange(hsv_frame, Scalar(0, 100, 100, 0), Scalar(10, 255, 255, 0), thresholded);
		//inRange(hsv_frame, Scalar(160, 100, 100, 0), Scalar(180, 255, 255, 0), thresholded2);
		//Blue
		//inRange(hsv_frame, Scalar(60, 40, 40, 0), Scalar(100, 200, 200, 0), thresholded);
		//inRange(hsv_frame, Scalar(60, 40, 40, 0), Scalar(100, 255, 100, 0), thresholded2);
		//inRange(hsv_frame, Scalar(110, 100, 100, 0), Scalar(120, 200, 200, 0), thresholded2);//100 255
		// Fusion avec poids
		//addWeighted(thresholded, 1.0, thresholded2, 1.0, 0.0, thresholdedFinal);

		// Amélioration de la qualité du résultat
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		GaussianBlur(thresholdedFinal, thresholdedFinal, Size(9, 9), 2, 2);

		// Stockage des points de reconnaissance
		vector<Vec3f> storage;
		HoughCircles(thresholdedFinal, storage, CV_HOUGH_GRADIENT, 2, thresholdedFinal.rows / 4, 100, 60, 10, 400);
		// Autre méthode avec les contours
		/*Mat result;
		findContours(thresholded, storage, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		//max(storage, contourArea(storage));
		Point2f center;
		float radius;
		minEnclosingCircle(storage, center, radius);*/

		// Traitement du clic avec la balle
		if (storage.size() == 0) {
			if (currentTimeNoDetection == 0 && pts.size() > 0) {
				tmpPt = Point(pts[pts.size()-1].x, pts[pts.size()-1].y);
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
				cvRound(p[2]), CV_RGB(255, 0, 0), 1, 8, 0);

			pt = Point(cvRound(p[0]), cvRound(p[1]));

			//cout << ((pt.x - lastPt.x)) << "  " << sqrt((pt.y - lastPt.y)) << endl;
			// Pour éviter les tremblements
			if (pow((pt.x - lastPt.x), 2.0) > 5 && pow((pt.y - lastPt.y), 2.0) > 5) {
				lastPt = pt;
				// Stockage des points
				pts.push_back(pt);
			}
		}
		
		// Changement de couleur
		if (pts.size() > 0)
			if(pts[pts.size()-1].x > 450 && pts[pts.size()-1].y > 250) {
			if (!alreadyChangedColor) {
				color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255), 150);
				alreadyChangedColor = true;
			}
		}
		else {
			alreadyChangedColor = false;
		}

		int nb = 0;
		for (int i = 1; i < pts.size(); i++)
		{
			int thickness = int(10/(pts.size()-i)+2);
			Point p1 = convertCoord(pts[i - 1], centerDecalage, W, H, WS, HS);
			Point p2 = convertCoord(pts[i], centerDecalage, W, H, WS, HS);
			//Point p1 = pts[i - 1];
			//Point p2 = pts[i];
			//Point p1(pts[i - 1].x * screen.size().width / WC, pts[i - 1].y * screen.size().height / HC);
			//Point p2(pts[i].x * screen.size().width / WC, pts[i].y * screen.size().height / HC);
			line(screen, p1, p2, color, thickness);
			//line(screen, pts[i - 1], pts[i], color, thichness);
			nb++;
		}
		cout << "Number loop = " << nb << endl;
		//cout << "W = " << W << " H = " << H << " centerx = " << centerDecalage.x << " centery = " << centerDecalage.y << endl;

		if (pts.size() > 30) {
			cout << pts.size() << endl;
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
		if (screen.size().width > 0 && screen.size().height > 0)
		{
			cvNamedWindow("ECRAN VIRTUELLE", CV_WINDOW_NORMAL);
			cvSetWindowProperty("ECRAN VIRTUELLE", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			//line(screen, Point(0, 0), Point(screen.size().width, screen.size().height), CV_RGB(255, 0, 0), 2, 8, 0);
			imshow("ECRAN VIRTUELLE", screen);
		}
		imshow("Camera", frame);
		moveWindow("Camera", 10, 10);

		//storage.clear();

		if ((cvWaitKey(10) & 255) == 27) break;

	}

	cvDestroyAllWindows();
	cap.release();

	return 0;
}

Point convertCoord(Point p, Point decalage, int W, int H, int WS, int HS)
{
	Point temp = p - decalage;
	int x = cvRound(temp.x * WS * 1.0 / (double)W);
	x = x > 0 ? x : 0;
	int y = cvRound(temp.y * HS * 1.0 / (double)H);
	y = y > 0 ? y : 0;
	return Point(x, y);
}

// Fonction pour dessiner l'écran
int DrawScreen()
{
	VideoCapture cap(src);

	if (!cap.isOpened())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE);

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 50;
	int iHighS = 255;

	int iLowV = 50;
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

	//Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);;

	vector<Point> lastCenters;
	while (true)
	{
		Mat imgOriginal;

		cap.read(imgOriginal);
		flip(imgOriginal, imgOriginal, 1);
		// Eliminer bruit
		medianBlur(imgOriginal, imgOriginal, 3);

		// Convertir à HSV
		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		// Filtrer la couleur ROUGE
		Mat lowerRedThres;
		Mat upperRedThres;
		inRange(imgHSV, Scalar(0, 100, 100), Scalar(10, 255, 255), lowerRedThres);
		inRange(imgHSV, Scalar(160, 100, 100), Scalar(179, 255, 255), upperRedThres);
		Mat red_hue_image;
		addWeighted(lowerRedThres, 1.0, upperRedThres, 1.0, 0.0, red_hue_image);
		GaussianBlur(red_hue_image, red_hue_image, Size(9, 9), 2, 2);
		//imshow("RED Image", red_hue_image);

		// Filtrer la couleur BLEUE
		Mat blue_hue_image;
		inRange(imgHSV, Scalar(100, 100, 90), Scalar(130, 255, 255), blue_hue_image);
		GaussianBlur(blue_hue_image, blue_hue_image, Size(9, 9), 2, 2);
		imshow("BLUE Image", blue_hue_image);

		// Filtrer la couleur VERTE
		Mat green_hue_image;
		inRange(imgHSV, Scalar(50, 90, 50), Scalar(90, 255, 255), green_hue_image);
		GaussianBlur(green_hue_image, green_hue_image, Size(9, 9), 2, 2);
		imshow("GREEN Image", green_hue_image);

		// Stockage des points de reconnaissance
		vector<Vec3f> circles;
		HoughCircles(green_hue_image, circles, CV_HOUGH_GRADIENT, 2, green_hue_image.rows / 4, 100, 60, 1, 400);

		vector<Point> centers;
		// traitement des points détectés
		for (int i = 0; i < circles.size(); i++)
		{
			Vec3f p = circles[i];
			if (cvRound(p[2]) >= 5)
			{
				centers.push_back(Point(cvRound(p[0]), cvRound(p[1])));
				circle(imgOriginal, cvPoint(cvRound(p[0]), cvRound(p[1])),
					cvRound(p[2]), CV_RGB(255, 0, 0), 1, 8, 0);
			}
		}

		if (centers.size() == 2)
		{
			if (lastCenters.size() != 2)
				lastCenters = centers;
			else
			{
				double dist0 = cv::norm(centers[0] - lastCenters[0]);
				double dist1 = cv::norm(centers[1] - lastCenters[1]);
				cout << "dist0 = " << dist0 << " & dist1 = " << dist1 << endl;
				cout << "c0 = " << centers[0] << " & c1 = " << centers[1] << endl;
				cout << "LC0 = " << lastCenters[0] << " & LC1 = " << lastCenters[1] << endl;
				if (dist0 < 20 && dist1 < 20)
					centers = lastCenters;
				else
					lastCenters = centers;
			}
			Rect rec(centers[0], centers[1]);
			rectangle(imgOriginal, rec, CV_RGB(255, 255, 255), CV_FILLED, 8, 0);
		}

		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
		//imshow("Thresholded Image", imgThresholded); 
		//imgOriginal = imgOriginal + imgLines;
		imshow("Original", imgOriginal);

		if (waitKey(10) == 27)
		{
			cout << "Quit" << endl;
			break;
		}
	}

	return 0;
}

/////////////// Projet de calibration des couleurs
int Calibration()
{
	VideoCapture cap(src);

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



/*******************************************************************
TCV3501 - Computer Vision and Image Understanding
Farshid Tavakolizadeh (email@farshid.ws) http://farshid.ws
Copyrights goes to Mr. Wong Ya Ping (CVLab06)
Year: 18/1/2012
********************************************************************/
/********************************************************************
Noise Handing:
- using area to ignore the noise
- using magnitute limit to skip long distance points
- using 5x5 median filter to decrease the background noise

Control features:
- choosing line colors
- changing line thickness based on Y coordinates
- clearing the screen + saving the image
- exiting the program
********************************************************************/

using namespace std;

#define blue  CV_RGB(0,0,255)
#define green CV_RGB(0,255,0)
#define red   CV_RGB(255,0,0)
#define white CV_RGB(255,255,255)
#define black CV_RGB(0,0,0)

void ClearScreen(IplImage* imgScribble, IplImage* imgDrawing)
{
	cvSet(imgScribble, black);
	cvSet(imgDrawing, white);
}

IplImage* GetThresholdedImage(IplImage* img, CvScalar& lowerBound, CvScalar& upperBound)
{
	// Convert the image into an HSV image
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);

	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

	cvInRangeS(imgHSV, lowerBound, upperBound, imgThreshed);

	cvReleaseImage(&imgHSV);
	return imgThreshed;
}

int cvPaint()
{
	// controls
	double area_limit = 700;
	//CvScalar lowerBound = cvScalar(20, 100, 100);  // yellow
	//CvScalar upperBound = cvScalar(30, 255, 255);
	//CvScalar lowerBound = cvScalar(150, 100, 100);  // pink
	//CvScalar upperBound = cvScalar(180, 255, 255);
	CvScalar lowerBound = cvScalar(30, 100, 100);  // green
	CvScalar upperBound = cvScalar(50, 255, 255);

	// defaults
	int lineThickness = 2;
	CvScalar lineColor = blue;


	CvCapture* capture = 0;
	capture = cvCaptureFromCAM(0);
	if (!capture)
	{
		cout << "Could not initialize capturing...\n";
		return -1;
	}

	// This image holds the "scribble" data...
	// the tracked positions of the pointer object
	IplImage* imgScribble = NULL;


	IplImage* imgColorPanel = 0;
	imgColorPanel = cvLoadImage("cvPaint.panel", CV_LOAD_IMAGE_COLOR); // load the panel image. (This is a png image, not designed/included in the source code!) 
	if (!imgColorPanel)
	{
		cout << "cvPaint.panel is not found !!! \n";
		return -1;
	}

	IplImage* imgDrawing = 0;
	imgDrawing = cvCreateImage(cvSize(cvQueryFrame(capture)->width, cvQueryFrame(capture)->height),
		cvQueryFrame(capture)->depth,     //Bit depth per channel
		3  //number of channels
	);
	cvSet(imgDrawing, white);

	CvFont font, fontbig;
	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1, .6, 0, 2, CV_AA);
	cvInitFont(&fontbig, CV_FONT_HERSHEY_COMPLEX, 3, .6, 0, 3, CV_AA);

	int confirm_close = 10, confirm_clear = 20; // counters for clear and exit confirmation
	char buffer[50]; // buffer for cvPutText
	int image_num = 0; // to keep track of image numbers for saving
	int posX = 0;
	int posY = 0;

	while (true)
	{
		IplImage* frame = 0;
		frame = cvQueryFrame(capture);
		if (!frame)
			break;
		cvFlip(frame, NULL, 1); // flip the frame to overcome mirroring problem



								// If this is the first frame, we need to initialize it
		if (imgScribble == NULL)
			imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);

		// Median filter to decrease the background noise
		cvSmooth(frame, frame,
			CV_MEDIAN,
			5, 5 //parameters for filter, in this case it is filter size
		);


		// Holds the thresholded image (tracked color -> white, the rest -> black)
		IplImage* imgThresh = GetThresholdedImage(frame, lowerBound, upperBound);



		
		// Calculate the moments to estimate the position of the object
		CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
		cvMoments(imgThresh, moments, 1);


		// The actual moment values
		double moment10 = cvGetSpatialMoment(moments, 1, 0);
		double moment01 = cvGetSpatialMoment(moments, 0, 1);
		double area = cvGetCentralMoment(moments, 0, 0);

		// Holding the last and current positions
		int lastX = posX;
		int lastY = posY;

		posX = 0;
		posY = 0;


		if (moment10 / area >= 0 && moment10 / area < 1280 && moment01 / area >= 0 && moment01 / area < 1280
			&& area>area_limit /* to control the limit */)
		{
			posX = (int)moment10 / area;
			posY = (int)moment01 / area;
		}



		CvPoint cvpoint = cvPoint(150, 30); // location of the text
		if (posX < 90 && posY > 400) // clear
		{
			lineColor = white; // white color works as eraser
			cvPutText(frame, "Eraser selected.", cvpoint, &font, white);
			sprintf_s(buffer, "Clearing the screen in %d", confirm_clear); // count-down for clearing the screen
			cvPutText(frame, buffer, cvPoint(150, 70), &font, red);
			confirm_clear--;
			if (confirm_clear < 0) // confirm in 10 frames before clearing
			{
				confirm_clear = 20;
				sprintf_s(buffer, "d0%d.jpg", image_num++);
				cvSaveImage(buffer, imgDrawing); // save the frame into an image
				ClearScreen(imgScribble, imgDrawing);
				cvPutText(frame, "Cleared the screen.", cvPoint(150, 110), &font, white);
			}
		}
		else if (posX  > 540 && posY > 360)  // blue
		{
			lineColor = blue;
			cvPutText(frame, "Blue color selected.", cvpoint, &font, blue);
		}

		else if (posX  > 540 && posY > 200 && posY < 280) // green
		{
			lineColor = green;
			cvPutText(frame, "Green color selected.", cvpoint, &font, green);
		}

		else if (posX  > 540 && posY < 120) // red
		{
			lineColor = red;
			cvPutText(frame, "Red color selected.", cvpoint, &font, red);
		}

		else if (posX > 0 && posX  < 90 && posY > 0 && posY < 120) // exit
		{
			sprintf_s(buffer, "EXITING in %d", confirm_close);
			cvPutText(frame, buffer, cvpoint, &font, red);
			confirm_close--;
			if (confirm_close < 0) // confirm in 10 frames before exit
				break;
		}
		else if (posX < 90 && posY > 130 && posY < 390) // line thickness
		{
			lineThickness = 6 - (posY / 60 - 1);  // change the thickness of line from 1 - 5 based on posY
		}


		sprintf_s(buffer, "%d", lineThickness);
		cvPutText(frame, buffer, cvPoint(40, 255), &fontbig, lineColor);

		double diff_X = lastX - posX;
		double diff_Y = lastY - posY;
		double magnitude = sqrt(pow(diff_X, 2) + pow(diff_Y, 2));
		// We want to draw a line only if its a valid position
		//if(lastX>0 && lastY>0 && posX>0 && posY>0)
		if (magnitude > 0 && magnitude < 100 && posX > 120 && posX<530)
		{
			// Draw a line from the previous point to the current point
			cvLine(imgDrawing, cvPoint(posX, posY), cvPoint(lastX, lastY), lineColor, lineThickness, CV_AA);
		}


		//cout << "position = " << posX << "\t" <<  posY << "\t";
		//cout << "moment = " << moment10 << "\t" <<  moment01 << "\n";
		//cout << "d->" << magnitude << endl;
		//cout << "area = " << area << endl;

		// Add the scribbling image and the frame...
		cvAdd(imgDrawing, imgScribble, imgDrawing);

		//cvAdd(imgDrawing, image2, imgDrawing);

		// Combine everything in frame
		cvAnd(frame, imgDrawing, frame);
		cvAnd(imgColorPanel, frame, frame);

		cvShowImage("Threshold", imgThresh);
		cvShowImage("Drawing", imgDrawing);
		cvShowImage("Video", frame);


		int c = cvWaitKey(10);
		if (c == 27)  //ESC key
			break;
		//else if(c==49) // 1 key


		cvReleaseImage(&imgThresh);
		delete moments;
	}

	cvReleaseCapture(&capture);
	cvReleaseImage(&imgColorPanel);
	cvReleaseImage(&imgScribble);

	return 0;
}

/* Documentation
#################################################################################################################
IplImage* cvLoadImage( const char* filename, int iscolor=CV_LOAD_IMAGE_COLOR );
#define CV_LOAD_IMAGE_COLOR       1
#define CV_LOAD_IMAGE_GRAYSCALE   0
#define CV_LOAD_IMAGE_UNCHANGED  -1
#################################################################################################################
void cvLine(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int lineType=8, int shift=0);
img – The image
pt1 – First point of the line segment
pt2 – Second point of the line segment
Type of the line:
8 - (or omitted) 8-connected line.
4 - 4-connected line.
CV_AA - antialiased line.
shift – Number of fractional bits in the point coordinates
#################################################################################################################
void cvInitFont(CvFont* font, int fontFace, double hscale, double vscale, double shear=0, int thickness=1, int lineType=8)
Initializes font structure.Parameters:	font – Pointer to the font structure initialized by the function
fontFace –

Font name identifier. Only a subset of Hershey fonts http://sources.isc.org/utils/misc/hershey-font.txt are supported now:
CV_FONT_HERSHEY_SIMPLEX - normal size sans-serif font
CV_FONT_HERSHEY_PLAIN - small size sans-serif font
CV_FONT_HERSHEY_DUPLEX - normal size sans-serif font (more complex than


CV_FONT_HERSHEY_SIMPLEX)
CV_FONT_HERSHEY_COMPLEX - normal size serif font
CV_FONT_HERSHEY_TRIPLEX - normal size serif font (more complex than CV_FONT_HERSHEY_COMPLEX)
CV_FONT_HERSHEY_COMPLEX_SMALL - smaller version of CV_FONT_HERSHEY_COMPLEX
CV_FONT_HERSHEY_SCRIPT_SIMPLEX - hand-writing style font
CV_FONT_HERSHEY_SCRIPT_COMPLEX - more complex variant of CV_FONT_HERSHEY_SCRIPT_SIMPLEX
The parameter can be composited from one of the values above and an optional CV_FONT_ITALIC flag, which indicates italic or oblique font.param hscale:	Horizontal scale. If equal to 1.0f, the characters have the original width depending on the font type. If equal to 0.5f, the characters are of half the original width.
param vscale:	Vertical scale. If equal to 1.0f, the characters have the original height depending on the font type. If equal to 0.5f, the characters are of half the original height.
param shear:	Approximate tangent of the character slope relative to the vertical line. A zero value means a non-italic font, 1.0f means about a 45 degree slope, etc.
param thickness:
Thickness of the text strokes
param lineType:
#################################################################################################################
void cvSmooth(const CvArr* src, CvArr* dst, int smoothtype=CV_GAUSSIAN, int param1=3, int param2=0, double param3=0, double param4=0)¶
http://opencv.willowgarage.com/documentation/c/image_filtering.html#cvSmooth
#################################################################################################################
void cvAnd( const CvArr* A, const CvArr* B, CvArr* C, const CvArr* mask=0 );
http://www710.univ-lyon1.fr/~bouakaz/OpenCV-0.9.5/docs/ref/OpenCVRef_BasicFuncs.htm
#################################################################################################################
*/
#include "stdafx.h"
#include "VirtualWhiteboard.h"

int main(int argc, char* argv[])
{
	//Finger();
	//Calibration();
	//DrawScreen();
	Detection();
}

// Fonction pour d�tecter l'�cran virtuelle - Marqueur est un circle vert
vector<Point> DetectScreen(Mat image, vector<Point> &lastCenters, double minDist)
{
	// Filtrer la couleur VERTE - Marqueur pour l'�cran
	Mat green_hue_image;
	inRange(image, Scalar(50, 90, 50), Scalar(90, 255, 255), green_hue_image);
	GaussianBlur(green_hue_image, green_hue_image, Size(9, 9), 2, 2);
	imshow("GREEN Image", green_hue_image);

	// Stockage des points de reconnaissance
	vector<Vec3f> circles;
	HoughCircles(green_hue_image, circles, CV_HOUGH_GRADIENT, 2, green_hue_image.rows / 4, 100, 60, 1, 400);

	vector<Point> centers;
	// traitement des points d�tect�s
	for (int i = 0; i < circles.size(); i++)
	{
		Vec3f p = circles[i];
		if (cvRound(p[2]) >= 5)
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
				centers = lastCenters;
			else
				lastCenters = centers;
		}
		return centers;
	}
	return lastCenters;
}

// Projet de d�tection d'un marqueur (balle de couleur) comme outil de dessin et d'interaction
int Detection()
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

	// Les fen�tres
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("After Color Filtering", CV_WINDOW_AUTOSIZE);

	// Pour la balle rouge
	Scalar hsv_min;
	Scalar hsv_max;
	// Donn�es de la doc
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

	// Positions r�cuper�s
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

	Mat screen;
	vector<Point> lastCenters;
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
		flip(frame, frame, 1);
		// Pour le grain
		medianBlur(frame, frame, 3);
		// HSV plus facile � traiter
		cvtColor(frame, hsv_frame, CV_BGR2HSV);

		vector<Point> centers = DetectScreen(hsv_frame, lastCenters, 200);
		if (centers.size() == 2)
		{
			Rect rec(centers[0], centers[1]);
			rectangle(frame, rec, CV_RGB(255, 255, 255), CV_FILLED, 8, 0);
			screen = frame(rec).clone();
		}

		inRange(hsv_frame, Scalar(100, 100, 90), Scalar(130, 255, 255), thresholdedFinal);
		//Orange
		//inRange(hsv_frame, Scalar(0, 100, 100, 0), Scalar(10, 255, 255, 0), thresholded);
		//inRange(hsv_frame, Scalar(160, 100, 100, 0), Scalar(180, 255, 255, 0), thresholded2);
		//Blue
		//inRange(hsv_frame, Scalar(60, 40, 40, 0), Scalar(100, 200, 200, 0), thresholded);
		//inRange(hsv_frame, Scalar(60, 40, 40, 0), Scalar(100, 255, 100, 0), thresholded2);
		//inRange(hsv_frame, Scalar(110, 100, 100, 0), Scalar(120, 200, 200, 0), thresholded2);//100 255
		// Fusion avec poids
		//addWeighted(thresholded, 1.0, thresholded2, 1.0, 0.0, thresholdedFinal);

		// Am�lioration de la qualit� du r�sultat
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		GaussianBlur(thresholdedFinal, thresholdedFinal, Size(9, 9), 2, 2);

		// Stockage des points de reconnaissance
		vector<Vec3f> storage;
		HoughCircles(thresholdedFinal, storage, CV_HOUGH_GRADIENT, 2, thresholdedFinal.rows / 4, 100, 60, 1, 400);
		// Au m�thode avec les contours
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

		// traitement des points d�tect�s
		for (int i = 0; i < storage.size(); i++)
		{
			Vec3f p = storage[i];
			/*circle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),
				3, CV_RGB(0, 255, 0), -1, 8, 0);*/
			circle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),
				cvRound(p[2]), CV_RGB(255, 0, 0, 10), 1, 8, 0);

			pt = Point(cvRound(p[0]), cvRound(p[1]));

			//cout << ((pt.x - lastPt.x)) << "  " << sqrt((pt.y - lastPt.y)) << endl;
			// Pour �viter les tremblements
			if (pow((pt.x - lastPt.x), 2.0) > 10 && pow((pt.y - lastPt.y), 2.0) > 10) {
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
			line(screen, pts[i - 1], pts[i], color, thichness);
		}
		if (pts.size() > 30) {
			pts.erase(pts.begin());
		}

		if(clickedPts.size() > 0 )
			for (size_t i = 0; i < clickedPts.size()-1; i+=2)
			{
				rectangle(frame, Point(cvRound(clickedPts[i].x), cvRound(clickedPts[i].y)), Point(cvRound(clickedPts[i + 1].x), cvRound(clickedPts[i + 1].y)), Scalar(255, 0, 0, 100), 3, 8, 0);
			}

		// Affichage des r�sultats
		//imshow("HSV", hsv_frame); 
		imshow("After Color Filtering", thresholdedFinal);
		if (screen.size().width > 0 && screen.size().height > 0)
		{
			cvNamedWindow("ECRAN VIRTUELLE", CV_WINDOW_AUTOSIZE);
			//cvSetWindowProperty("ECRAN VIRTUELLE", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			imshow("ECRAN VIRTUELLE", screen);

			//imshow("ECRAN VIRTUELLE", screen);
		}
		imshow("Camera", frame);
		moveWindow("Camera", 10, 10);
		//createTrackbar("LowH", "Camera", &iLowH, 179); //Hue (0 - 179)
		//createTrackbar("HighH", "Camera", &iHighH, 179);

		//storage.clear();

		if ((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();
	cap.release();

	return 0;
}

// Fonction pour dessiner l'�cran
int DrawScreen()
{
	VideoCapture cap(0);

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

		// Convertir � HSV
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
		// traitement des points d�tect�s
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
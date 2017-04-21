#include "stdafx.h"
#include "VirtualWhiteboard.h"
#include "Interface.h"

int src = 0;

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
	bool detected = true;
	vector<Point> centers;

	// Ecran de dessin
	Rect rec(Point(0, 0), Point(WS, HS));
	//rectangle(screen, rec, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
	// Coloration pour la detection
	rectangle(screen, rec, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);

	Point2i p1(0,0), p2(0,0);

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
		inRange(hsv_frame, Scalar(30, 100, 100), Scalar(70, 255, 255), green_hue_image);
		//inRange(hsv_frame, Scalar(40, 0, 100), Scalar(70, 255, 255), green_hue_image); // vidéoprojecteur

		if (!detected) {
			centers.clear();

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			//inRange(hsv_frame, Scalar(20, 100, 100), Scalar(60, 255, 255), green_hue_image);
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
			int maxcontour = 0, maxcontourIndex = 0;
			for (int i = 0; i < contours.size(); i++) {

				if (contourArea(contours[i]) > maxcontour) {
					maxcontour = contourArea(contours[i]);
					maxcontourIndex = i;
				}
			}

			if (contours.size() > 0 && maxcontour > 10) {
				Scalar color = Scalar(255, 255, 255);
				drawContours(green_hue_image, contours_poly, maxcontourIndex, color, 1, 8, vector<Vec4i>(), 0, Point());
				// Plusieurs detectes, on prend que le premier
				//rectangle(green_hue_image, boundRect[maxcontourIndex].tl(), boundRect[maxcontourIndex].br(), color, 2, 8, 0);
				p1 = boundRect[maxcontourIndex].tl();
				p2 = boundRect[maxcontourIndex].br();

				// Des qu'on en a un, alors c'est bon
				if (contours.size() > 0) {
					centers.push_back(boundRect[0].tl());
					centers.push_back(boundRect[0].br());
					rectangle(screen, rec, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
					detected = true;
				}

			}

			// A faire
			//contours.clear();
			//contours_poly.clear();
			//boundRect.clear();
		}

		rectangle(green_hue_image, p1, p2, color, 2, 8, 0);

		// Debug
		imshow("GREEN Image", green_hue_image);

		int W = 1, H = 1;
		if (centers.size() == 2)
		{
			centerDecalage = centers[0].y < centers[1].y ? centers[0] : centers[1];
			W = abs(centers[0].x - centers[1].x);
			H = abs(centers[0].y - centers[1].y);
		}
		//inRange(hsv_frame, Scalar(100, 100, 90), Scalar(130, 255, 255), thresholdedFinal);
		inRange(hsv_frame, Scalar(30, 50, 50), Scalar(70, 255, 255), thresholdedFinal);
		//inRange(hsv_frame, Scalar(30, 100, 100), Scalar(50, 255, 255), thresholdedFinal);
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

		//if ((cvWaitKey(10) & 255) == 27) break;
		switch (waitKey(10))
		{
		case 27: //'esc' key has been pressed, exit program.
			return 0;
		case 100: //d
			rectangle(screen, rec, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
			detected = false;
			break;
		}
	}

	cvDestroyAllWindows();
	cap.release();

	return 0;
}

// Fonction pour convertir les coordonées de caméra vers 
// les coordonées locales de l'écran virtuel
// centerDecalage: coordonnées du sommet en haut à gauche de l'écran dans l'espace du caméra
// W & S : dimensions de l'écran perçues par la caméra (normalement plus petites que W & S)
// WS & HS : résolution réelle de l'écran
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
	VideoCapture cap(SOURCE);

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
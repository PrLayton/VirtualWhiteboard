#include "stdafx.h"

#include <Windows.h>
#include <chrono>
#include "VirtualWhiteboard.h"
#include "Interface.h"
#include "ScreenCapture.h"

using namespace std;
using namespace std::chrono;
using namespace cv;

// Eléments de l'UI
vector<Scalar> colors = { red, green, blue, white };
vector<int> thicknesses = { 1, 2, 3, 4, 5 };
double scaleUI = 20.0f;
bool showUI = false;

// Projet de détection d'un marqueur avec UI
int DetectionUI()
{
	VideoCapture cap(SOURCE); //capture the video from webcam
	ScreenCapture mainWindow(1);
	ScreenCapture monitor(0);

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the webcam" << endl;
		return -1;
	}

	// Résolution camera
	int WC = 800, HC = 600;
	// Résolution écran virtuel
	int WS = -1, HS = -1;

	// Tend au maximum/résolution forcée
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WC);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HC);

	// Defintion de la taille de l'ecran
	Mat drawing;
	if (doubleScreen)
	{
		WS = monitor.width;
		HS = monitor.height;
		drawing = Mat(Size(WS, HS), CV_8UC4);
	}
	else
	{
		WS = WC;
		HS = HC;
		drawing = Mat(Size(WS, HS), CV_8UC3);
	}
	// Ecran de dessin - NOIR
	drawing = black;
	// Couleur de dessin - éléments de l'UI
	Scalar color = red;
	int thickness = 3;
	vector<int> xBounds, yBounds;
	createBoundsUI(WS, HS, xBounds, yBounds);
	Mat ui = drawUI(WS, HS, color, thickness);
	int size = (int)(WS / scaleUI);

	Mat thresholdedFinal;
	Mat frame;
	Mat fore;

	Mat  hsv_frame;
	cv::Vec3b pixelsColor = cv::Vec3b(30, 120, 120);

	Mat frame1, frame2;
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1, grayImage2;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImage;

	CalibrationData calibrData;
	Mat green_hue_image;
	bool detected = true;
	Point pt = Point(-1, -1), lastPoint = Point(-1, -1);

	Mat calibScreen(Size(WS, HS), CV_8UC3);
	calibScreen = green;

	

	cap.read(frame1);

	// Pour controler la detection de la peau
	/*namedWindow("Control", CV_WINDOW_AUTOSIZE);
	int iLowH = 0, iLowW = 0;
	createTrackbar("LowH", "Control", &iLowH, 255);
	createTrackbar("LowW", "Control", &iLowW, 255);*/

	int cmpt = 0;
	auto sum = 0;
	while (1)
	{
		Mat screen;
		// Capture d'écran
		auto t1 = high_resolution_clock::now();
		if (doubleScreen)
			//monitor >> screen;
			monitor.capture(screen);
		auto t2 = high_resolution_clock::now();
		
		// Capture d'un frame de caméra
		cap.read(frame);
		
		auto duration = duration_cast<microseconds>(t2 - t1).count();
		sum += duration;
		cmpt++;
		if (cmpt == 20)
		{
			cout << sum / cmpt << endl;
			cmpt = 0;
			sum = 0;
		}

		if (frame.empty())
		{
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}

		// Si webcam interne => FLIP
		//flip(frame, frame, 1);
		
		if (!detected) 
		{
			cvNamedWindow("ECRAN CALIBRATION", CV_WINDOW_NORMAL);
			cvSetWindowProperty("ECRAN CALIBRATION", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			cvMoveWindow("ECRAN CALIBRATION", -WS, 0);
			imshow("ECRAN CALIBRATION", calibScreen);

			calibrData.HS = HS;
			calibrData.WS = WS;
			
			detected = Calibrate(calibrData, frame); 
			if (detected == true) 
				cvDestroyWindow("ECRAN CALIBRATION");
			else
				continue;
		}

		/*Mat tmpMat;
		cvtColor(frame, tmpMat, CV_BGR2HSV);
		inRange(tmpMat, Scalar(30, 100, 100), Scalar(70, 255, 255), green_hue_image);
		rectangle(green_hue_image, calibrData.p1, calibrData.p2, color, 2, 8, 0);
		// Debug de la calibration
		imshow("GREEN Image", green_hue_image);*/

		////// DETECTION DE DEPLACEMENT
		if (!alternativeMethod) 
		{	
			vector<vector<Point> > contours;
			cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
			// cpoie de la seconde frame
			cap.read(frame2);
			// conversion en niveau de gris
			cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);
			// construit une image qui represente in niveau d'intensité
			cv::absdiff(grayImage1, grayImage2, differenceImage);
			// seuil l'image avec un niveau d'intensité
			cv::threshold(differenceImage, thresholdImage, 20, 255, THRESH_BINARY);

			Mat copy;
			//bitwise_not(thresholdImage, thresholdImage);
			frame.copyTo(copy, thresholdImage);
			//hsv_frame = dst2;

			// Pour le grain
			//medianBlur(frame, frame, 3);
			// HSV plus facile à traiter
			cvtColor(copy, hsv_frame, CV_BGR2HSV);
			//inRange(hsv_frame, Scalar(20, 100, 100), Scalar(60, 255, 255), fore);
			//inRange(hsv_frame, Scalar(112, 0, 100), Scalar(152, 100, 255), fore);

			inRange(hsv_frame, Scalar(max(0, pixelsColor[0] - 20), max(0, pixelsColor[1] - 100), max(0, pixelsColor[2] - 100)), Scalar(min(255, pixelsColor[0] + 20), min(255, pixelsColor[1] + 100), min(255, pixelsColor[2] + 100)), fore);

			// Mettre en valeur les lignes et retirer du bruit
			erode(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			GaussianBlur(fore, fore, Size(5, 5), 0, 0);

			// Trouver les contours les plus importants
			// Sans hierarchie, sans compression
			findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			int maxcontour = 0, maxcontourIndex = 0;
			for (int i = 0; i < contours.size(); i++)

			{
				if (contourArea(contours[i]) > maxcontour)
				{
					maxcontour = contourArea(contours[i]);
					maxcontourIndex = i;
				}
			}

			int index = maxcontourIndex;
			if (contours.size()> 0 && contourArea(contours[index]) >= 500)
			{
				Mat contoursMat(contours[index]);

				// Enveloppe convexe du contour sous forme de points
				vector<vector<Point>> hulls(1);
				convexHull(contoursMat, hulls[0], false);
				// Dessine tous les contours
				drawContours(frame, hulls, -1, cv::Scalar(0, 255, 0), 2);

				// Enveloppe convexe du contour
				/*vector<vector<Point> > hulls(1);
				vector<vector<int> > hullsI(1);
				convexHull(Mat(tcontours[0]), hulls[0], false);
				convexHull(Mat(tcontours[0]), hullsI[0], false);
				drawContours(frame, hulls, -1, cv::Scalar(0, 255, 0), 2);*/

				// Rectangle englobant minimum
				RotatedRect rect = minAreaRect(contoursMat);

				if (hulls.size() > 0)
				{
					// Rectangle englobant
					Point2f rect_points[4]; rect.points(rect_points);
					// Coordonnées du centre du rectangle englobant
					Point2f p;
					for (int j = 0; j < 4; j++)
						p += rect_points[j];
					p /= 4;
					// Dessin
					circle(frame, p, 5, Scalar(255, 255, 255), 5);
					// Conversion
					pt = Point(cvRound(p.x), cvRound(p.y));
					pt = convertCoord(pt, calibrData);

					// Dessin du rectangle
					for (int j = 0; j < 4; j++)
						line(frame, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
				}
				else
				{
					pt = Point(-1, -1);
					lastPoint = Point(-1, -1);
				}

			}
			else
			{
				pt = Point(-1, -1);
				lastPoint = Point(-1, -1);
			}

			//imshow("Frame", frame);
			imshow("Move detection", thresholdImage);
			imshow("Color skin detection", fore);
		}
		else
		{
			// DECTECTION PAR TRACKER VERT - STICKER
			// HSV plus facile à traiter
			cvtColor(frame, hsv_frame, CV_BGR2HSV);
			// Pour le grain
			medianBlur(hsv_frame, hsv_frame, 3);

			//inRange(hsv_frame, Scalar(40, 90, 150), Scalar(90, 255, 255), thresholdedFinal); // VERT
			inRange(hsv_frame, Scalar(160, 100, 100), Scalar(179, 255, 255), thresholdedFinal); // ROSE

			GaussianBlur(thresholdedFinal, thresholdedFinal, Size(9, 9), 2, 2);
			imshow("FILTRE", thresholdedFinal);
			// Stockage des points de reconnaissance
			vector<Vec3f> storage;
			HoughCircles(thresholdedFinal, storage, CV_HOUGH_GRADIENT, 2, thresholdedFinal.rows / 4, 100, 60, 10, 400);

			// traitement des points détectés
			if (storage.size() >= 1)
			{
				int k = 0;
				for (k = 0; k < storage.size(); k++)
					if (storage[k][2] >= 50)
						break;
				if (k >= storage.size())
					continue;
				//cout << "RADIUS = " << storage[k][2] << endl;
				Vec3f p = storage[k];
				pt = Point(cvRound(p[0]), cvRound(p[1]));
				//pt = convertCoord(pt, calibrData);
				circle(frame, pt, cvRound(p[2]), red);
			}
			else
			{
				pt = Point(-1, -1);
				lastPoint = Point(-1, -1);
			}
		}

		if (pt.x > 0 && pt.y > 0)
		{
			// Détection de l'UI des couleurs
			if (pt.y <= yBounds[0] && pt.y >= yBounds[1])
			{
				if (pt.x <= xBounds[0] && pt.x >= xBounds[xBounds.size() - 1])
				{
					color = ui.at<Vec3b>(pt.y, pt.x);
					ui = drawUI(WS, HS, color, thickness);
				}
			}
			// Détection de l'UI des thickness
			else if (pt.x <= xBounds[0] && pt.x >= xBounds[1])
			{
				for (int i = 0; i < thicknesses.size(); i++)
				{
					int radius = size / (2 * (thicknesses[thicknesses.size() - 1] + 1 - thicknesses[i]));
					if (pt.y >= yBounds[i + 2] - radius && pt.y <= yBounds[i + 2] + radius)
					{
						thickness = thicknesses[i];
						ui = drawUI(WS, HS, color, thickness);
						break;
					}
				}				
			}
			// Dessiner le segment
			if (lastPoint.x != -1 && pt.x <= xBounds[xBounds.size() - 1])
				if (dist(pt, lastPoint) > 25 && showUI)
					line(drawing, lastPoint, pt, color, thickness, CV_AA);
			lastPoint = pt;
		}

		if (doubleScreen)
		{
			add(screen, drawing, screen);
			add(screen, ui, screen);
			cvNamedWindow("ECRAN VIRTUEL", CV_WINDOW_NORMAL);
			cvSetWindowProperty("ECRAN VIRTUEL", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			cvMoveWindow("ECRAN VIRTUEL", -WS, 0);
			imshow("ECRAN VIRTUEL", screen);
		}
		else
		{
			add(frame, drawing, frame);
			add(frame, ui, frame);
		}
		imshow("Camera", frame);

		// LE SWITCH DOIT ETRE ICI POUR QUE LA BOUCLE WHILE FONCTIONNE
		switch (waitKey(10))
		{
		case 27: //'esc' key has been pressed, exit program.
			return 0;
		case 99: // 'c' : calibration screen
			drawing = black;
			detected = false;
			break;
		case 100: // 'd'
			cap.read(frame1);
			break;
		case 110: // 'n' : new drawing
			drawing = black;
			break;
		case 117: // 'u'
			showUI = !showUI;
			ui = drawUI(WS, HS, color, thickness);
			break;
		case 116: //'t' has been pressed. this will toggle tracking
			int h = 0, s = 0, v = 0;
			cv::Vec3b tmppixel;
			for (int i = -2; i < 2; i++)
			{
				for (int y = -2; y < 2; y++)
				{
					tmppixel = hsv_frame.at<cv::Vec3b>(hsv_frame.rows / 2 + i, hsv_frame.cols / 2 + y);
					h += tmppixel[0];
					s += tmppixel[1];
					v += tmppixel[2];
				}
			}
			pixelsColor = cv::Vec3b(h / 16, s / 16, v / 16);
			cout << h / 16 << " " << s / 16 << " " << v / 16 << endl;
			drawing = black;
			break;
		}

	}

	cvDestroyAllWindows();
	cap.release();

	return 0;
}

// Calculer des coordonnées à utiliser pour distinguer les éléments de l'UI dans l'espace
void createBoundsUI(int WS, int HS, vector<int> &xBounds, vector<int> &yBounds)
{
	int size = (int)(WS / scaleUI);
	int x = WS - size;
	int y = HS - size;
	for (int i = 0; i < colors.size(); i++)
	{
		xBounds.push_back(x);
		x -= size;
	}
	xBounds.push_back(x);
	yBounds.push_back(y);
	yBounds.push_back(y - size);

	x = (int)(WS - size * 1.5f);
	y = HS - size * 4;
	for (int i = 0; i < thicknesses.size(); i++)
	{
		yBounds.push_back(y);
		int radius = size / (2 * (thicknesses[thicknesses.size() - 1] + 1 - thicknesses[i]));
		y -= size + radius;
	}
}

// Dessiner l'UI
Mat drawUI(int WS, int HS, Scalar color, int thick)
{
	Mat ui;
	if (doubleScreen)
		ui = Mat(Size(WS, HS), CV_8UC4, black);
	else
		ui = Mat(Size(WS, HS), CV_8UC3, black);

	if (!showUI)
		return ui;

	int size = (int)(WS / scaleUI);
	int x = WS - size;
	int y = HS - size;
	for (int i = 0; i < colors.size(); i++)
	{
		rectangle(ui, Point(x, y), Point(x - size, y - size), colors[i], CV_FILLED);
		x -= size; 
	}
	
	x = (int)(WS - size * 1.5f);
	y = HS - size * 4;
	for (int i = 0; i < thicknesses.size(); i++)
	{
		int radius = size / (2 * (thicknesses[thicknesses.size() - 1] + 1 - thicknesses[i]));
		circle(ui, Point(x, y), radius, color, -1, CV_AA);
		if (thicknesses[i] == thick)
			circle(ui, Point(x, y), radius+2, bound, 2, CV_AA);
		y -= size + radius;
	}

	return ui;
}

// Calibration de l'écran virtuel
bool Calibrate(CalibrationData &cd, Mat camFrame)
{
	Mat green_hue_image;
	cvtColor(camFrame, green_hue_image, CV_BGR2HSV);
	//inRange(green_hue_image, Scalar(30, 100, 100), Scalar(70, 255, 255), green_hue_image);
	inRange(green_hue_image, Scalar(50, 100, 100), Scalar(80, 255, 255), green_hue_image);
	//inRange(green_hue_image, Scalar(30, 100, 100), Scalar(70, 255, 255), green_hue_image);

	vector<Point> centers;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

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
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) > maxcontour)
		{
			maxcontour = contourArea(contours[i]);
			maxcontourIndex = i;
		}
	}
	if (contours.size() > 0 && maxcontour > 10)
	{
		cd.p1 = boundRect[maxcontourIndex].tl();
		cd.p2 = boundRect[maxcontourIndex].br();
		centers.push_back(boundRect[maxcontourIndex].tl());
		centers.push_back(boundRect[maxcontourIndex].br());
		cd.centerDecalage = centers[0].y < centers[1].y ? centers[0] : centers[1];
		cd.W = abs(centers[0].x - centers[1].x);
		cd.H = abs(centers[0].y - centers[1].y);
		drawContours(green_hue_image, contours_poly, maxcontourIndex, white, 1, 8, vector<Vec4i>(), 0);
		rectangle(camFrame, cd.p1, cd.p2, red, 2);
	}
	// Fenêtre pour confirmer la bonne calibration
	imshow("cameraCalib", camFrame);
	imshow("calib", green_hue_image);

	switch (waitKey(10))
	{
	case 99: // 'c' : confirmer calibration ok
		cvDestroyWindow("cameraCalib");
		cvDestroyWindow("calib");
		return true;
	}
	return false;
}

// Fonction pour convertir les coordonées de caméra vers 
// les coordonées locales de l'écran virtuel
// centerDecalage: coordonnées du sommet en haut à gauche de l'écran dans l'espace du caméra
// W & S : dimensions de l'écran perçues par la caméra (normalement plus petites que W & S)
// WS & HS : résolution réelle de l'écran 
Point convertCoord(Point p, CalibrationData calibr)
{
	Point temp = p - calibr.centerDecalage;
	int x = cvRound(temp.x * calibr.WS * 1.0 / (double)calibr.W);
	x = x > 0 ? x : 0;
	int y = cvRound(temp.y * calibr.HS * 1.0 / (double)calibr.H);
	y = y > 0 ? y : 0;
	return Point(x, y);
}

//This function returns the square of the euclidean distance between 2 points.
double dist(Point x, Point y)
{
	return (x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y);
}
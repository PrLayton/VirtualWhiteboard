#include "stdafx.h"

#include <Windows.h>
#include "VirtualWhiteboard.h"
#include "Interface.h"
#include "ScreenCapture.h"

using namespace std;
using namespace cv;

void testUI()
{
	int WS = 800, HS = 600;
	vector<int> xBounds, yBounds;
	Mat ui = drawUI(WS, HS, white, 2, xBounds, yBounds);
	
	while (1)
	{
		Mat screen(Size(WS, HS), CV_8UC3);
		rectangle(screen, Point(0, 0), Point(WS, HS), black, CV_FILLED, 8, 0);
		add(screen, ui, screen);
		imshow("testUI", screen);
		switch (waitKey(10))
		{
		case 27: //'esc' key has been pressed, exit program.
			return;
		case 116: //'t' has been pressed. this will toggle tracking
			ui = drawUI(WS, HS, red, 0, xBounds, yBounds);
		}
	}
}


//This function returns the square of the euclidean distance between 2 points.
double dist(Point x, Point y)
{
	return (x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y);
}


//This function returns the radius and the center of the circle given 3 points
//If a circle cannot be formed , it returns a zero radius circle centered at (0,0)
pair<Point, double> circleFromPoints(Point p1, Point p2, Point p3)
{
	double offset = pow(p2.x, 2) + pow(p2.y, 2);
	double bc = (pow(p1.x, 2) + pow(p1.y, 2) - offset) / 2.0;
	double cd = (offset - pow(p3.x, 2) - pow(p3.y, 2)) / 2.0;
	double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y);
	double TOL = 0.0000001;
	if (abs(det) < TOL) { cout << "POINTS TOO CLOSE" << endl; return make_pair(Point(0, 0), 0); }

	double idet = 1 / det;
	double centerx = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
	double centery = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
	double radius = sqrt(pow(p2.x - centerx, 2) + pow(p2.y - centery, 2));

	return make_pair(Point(centerx, centery), radius);
}

vector<Scalar> colors = { red, green, blue, white };
vector<int> thicknesses = { 5, 4, 3, 2, 1 };
double scaleUI = 20.0f;

Mat hwnd2mat(HWND hwnd);

// Projet de d�tection d'un marqueur avec UI
int DetectionUI()
{
	VideoCapture cap(SOURCE); //capture the video from webcam
	ScreenCapture window(0);
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the webcam" << endl;
		return -1;
	}

	// Taille camera
	int WC = 1920, HC = 1080;
	// Tend au maximum
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WC);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HC);

	Mat  thresholdedFinal;

	// Couleur de dessin
	Scalar color = red;
	int thickness = 3;

	// Ecran de dessin
	Mat drawing(Size(WC, HC), CV_8UC4);
	//rectangle(screen, rec, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
	// Coloration pour la detection
	rectangle(drawing, Point(0,0), Point(WC,HC), black, CV_FILLED);

	vector<int> xBounds, yBounds;
	Mat ui = drawUI(WC, HC, color, thickness, xBounds, yBounds);
	int size = (int)(WC / scaleUI);
	Point lastPoint(-1, -1);
	int confirm_choice_frames = 10;

	Mat frame;
	Mat fore;
	vector<pair<Point, double> > palm_centers;

	namedWindow("Frame");
	namedWindow("Background");
	int backgroundFrame = 500;

	Mat  hsv_frame;
	cv::Vec3b pixel = cv::Vec3b(30, 120, 120);

	Mat frame1, frame2;
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1, grayImage2;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImage;

	Point pt;

	cap.read(frame1);

	while (1)
	{
		pt = Point(0, 0);

		//Mat frame = hwnd2mat(GetDesktopWindow());
		Mat screen;
		window >> screen;
		
		cap.read(frame);
		if (frame.empty())
		{
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}
		// Flipper l'image
		//flip(frame, frame, 1);

		vector<vector<Point> > contours;

		// Detection de d�placement

		//convert frame1 to gray scale for frame differencing
		cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
		//copy second frame
		cap.read(frame2);
		//convert frame2 to gray scale for frame differencing
		cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);
		//perform frame differencing with the sequential images. This will output an "intensity image"
		//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
		cv::absdiff(grayImage1, grayImage2, differenceImage);
		//threshold intensity image at a given sensitivity value
		cv::threshold(differenceImage, thresholdImage, 20, 255, THRESH_BINARY);

		Mat copy;
		//bitwise_not(thresholdImage, thresholdImage);
		frame.copyTo(copy, thresholdImage);
		//hsv_frame = dst2;

		// Pour le grain
		//medianBlur(frame, frame, 3);
		// HSV plus facile � traiter
		cvtColor(copy, hsv_frame, CV_BGR2HSV);
		//inRange(hsv_frame, Scalar(20, 100, 100), Scalar(60, 255, 255), fore);
		//inRange(hsv_frame, Scalar(112, 0, 100), Scalar(152, 100, 255), fore);

		inRange(hsv_frame, Scalar(max(0, pixel[0] - 20), max(0, pixel[1] - 60), max(0, pixel[2] - 60)), Scalar(min(255, pixel[0] + 20), min(255, pixel[1] + 60), min(255, pixel[2] + 60)), fore);


		//Enhance edges in the foreground by applying erosion and dilation
		erode(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		GaussianBlur(fore, fore, Size(5, 5), 0, 0);


		//Find the contours in the foreground
		findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		int maxcontour = 0, maxcontourIndex = 0;
		for (int i = 0; i < contours.size(); i++) {

			if (contourArea(contours[i]) > maxcontour) {
				maxcontour = contourArea(contours[i]);
				maxcontourIndex = i;
			}
		}

		for (int i = 0; i<contours.size(); i++)
			//Ignore all small insignificant areas
			//if (contourArea(contours[i]) >= 500)
			if (i == maxcontourIndex)
			{
				//Draw contour
				vector<vector<Point> > tcontours;
				tcontours.push_back(contours[i]);
				drawContours(frame, tcontours, -1, cv::Scalar(0, 0, 255), 2);

				//Detect Hull in current contour
				vector<vector<Point> > hulls(1);
				vector<vector<int> > hullsI(1);
				convexHull(Mat(tcontours[0]), hulls[0], false);
				convexHull(Mat(tcontours[0]), hullsI[0], false);
				drawContours(frame, hulls, -1, cv::Scalar(0, 255, 0), 2);

				//Find minimum area rectangle to enclose hand
				RotatedRect rect = minAreaRect(Mat(tcontours[0]));

				//Find Convex Defects
				vector<Vec4i> defects;
				if (hullsI[0].size()>0)
				{
					Point2f rect_points[4]; rect.points(rect_points);

					Point2f p;
					for (int j = 0; j < 4; j++)
						p += rect_points[j];
					p /= 4;
					circle(frame, p, 5, Scalar(255, 255, 255), 5);
					pt = Point(cvRound(p.x), cvRound(p.y));

					for (int j = 0; j < 4; j++)
						line(frame, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
					Point rough_palm_center;
					convexityDefects(tcontours[0], hullsI[0], defects);
					if (defects.size() >= 3)
					{
						vector<Point> palm_points;
						for (int j = 0; j<defects.size(); j++)
						{
							int startidx = defects[j][0]; Point ptStart(tcontours[0][startidx]);
							int endidx = defects[j][1]; Point ptEnd(tcontours[0][endidx]);
							int faridx = defects[j][2]; Point ptFar(tcontours[0][faridx]);
							//Sum up all the hull and defect points to compute average
							rough_palm_center += ptFar + ptStart + ptEnd;
							palm_points.push_back(ptFar);
							palm_points.push_back(ptStart);
							palm_points.push_back(ptEnd);
						}

						//Get palm center by 1st getting the average of all defect points, this is the rough palm center,
						//Then U chose the closest 3 points ang get the circle radius and center formed from them which is the palm center.
						rough_palm_center.x /= defects.size() * 3;
						rough_palm_center.y /= defects.size() * 3;
						Point closest_pt = palm_points[0];
						vector<pair<double, int> > distvec;
						for (int i = 0; i<palm_points.size(); i++)
							distvec.push_back(make_pair(dist(rough_palm_center, palm_points[i]), i));
						sort(distvec.begin(), distvec.end());

						//Keep choosing 3 points till you find a circle with a valid radius
						//As there is a high chance that the closes points might be in a linear line or too close that it forms a very large circle
						pair<Point, double> soln_circle;
						for (int i = 0; i + 2<distvec.size(); i++)
						{
							Point p1 = palm_points[distvec[i + 0].second];
							Point p2 = palm_points[distvec[i + 1].second];
							Point p3 = palm_points[distvec[i + 2].second];
							soln_circle = circleFromPoints(p1, p2, p3);//Final palm center,radius
							if (soln_circle.second != 0)
								break;
						}

						//Find avg palm centers for the last few frames to stabilize its centers, also find the avg radius
						palm_centers.push_back(soln_circle);
						if (palm_centers.size()>10)
							palm_centers.erase(palm_centers.begin());

						Point palm_center;
						double radius = 0;
						for (int i = 0; i<palm_centers.size(); i++)
						{
							palm_center += palm_centers[i].first;
							radius += palm_centers[i].second;
						}
						palm_center.x /= palm_centers.size();
						palm_center.y /= palm_centers.size();
						radius /= palm_centers.size();

						//Draw the palm center and the palm circle
						//The size of the palm gives the depth of the hand
						//circle(frame, palm_center, 5, Scalar(144, 144, 255), 3);
						//circle(frame, palm_center, radius, Scalar(144, 144, 255), 2);

						//Detect fingers by finding points that form an almost isosceles triangle with certain thesholds
						int no_of_fingers = 0;
						for (int j = 0; j<defects.size(); j++)
						{
							int startidx = defects[j][0]; Point ptStart(tcontours[0][startidx]);
							int endidx = defects[j][1]; Point ptEnd(tcontours[0][endidx]);
							int faridx = defects[j][2]; Point ptFar(tcontours[0][faridx]);
							//X o--------------------------o Y
							double Xdist = sqrt(dist(palm_center, ptFar));
							double Ydist = sqrt(dist(palm_center, ptStart));
							double length = sqrt(dist(ptFar, ptStart));

							double retLength = sqrt(dist(ptEnd, ptFar));
							//Play with these thresholds to improve performance
							if (length <= 3 * radius&&Ydist >= 0.4*radius&&length >= 10 && retLength >= 10 && max(length, retLength) / min(length, retLength) >= 0.8)
								if (min(Xdist, Ydist) / max(Xdist, Ydist) <= 0.8)
								{
									if ((Xdist >= 0.1*radius&&Xdist <= 1.3*radius&&Xdist<Ydist) || (Ydist >= 0.1*radius&&Ydist <= 1.3*radius&&Xdist>Ydist))
										line(frame, ptEnd, ptFar, Scalar(0, 255, 0), 1), no_of_fingers++;
								}


						}

						no_of_fingers = min(5, no_of_fingers);
						//cout << "NO OF FINGERS: " << no_of_fingers << endl;
					}
				}

			}
		
		//imshow("Frame", frame);
		imshow("Background", fore);
		imshow("thresholdImage", thresholdImage);

		/*
		// TRACKER VERT - STICKER
		inRange(hsv_frame, Scalar(40, 90, 150), Scalar(90, 255, 255), thresholdedFinal);
		
		// Am�lioration de la qualit� du r�sultat
		//erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//dilate(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//erode(thresholdedFinal, thresholdedFinal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		GaussianBlur(thresholdedFinal, thresholdedFinal, Size(9, 9), 2, 2);

		// Stockage des points de reconnaissance
		vector<Vec3f> storage;
		HoughCircles(thresholdedFinal, storage, CV_HOUGH_GRADIENT, 2, thresholdedFinal.rows / 4, 100, 60, 10, 400);


		// traitement des points d�tect�s
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
			Point pt = Point(cvRound(p[0]), cvRound(p[1]));
			*/

			//cout << pt.x << "  " << pt.y << endl;

		if (pt.x > 0 && pt.y > 0)
		{
			// D�tection de l'UI des couleurs
			if (pt.y >= yBounds[0] && pt.y <= yBounds[1])
			{
				if (pt.x <= xBounds[0] && pt.x >= xBounds[4])
				{
					color = ui.at<Vec3b>(pt.y, pt.x);
					ui = drawUI(WC, HC, color, thickness, xBounds, yBounds);
				}
				/*else if (pt.x >= xBounds[4])
				{
				sprintf_s(buffer, "Changing color in %d", confirm_choice_frames);
				cvPutText(frame, buffer, cvPoint(150, 70), &font, red);
				confirm_choice_frames--;
				if (confirm_choice_frames < 0) // confirm in 10 frames before clearing
				{
				confirm_choice_frames = 10;
				color = colors[3];
				cvPutText(frame, "Changed color.", cvPoint(150, 110), &font, white);
				}
				}*/
			}
			// D�tection de l'UI des thickness
			else if (pt.x <= xBounds[0] && pt.x >= xBounds[1])
			{
				for (int i = 0; i < thicknesses.size(); i++)
				{
					int radius = size / (2 * (6 - thicknesses[i]));
					if (pt.y >= yBounds[i + 2] - radius && pt.y <= yBounds[i + 2] + radius)
					{
						thickness = thicknesses[i];
						ui = drawUI(WC, HC, color, thickness, xBounds, yBounds);
						break;
					}
				}				
			}
			// Dessiner le segment
			if (lastPoint.x != -1 && pt.x <= xBounds[4])
				line(drawing, lastPoint, pt, color, thickness, CV_AA);
			lastPoint = pt;
			//circle(frame, pt, cvRound(p[2]), red);
		}
		else
		{
			lastPoint = Point(-1, -1);
		}

		add(screen, drawing, screen);
		add(screen, ui, screen);
		imshow("GREEN FILTRE", thresholdedFinal);
		//cvNamedWindow("ECRAN VIRTUELLE", CV_WINDOW_NORMAL);
		//cvSetWindowProperty("ECRAN VIRTUELLE", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		//imshow("Camera", frame);
		imshow("ECRAN VIRTUELLE", screen);

		switch (waitKey(10))
		{
		case 27: //'esc' key has been pressed, exit program.
			return 0;
		case 100: //d
			cap.read(frame1);
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
			pixel = cv::Vec3b(h / 16, s / 16, v / 16);
			cout << h / 16 << " " << s / 16 << " " << v / 16 << endl;
			break;
		}
	}

	cvDestroyAllWindows();
	cap.release();

	return 0;
}

Mat drawUI(int WS, int HS, Scalar color, int thick, vector<int> &xBounds, vector<int> &yBounds)
{
	Mat ui(Size(WS, HS), CV_8UC4);
	rectangle(ui, Point(0, 0), Point(WS, HS), black, CV_FILLED, 8, 0);
	int size = (int)(WS / scaleUI);
	int x = WS - size;
	int y = size;
	for (int i = 0; i < colors.size(); i++)
	{
		rectangle(ui, Point(x, y), Point(x - size, y + size), colors[i], CV_FILLED, 8, 0);
		xBounds.push_back(x);
		x -= size; 
	}
	xBounds.push_back(x);
	
	x = (int)(WS - size * 1.5f);
	yBounds.push_back(y);
	yBounds.push_back(y + size);
	y = size * 4;
	for (int i = 0; i < thicknesses.size(); i++)
	{
		yBounds.push_back(y);
		int radius = size / (2 * (6-thicknesses[i]));
		circle(ui, Point(x, y), radius, color, -1, CV_AA);
		if (thicknesses[i] == thick)
			circle(ui, Point(x, y), radius+2, color.conj(), 2, CV_AA);
		y += size + radius;
	}

	return ui;
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
img � The image
pt1 � First point of the line segment
pt2 � Second point of the line segment
Type of the line:
8 - (or omitted) 8-connected line.
4 - 4-connected line.
CV_AA - antialiased line.
shift � Number of fractional bits in the point coordinates
#################################################################################################################
void cvInitFont(CvFont* font, int fontFace, double hscale, double vscale, double shear=0, int thickness=1, int lineType=8)
Initializes font structure.Parameters:	font � Pointer to the font structure initialized by the function
fontFace �

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
void cvSmooth(const CvArr* src, CvArr* dst, int smoothtype=CV_GAUSSIAN, int param1=3, int param2=0, double param3=0, double param4=0)�
http://opencv.willowgarage.com/documentation/c/image_filtering.html#cvSmooth
#################################################################################################################
void cvAnd( const CvArr* A, const CvArr* B, CvArr* C, const CvArr* mask=0 );
http://www710.univ-lyon1.fr/~bouakaz/OpenCV-0.9.5/docs/ref/OpenCVRef_BasicFuncs.htm
#################################################################################################################
*/
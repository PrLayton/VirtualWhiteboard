#include "stdafx.h"
#include "VirtualWhiteboard.h"
#include "Interface.h"

//#include "opencv2\tracking.hpp"

int Hand();

int DetectionUI();

int main(int argc, char* argv[])
{
	DetectionUI();
	//Finger();
	//Calibration();
	//DrawScreen();
	//Detection();
	//cvPaint();
	//DetectMovement();
	Hand();
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

//The main function :D
int Hand()
{
	Mat frame;
	Mat fore;
	vector<pair<Point, double> > palm_centers;
	VideoCapture cap(0);

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

	cap.read(frame1);

	while(1)
	{
		vector<vector<Point> > contours;
		//Get the frame
		cap >> frame;

		// Detection de déplacement

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

		//Update the current background model and get the foreground
		cvtColor(copy, hsv_frame, CV_BGR2HSV);
		//inRange(hsv_frame, Scalar(20, 100, 100), Scalar(60, 255, 255), fore);
		//inRange(hsv_frame, Scalar(112, 0, 100), Scalar(152, 100, 255), fore);

		inRange(hsv_frame, Scalar(max(0, pixel[0] - 20), max(0, pixel[1] - 60), max(0, pixel[2] - 60)), Scalar(min(255, pixel[0] + 20), min(255, pixel[1] + 60), min(255, pixel[2] + 60)), fore);


		//Enhance edges in the foreground by applying erosion and dilation
		erode(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(fore, fore, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		GaussianBlur(fore, fore, Size(5,5), 0, 0);


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
			if(i == maxcontourIndex)
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
					circle(frame, p / 4, 5, Scalar(255, 255, 255), 5);
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
		if (backgroundFrame>0)
			putText(frame, "Recording Background", cvPoint(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
		imshow("Frame", frame);
		imshow("Background", fore);
		imshow("thresholdImage", thresholdImage);
		

		switch (waitKey(10)) {
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
	return 0;
}
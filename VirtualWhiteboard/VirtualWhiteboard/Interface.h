#pragma once

#define blue  CV_RGB(0,0,255)
#define green CV_RGB(0,255,0)
#define red   CV_RGB(255,0,0)
#define white CV_RGB(255,255,255)
#define black CV_RGB(0,0,0)
#define bound CV_RGB(127,127,127)

struct CalibrationData
{
	int WS = 1600, HS = 900;
	int W = 1, H = 1;
	Point centerDecalage = Point(0, 0);
	Point2i p1 = Point(0, 0), p2 = Point(0, 0);
};

double dist(Point x, Point y);
Point convertCoord(Point p, CalibrationData calibr);
bool Calibrate(CalibrationData &cd, Mat camFrame);
void createBoundsUI(int WS, int HS, vector<int> &xBounds, vector<int> &yBounds);
Mat drawUI(int WS, int HS, Scalar color, int thick);
int DetectionUI();
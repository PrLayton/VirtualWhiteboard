#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <cmath>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

using namespace cv;
using namespace std;

// Source de webCam: interne = 0, externe = 1
#define SOURCE 0
#define doubleScreen 1

vector<Point> DetectScreen(Mat image, vector<Point> &lastCenters, double minDist);
Point convertCoord(Point p, Point decalage, int WC, int HC, int WS, int HS);
int Detection();
int DrawScreen();
int Calibration();
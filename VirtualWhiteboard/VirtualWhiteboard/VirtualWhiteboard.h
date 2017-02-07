#pragma once

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

vector<Point> DetectScreen(Mat image, vector<Point> &lastCenters, double minDist);
int Detection();
int DrawScreen();
int Calibration();
int Finger();
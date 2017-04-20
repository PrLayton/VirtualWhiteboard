#pragma once

#define blue  CV_RGB(0,0,255)
#define green CV_RGB(0,255,0)
#define red   CV_RGB(255,0,0)
#define white CV_RGB(255,255,255)
#define black CV_RGB(0,0,0)
#define bound CV_RGB(127,127,127)
#define doubleScreen false

void ClearScreen(IplImage* imgScribble, IplImage* imgDrawing);
IplImage* GetThresholdedImage(IplImage* img, CvScalar& lowerBound, CvScalar& upperBound);
int cvPaint();
Mat drawUI(int WS, int HS, Scalar color, int thick, vector<int> &xBounds, vector<int> &yBounds);
void testUI();
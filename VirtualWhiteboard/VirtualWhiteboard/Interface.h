#pragma once

void ClearScreen(IplImage* imgScribble, IplImage* imgDrawing);
IplImage* GetThresholdedImage(IplImage* img, CvScalar& lowerBound, CvScalar& upperBound);
int cvPaint();
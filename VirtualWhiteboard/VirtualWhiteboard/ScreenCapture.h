#pragma once

#include <opencv2\opencv.hpp>
#include <Windows.h>

class ScreenCapture
{
public:
	ScreenCapture(int displayIndex = -1);
	~ScreenCapture();

	void open(int displayIndex);
	void read(cv::Mat& destination);
	ScreenCapture& operator >> (cv::Mat& destination);

	int width;
	int height;

private:
	cv::Rect2d captureArea;
	HWND targetWindow = NULL;


	void captureHwnd(HWND window, cv::Rect2d targetArea, cv::Mat& dest);
	static BOOL CALLBACK monitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData);
};

struct MonitorIndexLookupInfo
{
	int targetIndex;

	RECT outRect;
	int currentIndex;
};

void GetDesktopResolution(int& horizontal, int& vertical);
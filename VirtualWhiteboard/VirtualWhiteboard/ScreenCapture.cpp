#include "stdafx.h"

#include "ScreenCapture.h"

#include <Windows.h>

using namespace std;
using namespace cv;

// Get the horizontal and vertical screen sizes in pixel
void GetDesktopResolution(int& horizontal, int& vertical)
{
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);
	// The top left corner will have coordinates (0,0)
	// and the bottom right corner will have coordinates
	// (horizontal, vertical)
	horizontal = desktop.right;
	vertical = desktop.bottom;
}

ScreenCapture::ScreenCapture(int displayIndex)
{
	if (displayIndex >= 0)
		open(displayIndex);
}

void ScreenCapture::open(int displayIndex)
{
	MonitorIndexLookupInfo enumState = { displayIndex, NULL, 0 };
	EnumDisplayMonitors(NULL, NULL, monitorEnumProc, (LPARAM)&enumState);

	this->captureArea = cv::Rect2d(enumState.outRect.left, enumState.outRect.top, enumState.outRect.right - enumState.outRect.left, enumState.outRect.bottom - enumState.outRect.top);
	width = captureArea.width;
	height = captureArea.height;
	this->targetWindow = GetDesktopWindow();
}

BOOL CALLBACK ScreenCapture::monitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData)
{
	MonitorIndexLookupInfo* enumState = (MonitorIndexLookupInfo*)dwData;
	if (enumState->targetIndex == enumState->currentIndex)
	{
		enumState->outRect = *lprcMonitor;
		return false;
	}

	enumState->currentIndex++;

}

ScreenCapture::~ScreenCapture()
{
}

void ScreenCapture::read(cv::Mat& destination)
{
	if (targetWindow == NULL)
		throw new std::exception("No target monitor specified! The 'open()' method must be called to select a target monitor before frames can be read.");

	captureHwnd(targetWindow, captureArea, destination);
}

ScreenCapture& ScreenCapture::operator >> (cv::Mat& destination)
{
	read(destination);
	return *this;
}

void ScreenCapture::captureHwnd(HWND window, cv::Rect2d targetArea, cv::Mat& dest)
{
	HDC hwindowDC, hwindowCompatibleDC;

	HBITMAP hbwindow;
	BITMAPINFOHEADER  bi;

	hwindowDC = GetDC(window);
	hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
	SetStretchBltMode(hwindowCompatibleDC, COLORONCOLOR);

	dest.create(targetArea.height, targetArea.width, CV_8UC4);

	// Initialize a bitmap
	hbwindow = CreateCompatibleBitmap(hwindowDC, targetArea.width, targetArea.height);
	bi.biSize = sizeof(BITMAPINFOHEADER);
	bi.biWidth = targetArea.width;
	// The negative height is required -- removing the inversion will make the image appear upside-down.
	bi.biHeight = -targetArea.height;
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	SelectObject(hwindowCompatibleDC, hbwindow);
	// Copy from the window device context to the bitmap device context
	// Use BitBlt to do a copy without any stretching -- the output is of the same dimensions as the target area.
	BitBlt(hwindowCompatibleDC, 0, 0, targetArea.width, targetArea.height, hwindowDC, targetArea.x, targetArea.y, SRCCOPY);
	// Copy into our own buffer as device-independent bitmap
	GetDIBits(hwindowCompatibleDC, hbwindow, 0, targetArea.height, dest.data, (BITMAPINFO *)&bi, DIB_RGB_COLORS);

	// Clean up memory to avoid leaks
	DeleteObject(hbwindow);
	DeleteDC(hwindowCompatibleDC);
	ReleaseDC(window, hwindowDC);
}

Mat hwnd2mat(HWND hwnd) 
{

	HDC hwindowDC, hwindowCompatibleDC;

	int height, width, srcheight, srcwidth;
	HBITMAP hbwindow;
	Mat src;
	BITMAPINFOHEADER bi;

	hwindowDC = GetDC(hwnd);
	hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
	SetStretchBltMode(hwindowCompatibleDC, COLORONCOLOR);

	RECT windowsize;    // get the height and width of the screen
	GetClientRect(hwnd, &windowsize);

	srcheight = windowsize.bottom;
	srcwidth = windowsize.right;
	height = windowsize.bottom / 2;  //change this to whatever size you want to resize to
	width = windowsize.right / 2;

	src.create(height, width, CV_8UC4);

	// create a bitmap
	hbwindow = CreateCompatibleBitmap(hwindowDC, width, height);
	bi.biSize = sizeof(BITMAPINFOHEADER);    //http://msdn.microsoft.com/en-us/library/windows/window/dd183402%28v=vs.85%29.aspx
	bi.biWidth = width;
	bi.biHeight = -height;  //this is the line that makes it draw upside down or not
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	// use the previously created device context with the bitmap
	SelectObject(hwindowCompatibleDC, hbwindow);
	// copy from the window device context to the bitmap device context
	StretchBlt(hwindowCompatibleDC, 0, 0, width, height, hwindowDC, 0, 0, srcwidth, srcheight, SRCCOPY); //change SRCCOPY to NOTSRCCOPY for wacky colors !
	GetDIBits(hwindowCompatibleDC, hbwindow, 0, height, src.data, (BITMAPINFO *)&bi, DIB_RGB_COLORS);  //copy from hwindowCompatibleDC to hbwindow

																									   // avoid memory leak
	DeleteObject(hbwindow); DeleteDC(hwindowCompatibleDC); ReleaseDC(hwnd, hwindowDC);

	return src;
}
/*******************************************************************************
文件名称：imageProcess.h 
功    能：算法实现
时    间：9/25/2015 XUE
版    本：1.0
注    意：
*******************************************************************************/
#ifndef imageProcess_h__
#define imageProcess_h__

#define _OPENCV_100_BUILD
//#define _OPENCV_249_BUILD

#ifdef _OPENCV_100_BUILD
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#ifdef _DEBUG
#pragma comment(lib,"cvd.lib")
#pragma comment(lib,"cvauxd.lib")
#pragma comment(lib,"cvcam.lib")
#pragma comment(lib,"cxcored.lib")
#pragma comment(lib,"highguid.lib")
#endif
#ifndef _DEBUG
#pragma comment(lib,"cv.lib")
#pragma comment(lib,"cvaux.lib")
#pragma comment(lib,"cvcam.lib")
#pragma comment(lib,"cxcore.lib")
#pragma comment(lib,"highgui.lib")
#endif
#endif //OPENCV_100_BUILD

#ifdef _OPENCV_249_BUILD
#include <vector>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

#ifdef _DEBUG
#pragma comment(lib,"opencv_contrib249d.lib")
#pragma comment(lib,"opencv_core249d.lib")
#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_imgproc249d.lib")
#pragma comment(lib,"opencv_legacy249d.lib")
#pragma comment(lib,"opencv_calib3d249d.lib")
#pragma comment(lib,"opencv_features2d249d.lib")
#pragma comment(lib,"opencv_flann249d.lib")
#pragma comment(lib, "opencv_objdetect249d.lib")
#pragma comment(lib, "opencv_video249d.lib")
#endif
#ifndef _DEBUG
#pragma comment(lib,"opencv_contrib249.lib")
#pragma comment(lib,"opencv_core249.lib")
#pragma comment(lib,"opencv_highgui249.lib")
#pragma comment(lib,"opencv_imgproc249.lib")
#pragma comment(lib,"opencv_legacy249.lib")
#pragma comment(lib,"opencv_calib3d249.lib")
#pragma comment(lib,"opencv_features2d249.lib")
#pragma comment(lib,"opencv_flann249.lib")
#pragma comment(lib, "opencv_objdetect249.lib")
#pragma comment(lib, "opencv_video249.lib")
#endif
#endif //OPENCV_249_BUILD

#include "itcCore.h"
#include "stuTrack_track_img.h"

#endif // imageProcess_h__
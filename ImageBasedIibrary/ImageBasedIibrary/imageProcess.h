/*******************************************************************************
文件名称：imageProcess.h 
功    能：算法实现
时    间：9/25/2015 XUE
版    本：1.0
注    意：
*******************************************************************************/
#ifndef imageProcess_h__
#define imageProcess_h__

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

#include "itcCore.h"

#endif // imageProcess_h__
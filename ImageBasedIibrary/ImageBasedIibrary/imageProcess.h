/*******************************************************************************
�ļ����ƣ�imageProcess.h 
��    �ܣ��㷨ʵ��
ʱ    �䣺9/25/2015 XUE
��    ����1.0
ע    �⣺
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
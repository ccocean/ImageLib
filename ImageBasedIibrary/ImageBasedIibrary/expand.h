/*******************************************************************************
�ļ����ƣ�expand.h 
��    �ܣ���չ��
ʱ    �䣺9/25/2015 XUE
��    ����1.0
ע    �⣺
*******************************************************************************/
#ifndef expand_h__
#define expand_h__
#include "imageProcess.h"

CvMat MatToCV(Itc_Mat_t src)
{
	CvMat m;

	m.type = src.type;
	m.cols = src.cols;
	m.rows = src.rows;
	m.step = src.step;
	m.data.ptr = src.data.ptr;
	m.refcount = src.refcount;
	m.hdr_refcount = src.hdr_refcount;
	return m;
};

Itc_Mat_t MatToITC(CvMat src)
{
	Itc_Mat_t m;

	m.type = src.type;
	m.cols = src.cols;
	m.rows = src.rows;
	m.step = src.step;
	m.data.ptr = src.data.ptr;
	m.refcount = src.refcount;
	m.hdr_refcount = src.hdr_refcount;
	return m;
};

#endif // expand_h__
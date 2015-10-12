/************************************************************************** 
    *  @Copyright (c) 2015, XueYB, All rights reserved. 
 
    *  @file     : itcCore.h 
    *  @version  : ver 1.0 
 
    *  @author   : XueYB 
    *  @date     : 2015/10/09 11:19 
    *  @brief    : Mat�ṹ��ͼ���������
**************************************************************************/
#ifndef itcCore_h__
#define itcCore_h__
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>

#include "itctype.h"
#include "itcerror.h"
#include "itcdatastructs.h"

typedef struct ItcMat
{
	int type;
	int step;

	/* for internal use only */
	int* refcount;
	int hdr_refcount;

	union
	{
		uchar* ptr;
		short* s;
		int* i;
		float* fl;
		double* db;
	} data;

	union
	{
		int rows;
		int height;
	};

	union
	{
		int cols;
		int width;
	};
}
ItcMat;

ItcMat	itcMat( int rows, int cols, int type, void* data);									//�ֶ��������ݴ���Mat,ע�ⲻ����itcReleaseMat�ͷ�
ItcMat*	itcCreateMat( int height, int width, int type );									//����Mat
ItcMat*	itcCreateMatHeader( int rows, int cols, int type );									//����Matͷ
ItcMat*	itcInitMatHeader( ItcMat* arr, int rows, int cols, int type, void* data, int step );//��Matͷ���ʼ��
void	itcReleaseMat( ItcMat** mat );														//�ͷ�Mat,����ͷ������


/*------------------------------------�����������-------------------------------------------*/

// typedef struct ItcFuncTable	//����ָ�룬������ȵĲ�ָͬ��ͬ����δʹ��
// {
// 	void*   fn_2d[ITC_DEPTH_MAX];
// }
// ItcFuncTable;

#define ICV_DEF_BIN_ARI_OP_2D( __op__, name, type, worktype, cast_macro )   \
static void  name													        \
    ( uchar* src1, int step1, uchar* src2,int step2,						\
	  uchar* dst, int dstep, ItcSize size)									\
{                                                                           \
	int i=0;																\
	int j=0;																\
	type *srct1;															\
	type *srct2;															\
	type *dstt;																\
	for(i=0; i < size.height; i++)                                          \
	{																		\
		srct1=(type*)src1;													\
		srct2=(type*)src2;													\
		dstt=(type*)dst;													\
		for(j=0; j < size.width; j++)										\
		{                                                                   \
			worktype t0 = __op__((srct1)[j],(srct2)[j]);					\
			(dstt)[j] = cast_macro( t0 );									\
		}																	\
		src1 += step1;														\
		src2 += step2;														\
		dst += dstep;														\
	}		                                                        		\
}																			                                                          

//__op__�ǲ������ͣ�
#define ICV_DEF_BIN_ARI_ALL( __op__, name )										 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_8u_C1R, uchar, int, ITC_CAST_8U )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_8s_C1R, char, int, ITC_CAST_8S )		 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_16u_C1R, ushort, int, ITC_CAST_16U )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_16s_C1R, short, int, ITC_CAST_16S )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_32s_C1R, int, int, ITC_CAST_32S )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_32f_C1R, float, float, ITC_CAST_32F ) \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_64f_C1R, double, double, ITC_CAST_64F )

#undef ITC_SUB_R
#define ITC_SUB_R(a,b) ((a) - (b))							//����sub����

ICV_DEF_BIN_ARI_ALL( ITC_SUB_R, Sub )						//����sub�����ĺ���

void itcSub(ItcMat* src1,ItcMat* src2,ItcMat* dst);			//dst=src1-src2

//����������֡ͼ������mhiͼ
void itcUpdateMHI(ItcMat* src1,//��ǰ֡
	ItcMat* src2,		//��һ֡
	ItcMat* mhi,		//�˶���ʷͼ��
	int diffThreshold,	//���ڹ���֡���С
	ItcMat* maskT,		//����ͼ��������������
	int Threshold);		//������������,mhi���ڸ�ֵ�ŰѶ�Ӧ��maskT��Ϊ1

//�������
int itcFindContours(ItcMat* src1, ItcContour** pContour, ItcMemStorage*  storage);
#endif // itcCore_h__
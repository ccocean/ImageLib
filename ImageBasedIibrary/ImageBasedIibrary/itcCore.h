/************************************************************************** 
    *  @Copyright (c) 2015, XueYB, All rights reserved. 
 
    *  @file     : itcCore.h 
    *  @version  : ver 1.0 
 
    *  @author   : XueYB 
    *  @date     : 2015/10/09 11:19 
    *  @brief    : Mat结构和图像基本操作
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

ItcMat	itcMat( int rows, int cols, int type, void* data);									//手动分配数据创建Mat,注意不能用itcReleaseMat释放
ItcMat*	itcCreateMat( int height, int width, int type );									//创建Mat
ItcMat*	itcCreateMatHeader( int rows, int cols, int type );									//创建Mat头
ItcMat*	itcInitMatHeader( ItcMat* arr, int rows, int cols, int type, void* data, int step );//有Mat头后初始化
void	itcReleaseMat( ItcMat** mat );														//释放Mat,包括头和数据


/*------------------------------------矩阵基本运算-------------------------------------------*/

// typedef struct ItcFuncTable	//函数指针，根据深度的不同指向不同，暂未使用
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

//__op__是操作类型，
#define ICV_DEF_BIN_ARI_ALL( __op__, name )										 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_8u_C1R, uchar, int, ITC_CAST_8U )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_8s_C1R, char, int, ITC_CAST_8S )		 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_16u_C1R, ushort, int, ITC_CAST_16U )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_16s_C1R, short, int, ITC_CAST_16S )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_32s_C1R, int, int, ITC_CAST_32S )	 \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_32f_C1R, float, float, ITC_CAST_32F ) \
ICV_DEF_BIN_ARI_OP_2D( __op__, icv##name##_64f_C1R, double, double, ITC_CAST_64F )

#undef ITC_SUB_R
#define ITC_SUB_R(a,b) ((a) - (b))							//定义sub操作

ICV_DEF_BIN_ARI_ALL( ITC_SUB_R, Sub )						//定义sub操作的函数

void itcSub(ItcMat* src1,ItcMat* src2,ItcMat* dst);			//dst=src1-src2

//输入连续两帧图像，生成mhi图
void itcUpdateMHI(ItcMat* src1,//当前帧
	ItcMat* src2,		//上一帧
	ItcMat* mhi,		//运动历史图像
	int diffThreshold,	//用于过滤帧差大小
	ItcMat* maskT,		//掩码图像，用于轮廓分析
	int Threshold);		//用于生成掩码,mhi大于该值才把对应的maskT置为1

//轮廓检测
int itcFindContours(ItcMat* src1, ItcContour** pContour, ItcMemStorage*  storage);
#endif // itcCore_h__
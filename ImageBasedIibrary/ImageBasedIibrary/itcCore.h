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
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>

#include "itctype.h"
#include "itcerror.h"
#include "itcdatastructs.h"

typedef struct Teack_Stand_t
{
	ItcPoint centre;
	ItcRect roi;
	int count_teack;	//
	int count_up;		//
	int count_down;		//
	int flag_Stand;		//
	int flag_matching;	//匹配标志
}Teack_Stand_t;

typedef struct Itc_Mat
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
Itc_Mat_t;

Itc_Mat_t	itc_mat(int rows, int cols, int type, void* data);											//手动分配数据创建Mat,注意不能用itcReleaseMat释放
Itc_Mat_t*	itc_create_mat( int height, int width, int type );											//创建Mat
Itc_Mat_t*	itc_create_matHeader( int rows, int cols, int type );										//创建Mat头
Itc_Mat_t*	itc_init_matHeader( Itc_Mat_t* arr, int rows, int cols, int type, void* data, int step );	//有Mat头后初始化
void	itc_release_mat( Itc_Mat_t** mat );																//释放Mat,包括头和数据


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

void track_sub_mat(Itc_Mat_t* src1, Itc_Mat_t* src2, Itc_Mat_t* dst);			//dst=src1-src2

//输入连续两帧图像，生成mhi图
void track_update_MHI(Itc_Mat_t* src1,//当前帧
	Itc_Mat_t* src2,				//上一帧
	Itc_Mat_t* mhi,					//运动历史图像
	int diffThreshold,				//用于过滤帧差大小
	Itc_Mat_t* maskT,				//掩码图像，用于轮廓分析
	int Threshold);					//用于生成掩码,mhi大于该值才把对应的maskT置为1

//轮廓检测
int track_find_contours(Itc_Mat_t* src1,	//输入二值图像（0，1），4周边界必须为0，否则会越界
	ItcContour** pContour,				//输出的轮廓
	ItcMemStorage*  storage);			//存储器

//轮廓筛选
int stuTrack_filtrate_contours(ItcContour** pContour, int* size_Threshold, ItcRect *rect_arr);
int track_filtrate_contours(ItcContour** pContour,int size_Threshold, ItcRect *rect_arr);

bool track_intersect_rect(ItcRect *rectA, ItcRect *rectB);					//判断两个矩形框是否相交

int track_calculateDirect_ROI(Itc_Mat_t* src, ItcRect roi, int &Direct);	//返回的方向Direct取1,2,3,4分别代表上，下，左，右

void track_update_midValueBK(Itc_Mat_t* mat, Itc_Mat_t* matBK);				//用中值法更新背景

void stuTrack_matching_ROI(ItcRect roi, Teack_Stand_t teack_stand[], int &count_trackObj);
void stuTrack_analyze_ROI(Itc_Mat_t* mhi,Teack_Stand_t teack_stand[], int &count_trackObj);
bool stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, Teack_Stand_t teack_stand);
#endif // itcCore_h__
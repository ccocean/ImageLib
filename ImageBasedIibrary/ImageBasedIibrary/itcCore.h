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
	int flag_matching;	//ƥ���־
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

Itc_Mat_t	itc_mat(int rows, int cols, int type, void* data);											//�ֶ��������ݴ���Mat,ע�ⲻ����itcReleaseMat�ͷ�
Itc_Mat_t*	itc_create_mat( int height, int width, int type );											//����Mat
Itc_Mat_t*	itc_create_matHeader( int rows, int cols, int type );										//����Matͷ
Itc_Mat_t*	itc_init_matHeader( Itc_Mat_t* arr, int rows, int cols, int type, void* data, int step );	//��Matͷ���ʼ��
void	itc_release_mat( Itc_Mat_t** mat );																//�ͷ�Mat,����ͷ������


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

void track_sub_mat(Itc_Mat_t* src1, Itc_Mat_t* src2, Itc_Mat_t* dst);			//dst=src1-src2

//����������֡ͼ������mhiͼ
void track_update_MHI(Itc_Mat_t* src1,//��ǰ֡
	Itc_Mat_t* src2,				//��һ֡
	Itc_Mat_t* mhi,					//�˶���ʷͼ��
	int diffThreshold,				//���ڹ���֡���С
	Itc_Mat_t* maskT,				//����ͼ��������������
	int Threshold);					//������������,mhi���ڸ�ֵ�ŰѶ�Ӧ��maskT��Ϊ1

//�������
int track_find_contours(Itc_Mat_t* src1,	//�����ֵͼ��0��1����4�ܱ߽����Ϊ0�������Խ��
	ItcContour** pContour,				//���������
	ItcMemStorage*  storage);			//�洢��

//����ɸѡ
int stuTrack_filtrate_contours(ItcContour** pContour, int* size_Threshold, ItcRect *rect_arr);
int track_filtrate_contours(ItcContour** pContour,int size_Threshold, ItcRect *rect_arr);

bool track_intersect_rect(ItcRect *rectA, ItcRect *rectB);					//�ж��������ο��Ƿ��ཻ

int track_calculateDirect_ROI(Itc_Mat_t* src, ItcRect roi, int &Direct);	//���صķ���Directȡ1,2,3,4�ֱ�����ϣ��£�����

void track_update_midValueBK(Itc_Mat_t* mat, Itc_Mat_t* matBK);				//����ֵ�����±���

void stuTrack_matching_ROI(ItcRect roi, Teack_Stand_t teack_stand[], int &count_trackObj);
void stuTrack_analyze_ROI(Itc_Mat_t* mhi,Teack_Stand_t teack_stand[], int &count_trackObj);
bool stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, Teack_Stand_t teack_stand);
#endif // itcCore_h__
/*******************************************************************************
文件名称：itcCore.h 
功    能：图像算法基础数据结构
时    间：9/25/2015 XUE
版    本：1.0
注    意：
*******************************************************************************/
#ifndef itcCore_h__
#define itcCore_h__
#include <stdlib.h>
#include <stdio.h>

#include "itctypes.h"

typedef __int64 int64;
typedef unsigned __int64 uint64;

typedef unsigned char uchar;
typedef unsigned short ushort;

#define ITC_PI   3.1415926535897932384626433832795
#define ITC_LOG2 0.69314718055994530941723212145818

#define ITC_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))

#define ITC_MIN(a,b)  ((a) > (b) ? (b) : (a))
#define ITC_MAX(a,b)  ((a) < (b) ? (b) : (a))
/* min & max without jumps */
#define  ITC_IMIN(a, b)  ((a) ^ (((a)^(b)) & (((a) < (b)) - 1)))

#define  ITC_IMAX(a, b)  ((a) ^ (((a)^(b)) & (((a) > (b)) - 1)))

/* absolute value without jumps */
#define  ITC_CMP(a,b)    (((a) > (b)) - ((a) < (b)))
#define  ITC_SIGN(a)     ITC_CMP((a),0)


#define ITC_CN_MAX     64
#define ITC_CN_SHIFT   3
#define ITC_DEPTH_MAX  (1 << ITC_CN_SHIFT)

#define ITC_8U   0
#define ITC_8S   1
#define ITC_16U  2
#define ITC_16S  3
#define ITC_32S  4
#define ITC_32F  5
#define ITC_64F  6
#define ITC_USRTYPE1 7

#define ITC_MAT_DEPTH_MASK       (ITC_DEPTH_MAX - 1)									// 0000 0111
#define ITC_MAT_DEPTH(flags)     ((flags) & ITC_MAT_DEPTH_MASK)							//获取深度（低3位）

#define ITC_MAKETYPE(depth,cn) (ITC_MAT_DEPTH(depth) + (((cn)-1) << ITC_CN_SHIFT))		//低3位表示类型，其余高位中表示通道数
#define ITC_MAKE_TYPE ITC_MAKETYPE

#define ITC_8UC1 ITC_MAKETYPE(ITC_8U,1)
#define ITC_8UC2 ITC_MAKETYPE(ITC_8U,2)
#define ITC_8UC3 ITC_MAKETYPE(ITC_8U,3)
#define ITC_8UC4 ITC_MAKETYPE(ITC_8U,4)
#define ITC_8UC(n) ITC_MAKETYPE(ITC_8U,(n))

#define ITC_8SC1 ITC_MAKETYPE(ITC_8S,1)
#define ITC_8SC2 ITC_MAKETYPE(ITC_8S,2)
#define ITC_8SC3 ITC_MAKETYPE(ITC_8S,3)
#define ITC_8SC4 ITC_MAKETYPE(ITC_8S,4)
#define ITC_8SC(n) ITC_MAKETYPE(ITC_8S,(n))

#define ITC_16UC1 ITC_MAKETYPE(ITC_16U,1)
#define ITC_16UC2 ITC_MAKETYPE(ITC_16U,2)
#define ITC_16UC3 ITC_MAKETYPE(ITC_16U,3)
#define ITC_16UC4 ITC_MAKETYPE(ITC_16U,4)
#define ITC_16UC(n) ITC_MAKETYPE(ITC_16U,(n))

#define ITC_16SC1 ITC_MAKETYPE(ITC_16S,1)
#define ITC_16SC2 ITC_MAKETYPE(ITC_16S,2)
#define ITC_16SC3 ITC_MAKETYPE(ITC_16S,3)
#define ITC_16SC4 ITC_MAKETYPE(ITC_16S,4)
#define ITC_16SC(n) ITC_MAKETYPE(ITC_16S,(n))

#define ITC_32SC1 ITC_MAKETYPE(ITC_32S,1)
#define ITC_32SC2 ITC_MAKETYPE(ITC_32S,2)
#define ITC_32SC3 ITC_MAKETYPE(ITC_32S,3)
#define ITC_32SC4 ITC_MAKETYPE(ITC_32S,4)
#define ITC_32SC(n) ITC_MAKETYPE(ITC_32S,(n))

#define ITC_32FC1 ITC_MAKETYPE(ITC_32F,1)
#define ITC_32FC2 ITC_MAKETYPE(ITC_32F,2)
#define ITC_32FC3 ITC_MAKETYPE(ITC_32F,3)
#define ITC_32FC4 ITC_MAKETYPE(ITC_32F,4)
#define ITC_32FC(n) ITC_MAKETYPE(ITC_32F,(n))

#define ITC_64FC1 ITC_MAKETYPE(ITC_64F,1)
#define ITC_64FC2 ITC_MAKETYPE(ITC_64F,2)
#define ITC_64FC3 ITC_MAKETYPE(ITC_64F,3)
#define ITC_64FC4 ITC_MAKETYPE(ITC_64F,4)
#define ITC_64FC(n) ITC_MAKETYPE(ITC_64F,(n))

#define ITC_AUTO_STEP  0x7fffffff
#define ITC_WHOLE_ARR  ITCSlice( 0, 0x3fffffff )

#define ITC_MAT_CN_MASK          ((ITC_CN_MAX - 1) << ITC_CN_SHIFT)							//通道掩码
#define ITC_MAT_CN(flags)        ((((flags) & ITC_MAT_CN_MASK) >> ITC_CN_SHIFT) + 1)		//获得通道数（第4到12位）
#define ITC_MAT_TYPE_MASK        (ITC_DEPTH_MAX*ITC_CN_MAX - 1)
#define ITC_MAT_TYPE(flags)      ((flags) & ITC_MAT_TYPE_MASK)								//获得数据类型和通道
#define ITC_MAT_CONT_FLAG_SHIFT  14
#define ITC_MAT_CONT_FLAG        (1 << ITC_MAT_CONT_FLAG_SHIFT)								//第15位
#define ITC_IS_MAT_CONT(flags)   ((flags) & ITC_MAT_CONT_FLAG)
#define ITC_IS_CONT_MAT          ITC_IS_MAT_CONT
#define ITC_SUBMAT_FLAG_SHIFT    15
#define ITC_SUBMAT_FLAG          (1 << ITC_SUBMAT_FLAG_SHIFT)								//第16位
#define ITC_IS_SUBMAT(flags)     ((flags) & ITC_MAT_SUBMAT_FLAG)

#define ITC_MAGIC_MASK       0xFFFF0000
#define ITC_MAT_MAGIC_VAL    0x42420000

#define ITC_MALLOC_ALIGN    16				//用于对其Mat数据指针
#define INT_MAX         2147483647
#define NULL	0

#define ITC_AUTOSTEP  0x7fffffff

#define ITC_ERROR_(errors) printf("error:%s\n",errors);

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

#define ITC_IS_MAT_HDR(mat) \
	((mat) != NULL && \
	(((const ItcMat*)(mat))->type & CV_MAGIC_MASK) == ITC_MAT_MAGIC_VAL && \
	((const ItcMat*)(mat))->cols > 0 && ((const ItcMat*)(mat))->rows > 0)		//判断是否是Mat

#define ITC_IS_MAT(mat) \
	(ITC_IS_MAT_HDR(mat) && ((const ItcMat*)(mat))->data.ptr != NULL)//是否有数据

#define ITC_IS_MASK_ARR(mat) \
	(((mat)->type & (ITC_MAT_TYPE_MASK & ~ITC_8SC1)) == 0)

#define ITC_ARE_TYPES_EQ(mat1, mat2) \
	((((mat1)->type ^ (mat2)->type) & ITC_MAT_TYPE_MASK) == 0)//判断mat1和mat2类型（通道数和深度）是否相同

#define ITC_ARE_CNS_EQ(mat1, mat2) \
	((((mat1)->type ^ (mat2)->type) & ITC_MAT_CN_MASK) == 0)//判断mat1和mat2通道数是否相等

#define ITC_ARE_DEPTHS_EQ(mat1, mat2) \
	((((mat1)->type ^ (mat2)->type) & ITC_MAT_DEPTH_MASK) == 0)//判断mat1和mat2深度是否相等

#define ITC_ARE_SIZES_EQ(mat1, mat2) \
	((mat1)->height == (mat2)->height && (mat1)->width == (mat2)->width)//判断mat1和mat2尺寸是否相等

#define ITC_IS_MAT_CONST(mat)  \
	(((mat)->height|(mat)->width) == 1)

#define ITC_ELEM_SIZE(type) \
	(ITC_MAT_CN(type) << ((((sizeof(size_t)/4+1)*0x4000|0x3a50) >> ITC_MAT_DEPTH(type)*2) & 3))	//计算单个元素所占的内存大小

/* general-purpose saturation macros */ 
#define  ITC_CAST_8U(t)  (uchar)(!((t) & ~255) ? (t) : (t) > 0 ? 255 : 0)					//((t) & ~255)不为0说明已经越界
#define  ITC_CAST_8S(t)  (char)(!(((t)+128) & ~255) ? (t) : (t) > 0 ? 127 : -128)
#define  ITC_CAST_16U(t) (ushort)(!((t) & ~65535) ? (t) : (t) > 0 ? 65535 : 0)
#define  ITC_CAST_16S(t) (short)(!(((t)+32768) & ~65535) ? (t) : (t) > 0 ? 32767 : -32768)
#define  ITC_CAST_32S(t) (int)(t)
#define  ITC_CAST_64S(t) (int64)(t)
#define  ITC_CAST_32F(t) (float)(t)
#define  ITC_CAST_64F(t) (double)(t)

static void iicvCheckHuge( ItcMat* arr );//检查需要分配的空间是否过大

ItcMat itcMat( int rows, int cols, int type, void* data);	//手动分配数据创建Mat,注意不能用itcReleaseMat释放
ItcMat*	itcCreateMat( int height, int width, int type );	//创建Mat
ItcMat*	itcCreateMatHeader( int rows, int cols, int type );	//创建Mat头
ItcMat*	itcInitMatHeader( ItcMat* arr, int rows, int cols, int type, void* data, int step );//有Mat头后初始化
void  itcReleaseMat( ItcMat** mat );						//释放Mat,包括头和数据


/*------------------------------------矩阵基本运算-------------------------------------------*/

typedef struct ItcFuncTable
{
	void*   fn_2d[ITC_DEPTH_MAX];
}
ItcFuncTable;

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
#define ITC_SUB_R(a,b) ((a) - (b))				//定义sub操作

ICV_DEF_BIN_ARI_ALL( ITC_SUB_R, Sub )			//定义sub操作的函数

void itcSub(ItcMat* src1,ItcMat* src2,ItcMat* dst);			//dst=src1-src2

#endif // itcCore_h__
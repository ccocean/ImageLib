/*******************************************************************************
�ļ����ƣ�itcCore.h 
��    �ܣ�ͼ���㷨�������ݽṹ
ʱ    �䣺9/25/2015 XUE
��    ����1.0
ע    �⣺
*******************************************************************************/
#ifndef itcCore_h__
#define itcCore_h__
#include <stdlib.h>

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


#define ITC_CN_MAX     512
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
#define ITC_MAT_DEPTH(flags)     ((flags) & ITC_MAT_DEPTH_MASK)							//��ȡ��ȣ���3λ��

#define ITC_MAKETYPE(depth,cn) (ITC_MAT_DEPTH(depth) + (((cn)-1) << ITC_CN_SHIFT))		//��3λ��ʾ���ͣ������λ�б�ʾͨ����
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

#define ITC_MAT_CN_MASK          ((ITC_CN_MAX - 1) << ITC_CN_SHIFT)							//��λ����
#define ITC_MAT_CN(flags)        ((((flags) & ITC_MAT_CN_MASK) >> ITC_CN_SHIFT) + 1)		//���ͨ��������4��12λ��
#define ITC_MAT_TYPE_MASK        (ITC_DEPTH_MAX*ITC_CN_MAX - 1)
#define ITC_MAT_TYPE(flags)      ((flags) & ITC_MAT_TYPE_MASK)								//����������ͺ�ͨ��
#define ITC_MAT_CONT_FLAG_SHIFT  14
#define ITC_MAT_CONT_FLAG        (1 << ITC_MAT_CONT_FLAG_SHIFT)								//��15λ
#define ITC_IS_MAT_CONT(flags)   ((flags) & ITC_MAT_CONT_FLAG)
#define ITC_IS_CONT_MAT          ITC_IS_MAT_CONT
#define ITC_SUBMAT_FLAG_SHIFT    15
#define ITC_SUBMAT_FLAG          (1 << ITC_SUBMAT_FLAG_SHIFT)								//��16λ
#define ITC_IS_SUBMAT(flags)     ((flags) & ITC_MAT_SUBMAT_FLAG)

#define ITC_MAGIC_MASK       0xFFFF0000
#define ITC_MAT_MAGIC_VAL    0x42420000

#define ITC_ELEM_SIZE(type) \
	(ITC_MAT_CN(type) << ((((sizeof(size_t)/4+1)*0x4000|0x3a50) >> ITC_MAT_DEPTH(type)*2) & 3))	//���㵥��Ԫ����ռ���ڴ��С

#define ITC_MALLOC_ALIGN    16				//���ڶ���Mat����ָ��
#define INT_MAX         2147483647
#define NULL	0

#define ITC_AUTOSTEP  0x7fffffff

#define ITC_ERROR_ ;

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

static void iicvCheckHuge( ItcMat* arr );//�����Ҫ����Ŀռ��Ƿ����

ItcMat itcMat( int rows, int cols, int type, void* data);	//�ֶ��������ݴ���Mat,ע�ⲻ����itcReleaseMat�ͷ�
ItcMat*	itcCreateMat( int height, int width, int type );	//����Mat
ItcMat*	itcCreateMatHeader( int rows, int cols, int type );	//����Matͷ
ItcMat*	itcInitMatHeader( ItcMat* arr, int rows, int cols, int type, void* data, int step );//��Matͷ���ʼ��
void  itcReleaseMat( ItcMat** mat );						//�ͷ�Mat,����ͷ������



#endif // itcCore_h__
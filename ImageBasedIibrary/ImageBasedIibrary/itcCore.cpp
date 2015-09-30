#include "itcCore.h"


ItcMat itcMat( int rows, int cols, int type, void* data)
{
	ItcMat m;

	type = ITC_MAT_TYPE(type);					//ֻ��ȡ��12λ����
	m.type = ITC_MAT_MAGIC_VAL | ITC_MAT_CONT_FLAG | type;
	m.cols = cols;
	m.rows = rows;
	m.step = m.cols*ITC_ELEM_SIZE(type);
	m.data.ptr = (uchar*)data;
	m.refcount = NULL;
	m.hdr_refcount = 0;

	return m;
}

ItcMat*	itcCreateMat( int height, int width, int type )
{
	ItcMat* arr = itcCreateMatHeader( height, width, type );
	size_t step, total_size;
	ItcMat* mat = (ItcMat*)arr;
	step = mat->step;

	if( mat->rows == 0 || mat->cols == 0 )
		return arr;

	if( mat->data.ptr != 0 )
		free( mat->data.ptr);//"Data is already allocated"

	if( step == 0 )
		step = ITC_ELEM_SIZE(mat->type)*mat->cols;

	int64 _total_size = (int64)step*mat->rows + sizeof(int) + ITC_MALLOC_ALIGN;	//int�����ڱ���ͳ�Ƽ����ģ�ITC_MALLOC_ALIGN�����ڴ�Ķ���
	total_size = (size_t)_total_size;
 	if(_total_size != (int64)total_size)
 		ITC_ERROR_("Too big buffer is allocated");//
	mat->refcount = (int*)malloc( (size_t)total_size );
	mat->data.ptr = (uchar*)( mat->refcount + 1);
	mat->data.ptr = (uchar*)(((size_t)mat->data.ptr + ITC_MALLOC_ALIGN - 1) &~ (size_t)(ITC_MALLOC_ALIGN - 1));//���뵽ITC_MALLOC_ALIGN����λ������˵��ַ��110��ITC_MALLOC_ALIGN=16����ô�Ͱѵ�ַ���뵽112�������ַ��120����ô�Ͷ��뵽128��
	*mat->refcount = 1;

	return arr;
}


static void iicvCheckHuge( ItcMat* arr )
{
	if( (int64)arr->step*arr->rows > INT_MAX )	//�������������С
		arr->type &= ~ITC_MAT_CONT_FLAG;		//����Ϊ������
}

ItcMat*	itcCreateMatHeader( int rows, int cols, int type )
{
	type = ITC_MAT_TYPE(type);

	if( rows < 0 || cols <= 0 )
		ITC_ERROR_("Non-positive width or height");//

	int min_step = ITC_ELEM_SIZE(type)*cols;
	if( min_step <= 0 )
		ITC_ERROR_("Invalid matrix type");//

	ItcMat* arr = (ItcMat*)malloc( sizeof(*arr));

	arr->step = min_step;
	arr->type = ITC_MAT_MAGIC_VAL | type | ITC_MAT_CONT_FLAG;
	arr->rows = rows;
	arr->cols = cols;
	arr->data.ptr = NULL;
	arr->refcount = NULL;
	arr->hdr_refcount = 1;

	iicvCheckHuge( arr );
	return arr;
}

ItcMat*	itcInitMatHeader( ItcMat* arr, int rows, int cols, int type, void* data, int step )
{
	if( !arr )
		ITC_ERROR_("����ͷΪ�գ�");

	if( (unsigned)ITC_MAT_DEPTH(type) > ITC_DEPTH_MAX )
		ITC_ERROR_("������ʹ���");

	if( rows < 0 || cols <= 0 )
		ITC_ERROR_("Non-positive cols or rows" );// "Non-positive cols or rows" 

	type = ITC_MAT_TYPE( type );
	arr->type = type | ITC_MAT_MAGIC_VAL;
	arr->rows = rows;
	arr->cols = cols;
	arr->data.ptr = (uchar*)data;
	arr->refcount = 0;
	arr->hdr_refcount = 0;

	int pix_size = ITC_ELEM_SIZE(type);
	int min_step = arr->cols*pix_size;

	if( step != ITC_AUTOSTEP && step != 0 )
	{
		if( step < min_step )
			ITC_ERROR_("���벽������");
		arr->step = step;
	}
	else
	{
		arr->step = min_step;
	}

	arr->type = ITC_MAT_MAGIC_VAL | type |
		(arr->rows == 1 || arr->step == min_step ? ITC_MAT_CONT_FLAG : 0);

	iicvCheckHuge( arr );
	return arr;
}

void itcReleaseMat(ItcMat** arr)
{
	ItcMat *mat=*arr;
	mat->data.ptr = NULL;
	if( mat->refcount != NULL && --*mat->refcount == 0 )//���ü���Ϊ0ʱ���ͷ������ڴ�
		free( mat->refcount );
	mat->refcount = NULL;
	free(mat);
	mat = NULL;
}

void itcSub(ItcMat* src1,ItcMat* src2,ItcMat* dst)
{
	if( !ITC_ARE_TYPES_EQ( src1, src2 ) || !ITC_ARE_TYPES_EQ( src1, dst ))//��������Ƿ�һ��
		ITC_ERROR_("�������Ͳ�һ��");

	if( !ITC_ARE_SIZES_EQ( src1, src2 ) || !ITC_ARE_SIZES_EQ( src1, dst ))//����С�Ƿ�һ��
		ITC_ERROR_("�����С��һ��");

	int type = ITC_MAT_TYPE(src1->type);	//����
	int depth = ITC_MAT_DEPTH(type);		//���
	int cn = ITC_MAT_CN(type);				//ͨ����

	ItcSize sizeMat;
	sizeMat.width=src1->cols * cn;
	sizeMat.height=src1->rows;

	switch (depth)
	{
	case ITC_8U:
		icvSub_8u_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	case ITC_8S:
		icvSub_8s_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	case ITC_16U:
		icvSub_16u_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	case ITC_16S:
		icvSub_16s_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	case ITC_32S:
		icvSub_32s_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	case ITC_32F:
		icvSub_32f_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	case ITC_64F:
		icvSub_64f_C1R(src1->data.ptr,src1->step,src2->data.ptr,src2->step,dst->data.ptr,dst->step,sizeMat);
		break;
	default:
		break;
	}
}

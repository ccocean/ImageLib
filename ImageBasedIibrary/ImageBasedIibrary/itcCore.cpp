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
 		ITC_ERROR_;//"Too big buffer is allocated"
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
		ITC_ERROR_;//"Non-positive width or height"

	int min_step = ITC_ELEM_SIZE(type)*cols;
	if( min_step <= 0 )
		ITC_ERROR_;//"Invalid matrix type"

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
		ITC_ERROR_;

	if( (unsigned)ITC_MAT_DEPTH(type) > ITC_DEPTH_MAX )
		ITC_ERROR_;

	if( rows < 0 || cols <= 0 )
		ITC_ERROR_;// "Non-positive cols or rows" 

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
			ITC_ERROR_;
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

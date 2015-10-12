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
	total_size = (size_t)_total_size;				//����ϵͳ��ͬ����size_t���ͽ�ȡ��ϵͳ�ܷ���Ŀռ��С
 	if(_total_size != (int64)total_size)			//�������ȣ�˵���Ѿ����
 		ITC_ERROR_("Too big buffer is allocated");	//����Ŀռ䳬����ǰϵͳ��Ѱַ��Χ
	mat->refcount = (int*)malloc( (size_t)total_size );
	memset(mat->refcount, 0, (size_t)total_size);	//��ʼ��Ϊ0
	mat->data.ptr = (uchar*)( mat->refcount + 1);
	mat->data.ptr = (uchar*)(((size_t)mat->data.ptr + ITC_MALLOC_ALIGN - 1) &~ (size_t)(ITC_MALLOC_ALIGN - 1));//���뵽ITC_MALLOC_ALIGN����λ������˵��ַ��110��ITC_MALLOC_ALIGN=16����ô�Ͱѵ�ַ���뵽112�������ַ��120����ô�Ͷ��뵽128��
	*mat->refcount = 1;

	return arr;
}


static void iicvCheckHuge( ItcMat* arr )
{
	//�����Ҫ����Ŀռ��Ƿ����
	if( (int64)arr->step*arr->rows > INT_MAX )	//�������������С
		arr->type &= ~ITC_MAT_CONT_FLAG;		//����Ϊ������
}

ItcMat*	itcCreateMatHeader( int rows, int cols, int type )
{
	type = ITC_MAT_TYPE(type);

	if( rows < 0 || cols <= 0 )
		ITC_ERROR_("Non-positive width or height");				//

	int min_step = ITC_ELEM_SIZE(type)*cols;
	if( min_step <= 0 )
		ITC_ERROR_("Invalid matrix type");						//

	ItcMat* arr = (ItcMat*)malloc( sizeof(*arr));

	arr->step = min_step;
	arr->type = ITC_MAT_MAGIC_VAL | type | ITC_MAT_CONT_FLAG;	//ITC_MAT_MAGIC_VALΪMat�ṹ�ı�־�������ж��Ƿ���mat�ṹ
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

void itcUpdateMHI(ItcMat* src1, ItcMat* src2, ItcMat* mhi, int diffThreshold, ItcMat* maskT, int Threshold)
{
	if (!ITC_ARE_TYPES_EQ(src1, src2) || !ITC_ARE_TYPES_EQ(src1, mhi))//��������Ƿ�һ��
		ITC_ERROR_("�������Ͳ�һ��");

	if (!ITC_ARE_SIZES_EQ(src1, src2) || !ITC_ARE_SIZES_EQ(src1, mhi))//����С�Ƿ�һ��
		ITC_ERROR_("�����С��һ��");

	int type = ITC_MAT_TYPE(src1->type);	//����
	int depth = ITC_MAT_DEPTH(type);		//���
	if( ITC_MAT_CN(type)!=1 )				//ͨ����
		ITC_ERROR_("ͨ������Ϊ1");

	ItcSize sizeMat;
	sizeMat.width = src1->cols;
	sizeMat.height = src1->rows;

	int i = 0;
	int j = 0;
	uchar *qsrc1 = src1->data.ptr;
	uchar *qsrc2 = src2->data.ptr;
	uchar *qmhi = mhi->data.ptr;
	if (maskT == NULL)
	{
		for (i = 0; i < sizeMat.height; i++)
		{
			for (j = 0; j < sizeMat.width; j++)
			{
				int k = abs(qsrc1[j] - qsrc2[j]);
				if ( k > diffThreshold )
				{
					qmhi[j] = 255;
				}
				else
				{
					//mhi����ȡС��0��ֵ
					qmhi[j] = ITC_IMAX(qmhi[j], 1);
					qmhi[j]--;
				}
			}
			qsrc1 += src1->step;
			qsrc2 += src2->step;
			qmhi += mhi->step;
		}
	}
	else
	{
		uchar *qmask = maskT->data.ptr;
		//��������Ҫʹ���ܱ߽綼Ϊ0�����ڽ����������
		qsrc1 += src1->step;
		qsrc2 += src2->step;
		qmhi += mhi->step;
		qmask += maskT->step;
		if (!ITC_ARE_TYPES_EQ(src1, maskT))//��������Ƿ�һ��
			ITC_ERROR_("�������Ͳ�һ��");
		if (!ITC_ARE_SIZES_EQ(src1, maskT))//����С�Ƿ�һ��
			ITC_ERROR_("�����С��һ��");
		for (i = 1; i < sizeMat.height - 1; i++)
		{
			for (j = 1; j < sizeMat.width - 1; j++)
			{
				int k = abs(qsrc1[j] - qsrc2[j]);
				if (k > diffThreshold)
				{
					qmhi[j] = 255;
				}
				else
				{
					//mhi����ȡС��0��ֵ
					qmhi[j] = ITC_IMAX(qmhi[j], 1);
					qmhi[j]--;
				}

				qmask[j] = qmhi[j] > Threshold ? 1 : 0;//����һ����ֵ������
			}
			qsrc1 += src1->step;
			qsrc2 += src2->step;
			qmhi += mhi->step;
			qmask += maskT->step;
		}
	}
}

/* initializes 8-element array for fast access to 3x3 neighborhood of a pixel */
#define  ITC_INIT_3X3_DELTAS( deltas, step, nch )				\
	((deltas)[0] = (nch), (deltas)[1] = -(step)+(nch),			\
	(deltas)[2] = -(step), (deltas)[3] = -(step)-(nch),			\
	(deltas)[4] = -(nch), (deltas)[5] = (step)-(nch),			\
	(deltas)[6] = (step), (deltas)[7] = (step)+(nch))

static const ItcPoint icvCodeDeltas[8] =
{ { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 } };

static int itcFetchContourEx(char*		ptr,
int							step,
ItcPoint					pt,
ItcSeq*					contour,
int							nbd)
{
	int         deltas[16];
	char        *i0 = ptr, *i1, *i3, *i4;
	ItcRect      rect;
	int         prev_s = -1, s, s_end;
	ItcSeqWriter writer;

	assert(1 < nbd && nbd < 128);

	/* initialize local state */
	ITC_INIT_3X3_DELTAS(deltas, step, 1);
	memcpy(deltas + 8, deltas, 8 * sizeof(deltas[0]));

	/* initialize writer */
	itcStartAppendToSeq((ItcSeq*)contour, &writer);

	rect.x = rect.width = pt.x;
	rect.y = rect.height = pt.y;

	s_end = s = (contour->flags) ? 0 : 4;	//�ж��Ƿ��ǿ�,�ǿ׾ʹ��ұ߿�ʼɨ�����������ʹ���߿�ʼɨ

	do
	{
		s = (s - 1) & 7;				//˳ʱ���ұ�Ե
		i1 = i0 + deltas[s];
		if (*i1 != 0)
			break;
	} while (s != s_end);

	if (s == s_end)						//ɨ��һȦû���ҵ�������Ե��˵���ǵ�һһ����
	{
		*i0 = (char)(nbd | 0x80);		//��char�������λ��Ϊ1,��Ϊ����Ҳ��һ���ұ�Ե��
		ITC_WRITE_SEQ_ELEM(pt, writer);//�����
	}
	else
	{
		i3 = i0;
		prev_s = s ^ 4;

		//���ٱ�Ե
		for (;;)
		{
			s_end = s;
			for (;;)
			{
				i4 = i3 + deltas[++s];	//��ʱ��ɨ��8����
				if (*i4 != 0)
					break;
			}
			s &= 7;

			//����ұ�Ե
			if ((unsigned)(s - 1) < (unsigned)s_end)
			{
				*i3 = (char)(nbd | 0x80);//���ұ�Ե���ֵ��λ����Ϊ1����Ϊ-128 = 0x80 =1000 0000
			}
			else if (*i3 == 1)
			{
				*i3 = (char)nbd;//���ұ�Եֵ��λ����Ϊ0
			}

			if (s != prev_s)//ѹ��ͬ����ĵ�
			{
				ITC_WRITE_SEQ_ELEM(pt, writer);//�����
			}

			if (s != prev_s)
			{
				//���±߽�
				if (pt.x < rect.x)
					rect.x = pt.x;
				else if (pt.x > rect.width)
					rect.width = pt.x;

				if (pt.y < rect.y)
					rect.y = pt.y;
				else if (pt.y > rect.height)
					rect.height = pt.y;
			}

			prev_s = s;
			pt.x += icvCodeDeltas[s].x;//icvCodeDeltas��Ԥ�����úõ�ƫ���������ݷ���s
			pt.y += icvCodeDeltas[s].y;

			if (i4 == i0 && i3 == i1)  break;

			i3 = i4;
			s = (s + 4) & 7;
		}                       /* end of border following loop */
	}

	rect.width -= rect.x - 1;
	rect.height -= rect.y - 1;
	((ItcContour*)(contour))->rect = rect;
	itcEndWriteSeq(&writer);

	return 1;
}

int itcFindContours(ItcMat* src, ItcContour** pContour, ItcMemStorage*  storage)
{
	int step = src->step;
	char *img0 = (char*)(src->data.ptr);
	char *img = (char*)(src->data.ptr + step);
	
	int width = src->cols - 1;
	int height = src->rows - 1;

	ItcPoint lnbd = itcPoint(0, 1);	//��¼��һ��ɨ�赽��Ե���λ��
	int x = 1;						//ɨ����ʼλ��
	int y = 1;
	int prev = img[x - 1];

	int count = 0;
	for (; y < height; y++, img += step)		//�У���������
	{
		for (x = 1; x < width; x++)				//�У���������
		{
			int p = img[x];
			if (p != prev)						//�ҵ�һ����Ե��
			{
				int is_hole = 0;
				ItcPoint origin;				//ɨ�����
				if (!(prev == 0 && p == 1))		//���û���ҵ���������0->1��ע�����������ͱ�Ե����Ϊ���������ұ�ԵҲ��1->0��Ҫ��׽������֣�
				{
					//����Ƿ��ǿ�
					if (p != 0 || prev < 1)		//��Ӧ����(p=0 && prev>=1)
						goto resume_scan;		//����ɨ����
					is_hole = 1;				//���ÿױ�־
				}
				count++;
				ItcSeq* contour = itcCreateSeq(0, sizeof(ItcContour), sizeof(ItcPoint), storage);
				contour->flags = is_hole;
				//���ٱ�Ե�����
				origin.y = y;
				origin.x = x - is_hole;			//���������������ǿף���Ե��ɨ����㣩��ȡ��Ϊ0������
				itcFetchContourEx(img + x - is_hole, step, itcPoint(origin.x, origin.y), contour, 126);
				lnbd.x = x - is_hole;			//��ǰɨ�赽��Ե���λ�ã������´�ɨ���ж��Ƿ��а�����ϵ

				if ((*pContour) == NULL)
				{
					(*pContour) = (ItcContour*)contour;
				}
				else
				{
					//����
					contour->h_next = (*pContour)->h_next;
					(*pContour)->h_next = contour;			
				}
			resume_scan:
				prev = img[x];		//����ֱ�ӵ���p,��ΪitcFetchContourEx��ı䵱ǰɨ����ĵ�
				if (prev & -2)		//ֻ������֪�ı�Ե
				{
					lnbd.x = x;		//��¼��ǰɨ�赽��Ե���λ�ã�������һɨ��ʹ��
				}
			}
		}
		lnbd.x = 0;
		lnbd.y = y + 1;
		prev = 0;
	}
	return count;
}

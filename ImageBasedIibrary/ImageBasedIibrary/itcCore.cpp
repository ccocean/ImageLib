#include "itcCore.h"


ItcMat itcMat( int rows, int cols, int type, void* data)
{
	ItcMat m;

	type = ITC_MAT_TYPE(type);					//只截取低12位数据
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

	int64 _total_size = (int64)step*mat->rows + sizeof(int) + ITC_MALLOC_ALIGN;	//int是用于保存统计计数的，ITC_MALLOC_ALIGN用与内存的对齐
	total_size = (size_t)_total_size;				//根据系统不同，用size_t类型截取本系统能分配的空间大小
 	if(_total_size != (int64)total_size)			//如果不相等，说明已经溢出
 		ITC_ERROR_("Too big buffer is allocated");	//分配的空间超出当前系统的寻址范围
	mat->refcount = (int*)malloc( (size_t)total_size );
	memset(mat->refcount, 0, (size_t)total_size);	//初始化为0
	mat->data.ptr = (uchar*)( mat->refcount + 1);
	mat->data.ptr = (uchar*)(((size_t)mat->data.ptr + ITC_MALLOC_ALIGN - 1) &~ (size_t)(ITC_MALLOC_ALIGN - 1));//对齐到ITC_MALLOC_ALIGN整数位，比如说地址是110，ITC_MALLOC_ALIGN=16，那么就把地址对齐到112，如果地址是120，那么就对齐到128，
	*mat->refcount = 1;

	return arr;
}


static void iicvCheckHuge( ItcMat* arr )
{
	//检查需要分配的空间是否过大
	if( (int64)arr->step*arr->rows > INT_MAX )	//不超出最大分配大小
		arr->type &= ~ITC_MAT_CONT_FLAG;		//设置为不连续
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
	arr->type = ITC_MAT_MAGIC_VAL | type | ITC_MAT_CONT_FLAG;	//ITC_MAT_MAGIC_VAL为Mat结构的标志，用于判断是否是mat结构
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
		ITC_ERROR_("矩阵头为空！");

	if( (unsigned)ITC_MAT_DEPTH(type) > ITC_DEPTH_MAX )
		ITC_ERROR_("深度类型错误！");

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
			ITC_ERROR_("输入步长有误！");
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
	if( mat->refcount != NULL && --*mat->refcount == 0 )//引用计数为0时才释放数据内存
		free( mat->refcount );
	mat->refcount = NULL;
	free(mat);
	mat = NULL;
}

void itcSub(ItcMat* src1,ItcMat* src2,ItcMat* dst)
{
	if( !ITC_ARE_TYPES_EQ( src1, src2 ) || !ITC_ARE_TYPES_EQ( src1, dst ))//检测类型是否一致
		ITC_ERROR_("矩阵类型不一致");

	if( !ITC_ARE_SIZES_EQ( src1, src2 ) || !ITC_ARE_SIZES_EQ( src1, dst ))//检查大小是否一致
		ITC_ERROR_("矩阵大小不一致");

	int type = ITC_MAT_TYPE(src1->type);	//类型
	int depth = ITC_MAT_DEPTH(type);		//深度
	int cn = ITC_MAT_CN(type);				//通道数

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
	if (!ITC_ARE_TYPES_EQ(src1, src2) || !ITC_ARE_TYPES_EQ(src1, mhi))//检测类型是否一致
		ITC_ERROR_("矩阵类型不一致");

	if (!ITC_ARE_SIZES_EQ(src1, src2) || !ITC_ARE_SIZES_EQ(src1, mhi))//检查大小是否一致
		ITC_ERROR_("矩阵大小不一致");

	int type = ITC_MAT_TYPE(src1->type);	//类型
	int depth = ITC_MAT_DEPTH(type);		//深度
	if( ITC_MAT_CN(type)!=1 )				//通道数
		ITC_ERROR_("通道数不为1");

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
					//mhi不能取小于0的值
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
		//生成掩码要使四周边界都为0，用于进行轮廓检测
		qsrc1 += src1->step;
		qsrc2 += src2->step;
		qmhi += mhi->step;
		qmask += maskT->step;
		if (!ITC_ARE_TYPES_EQ(src1, maskT))//检测类型是否一致
			ITC_ERROR_("矩阵类型不一致");
		if (!ITC_ARE_SIZES_EQ(src1, maskT))//检查大小是否一致
			ITC_ERROR_("矩阵大小不一致");
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
					//mhi不能取小于0的值
					qmhi[j] = ITC_IMAX(qmhi[j], 1);
					qmhi[j]--;
				}

				qmask[j] = qmhi[j] > Threshold ? 1 : 0;//生成一个二值化掩码
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

	s_end = s = (contour->flags) ? 0 : 4;	//判断是否是孔,是孔就从右边开始扫，是外轮廓就从左边开始扫

	do
	{
		s = (s - 1) & 7;				//顺时针找边缘
		i1 = i0 + deltas[s];
		if (*i1 != 0)
			break;
	} while (s != s_end);

	if (s == s_end)						//扫了一圈没有找到其他边缘，说明是单一一个点
	{
		*i0 = (char)(nbd | 0x80);		//把char类型最高位置为1,因为单点也是一个右边缘点
		ITC_WRITE_SEQ_ELEM(pt, writer);//保存点
	}
	else
	{
		i3 = i0;
		prev_s = s ^ 4;

		//跟踪边缘
		for (;;)
		{
			s_end = s;
			for (;;)
			{
				i4 = i3 + deltas[++s];	//逆时针扫描8邻域
				if (*i4 != 0)
					break;
			}
			s &= 7;

			//检查右边缘
			if ((unsigned)(s - 1) < (unsigned)s_end)
			{
				*i3 = (char)(nbd | 0x80);//把右边缘点的值首位设置为1，因为-128 = 0x80 =1000 0000
			}
			else if (*i3 == 1)
			{
				*i3 = (char)nbd;//非右边缘值首位保持为0
			}

			if (s != prev_s)//压缩同方向的点
			{
				ITC_WRITE_SEQ_ELEM(pt, writer);//保存点
			}

			if (s != prev_s)
			{
				//更新边界
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
			pt.x += icvCodeDeltas[s].x;//icvCodeDeltas是预先设置好的偏移量，根据方向s
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

	ItcPoint lnbd = itcPoint(0, 1);	//记录上一次扫描到边缘点的位置
	int x = 1;						//扫描起始位置
	int y = 1;
	int prev = img[x - 1];

	int count = 0;
	for (; y < height; y++, img += step)		//行，从上至下
	{
		for (x = 1; x < width; x++)				//列，从左至右
		{
			int p = img[x];
			if (p != prev)						//找到一个边缘点
			{
				int is_hole = 0;
				ItcPoint origin;				//扫描起点
				if (!(prev == 0 && p == 1))		//如果没有找到外轮廓（0->1，注意区分左右型边缘，因为外轮廓的右边缘也是1->0，要与孔进行区分）
				{
					//检查是否是孔
					if (p != 0 || prev < 1)		//孔应该是(p=0 && prev>=1)
						goto resume_scan;		//跳出扫描器
					is_hole = 1;				//设置孔标志
				}
				count++;
				ItcSeq* contour = itcCreateSeq(0, sizeof(ItcContour), sizeof(ItcPoint), storage);
				contour->flags = is_hole;
				//跟踪边缘的起点
				origin.y = y;
				origin.x = x - is_hole;			//不管是外轮廓还是孔，边缘（扫描起点）都取不为0的像素
				itcFetchContourEx(img + x - is_hole, step, itcPoint(origin.x, origin.y), contour, 126);
				lnbd.x = x - is_hole;			//当前扫描到边缘点的位置，用于下次扫描判断是否有包含关系

				if ((*pContour) == NULL)
				{
					(*pContour) = (ItcContour*)contour;
				}
				else
				{
					//插入
					contour->h_next = (*pContour)->h_next;
					(*pContour)->h_next = contour;			
				}
			resume_scan:
				prev = img[x];		//不能直接等于p,因为itcFetchContourEx会改变当前扫描过的点
				if (prev & -2)		//只保存已知的边缘
				{
					lnbd.x = x;		//记录当前扫描到边缘点的位置，用于下一扫描使用
				}
			}
		}
		lnbd.x = 0;
		lnbd.y = y + 1;
		prev = 0;
	}
	return count;
}

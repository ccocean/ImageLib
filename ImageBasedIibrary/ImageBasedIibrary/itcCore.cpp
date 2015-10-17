#include "itcCore.h"


Itc_Mat_t itc_mat( int rows, int cols, int type, void* data)
{
	Itc_Mat_t m;

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

Itc_Mat_t*	itc_create_mat( int height, int width, int type )
{
	Itc_Mat_t* arr = itc_create_matHeader( height, width, type );
	size_t step, total_size;
	Itc_Mat_t* mat = (Itc_Mat_t*)arr;
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


static void iicvCheckHuge( Itc_Mat_t* pMat )
{
	//检查需要分配的空间是否过大
	if ((int64)pMat->step*pMat->rows > INT_MAX)	//不超出最大分配大小
		pMat->type &= ~ITC_MAT_CONT_FLAG;		//设置为不连续
}

Itc_Mat_t*	itc_create_matHeader( int rows, int cols, int type )
{
	type = ITC_MAT_TYPE(type);

	if( rows < 0 || cols <= 0 )
		ITC_ERROR_("Non-positive width or height");				//

	int min_step = ITC_ELEM_SIZE(type)*cols;
	if( min_step <= 0 )
		ITC_ERROR_("Invalid matrix type");						//

	Itc_Mat_t* pMat = (Itc_Mat_t*)malloc(sizeof(*pMat));
	memset(pMat, 0, sizeof(*pMat));								//

	pMat->step = min_step;
	pMat->type = ITC_MAT_MAGIC_VAL | type | ITC_MAT_CONT_FLAG;	//ITC_MAT_MAGIC_VAL为Mat结构的标志，用于判断是否是mat结构
	pMat->rows = rows;
	pMat->cols = cols;
	pMat->data.ptr = NULL;
	pMat->refcount = NULL;
	pMat->hdr_refcount = 1;

	iicvCheckHuge(pMat);
	return pMat;
}

Itc_Mat_t*	itc_init_matHeader( Itc_Mat_t* arr, int rows, int cols, int type, void* data, int step )
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

void itc_release_mat(Itc_Mat_t** arr)
{
	Itc_Mat_t *mat=*arr;
	mat->data.ptr = NULL;
	if( mat->refcount != NULL && --*mat->refcount == 0 )//引用计数为0时才释放数据内存
		free( mat->refcount );
	mat->refcount = NULL;
	free(mat);
	mat = NULL;
}

void itc_sub_mat(Itc_Mat_t* src1, Itc_Mat_t* src2, Itc_Mat_t* dst)
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

void track_update_MHI(Itc_Mat_t* src1, Itc_Mat_t* src2, Itc_Mat_t* mhi, int diffThreshold, Itc_Mat_t* maskT, int Threshold)
{
	if (!ITC_ARE_TYPES_EQ(src1, src2) || !ITC_ARE_TYPES_EQ(src1, mhi))//检测类型是否一致
		ITC_ERROR_("矩阵类型不一致");

	if (!ITC_ARE_SIZES_EQ(src1, src2) || !ITC_ARE_SIZES_EQ(src1, mhi))//检查大小是否一致
		ITC_ERROR_("矩阵大小不一致");

	int type = ITC_MAT_TYPE(src1->type);	//类型
	int depth = ITC_MAT_DEPTH(type);		//深度
	if (depth != ITC_8U)
		ITC_ERROR_("数据深度不是uchar");
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
					qmhi[j] = qmhi[j] > 210 ? qmhi[j] : 1;
					//qmhi[j] = ITC_IMAX(qmhi[j], 1);
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
					qmask[j] = 1;//生成一个二值化掩码
					qmhi[j] = 255;
				}
				else
				{
					//mhi不能取小于0的值
					qmask[j] = qmhi[j] > Threshold;//生成一个二值化掩码
					qmhi[j] = qmhi[j] > 210 ? qmhi[j] : 1;
					//qmhi[j] = ITC_IMAX(qmhi[j], 1);
					qmhi[j]--;
				}
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

int track_find_contours(Itc_Mat_t* src, ItcContour** pContour, ItcMemStorage*  storage)
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
					(*pContour)->h_prev = (*pContour)->h_next = contour;
				}
				else
				{
					//插入
					contour->h_next = (*pContour)->h_next;
					(*pContour)->h_next = contour;
					contour->h_prev = (ItcSeq*)(*pContour);
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

#define INTERSECT_TRACK_EXTERN_RECT 5
bool track_intersect_rect(ItcRect *rectA, ItcRect *rectB)
{
	int x1_A = rectA->x;
	int y1_A = rectA->y;
	int x2_A = rectA->x + rectA->width;
	int y2_A = rectA->y + rectA->height;

	int x1_B = rectB->x;
	int y1_B = rectB->y;
	int x2_B = rectB->x + rectB->width;
	int y2_B = rectB->y + rectB->height;

	int x1_min = ITC_MIN(x1_A, x1_B);
	int y1_min = ITC_MIN(y1_A, y1_B);
	int x1_max = ITC_MAX(x2_A, x2_B);
	int y1_max = ITC_MAX(y2_A, y2_B);

	if ((rectA->width + rectB->width + INTERSECT_TRACK_EXTERN_RECT> x1_max - x1_min)
		&& (rectA->height + rectB->height + INTERSECT_TRACK_EXTERN_RECT> y1_max - y1_min))
	{
		//合并到rectA
		rectA->x = x1_min;
		rectA->y = y1_min;
		rectA->width = x1_max - x1_min;
		rectA->height = y1_max - y1_min;
		return true;
	}
	return false;
}

int stuTrack_filtrate_contours(ItcContour** pContour,int* size_Threshold, ItcRect *rect_arr)
{
	if (rect_arr == NULL || *pContour == NULL)
		return 0;

	int count_rect = 0;
	
	ItcContour *Contour = *pContour;
	do
	{
		ItcRect rect = Contour->rect;
		int centre_x = rect.x + rect.width >> 1;
		if (rect.width > size_Threshold[centre_x] &&
			rect.height > size_Threshold[centre_x] &&
			rect.height > rect.width + (rect.width>>2))					//筛选
		{
			*(rect_arr + count_rect) = rect;
			count_rect++;
		}
		Contour = (ItcContour*)Contour->h_next;
	}while (Contour != *pContour);

	int i = 0, j = 0;
	if (count_rect < 100)
	{
		for (i = 0; i < count_rect; i++)
		{
			for (j = i + 1; j < count_rect; j++)
			{
				if (track_intersect_rect(rect_arr + i, rect_arr + j))		//判断是否相交，如果相交则直接合并
				{
					count_rect--;
					*(rect_arr + j) = *(rect_arr + count_rect);
					j--;
				}
			}
		}
	}

	return count_rect;
}

int track_filtrate_contours(ItcContour** pContour, int size_Threshold, ItcRect *rect_arr)
{
	if (rect_arr == NULL || *pContour == NULL)
		return 0;

	int count_rect = 0;

	ItcContour *Contour = *pContour;
	do
	{
		ItcRect rect = Contour->rect;
		int centre_x = rect.x + rect.width >> 1;
		if (rect.width > size_Threshold &&
			rect.height > size_Threshold)					//筛选
		{
			*(rect_arr + count_rect) = rect;
			count_rect++;
		}
		Contour = (ItcContour*)Contour->h_next;
	} while (Contour != *pContour);

	int i = 0, j = 0;
	for (i = 0; i < count_rect; i++)
	{
		for (j = i + 1; j < count_rect; j++)
		{
			if (track_intersect_rect(rect_arr + i, rect_arr + j))		//判断是否相交，如果相交则直接合并
			{
				count_rect--;
				*(rect_arr + j) = *(rect_arr + count_rect);
			}
		}
	}

	return count_rect;
}

//************************************
// 函数名称: track_caluclateDirect_ROI
// 函数说明：本函数假设运动像素速度较小，只计算连续的运动，即时间戳之差为1的运动
// 作    者：XueYB
// 作成日期：2015/10/14
// 返 回 值: 
// 参    数: 
//************************************
int track_calculateDirect_ROI(Itc_Mat_t* src, ItcRect roi, int &direct)
{
	int sum_gradientV = 0;		//垂直方向梯度
	int sum_gradientH = 0;		//水平方向
	int count_change = 0;
	int count_changeX = 0;		//统计有偏移的点数量
	int count_changeY = 0;		//

	int x1 = roi.x;
	int y1 = roi.y;
	int x2 = roi.x + roi.width;
	int y2 = roi.y + roi.height;

	int step = src->step;
	uchar *img0 = (uchar*)(src->data.ptr + step*y1);
	uchar *img1 = (uchar*)(src->data.ptr + step*(y1 + 1));

	int i = 0, j = 0;
	for (i = y1; i < y2-1; i++)
	{
		uchar last_Value = 0;
		int startX = x1 - 1;
		for (j = x1; j < x2-1; j++)
		{
			if (img1[j] != 0)
			{
				int gradientX = img0[j + 1] - img0[j];
				int gradientY = img1[j] - img0[j];
				if (gradientX <= 1 && gradientX >= -1)
				{
					sum_gradientH += gradientX;
					count_changeX++;
				}
				if (gradientY <= 1 && gradientY >= -1)
				{
					sum_gradientV += gradientY;
					count_changeY++;
				}
				count_change++;
			}
		}
		img0 = img1;
		img1 += step;
	}

	int threshold = (roi.width*roi.height) >> 2;
	if (count_change>threshold)
	{
		if (abs(sum_gradientV) > abs(sum_gradientH))
		{
			int threshold1 = count_changeY >> 4;
			if (sum_gradientV > threshold1)
			{
				//printf("位置：%d,%d,大小：%d,%d 垂直梯度：%d,水平梯度：%d\n", x1, y1, roi.width, roi.height, sum_gradientV, sum_gradientH);
				direct = 1;
				return 1;
			}
			else if (sum_gradientV < -threshold1)
			{
				direct = 2;
			}
		}
		else
		{
			int threshold2 = count_changeX >> 4;
			if (sum_gradientH > threshold2)
			{
				direct = 3;
			}
			else if (sum_gradientH < -threshold2)
			{
				direct = 4;
			}
		}		
	}
	return 0;
}

void track_update_midValueBK(Itc_Mat_t* mat, Itc_Mat_t* matBK)
{
	if (!ITC_ARE_TYPES_EQ(mat, matBK))		//检测类型是否一致
		ITC_ERROR_("矩阵类型不一致");

	if (!ITC_ARE_SIZES_EQ(mat, matBK))		//检查大小是否一致
		ITC_ERROR_("矩阵大小不一致");

	int type = ITC_MAT_TYPE(mat->type);		//类型
	int depth = ITC_MAT_DEPTH(type);		//深度
	if (depth != ITC_8U)
		ITC_ERROR_("数据深度不是uchar");
	if (ITC_MAT_CN(type) != 1)				//通道数
		ITC_ERROR_("通道数不为1");

	ItcSize sizeMat;
	sizeMat.width = mat->cols;
	sizeMat.height = mat->rows;

	int i = 0;
	int j = 0;
	uchar *qmat = mat->data.ptr;
	uchar *qmBK = matBK->data.ptr;

	for (i = 0; i < sizeMat.height; i++)
	{
		for (j = 0; j < sizeMat.width; j++)
		{
			int matValue = qmBK[j];
			int diff = abs(matValue - qmat[j]);
			if (diff<250)
			{
				if (qmat[j]>qmBK[j])
				{
					qmBK[j] = ITC_CAST_8U(++matValue);
				}
				else if (qmat[j] < qmBK[j])
				{
					qmBK[j] = ITC_CAST_8U(--matValue);
				}
			}
			else
			{
				qmBK[j] = qmat[j];
			}
		}
		qmat += mat->step;
		qmBK += matBK->step;
	}
}

void stuTrack_matching_ROI(ItcRect roi, Teack_Stand_t teack_stand[], int &count_trackObj)
{
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);
	if (count_trackObj > 0)
	{
		int min_ID = 0;
		int min_distance = INT_MAX;
		int distance = 0;
		for (int i = 0; i < count_trackObj; i++)
		{
			distance = (x - teack_stand[i].centre.x)*(x - teack_stand[i].centre.x) + (y - teack_stand[i].centre.y)*(y - teack_stand[i].centre.y);
			if (min_distance>distance)
			{
				min_distance = distance;
				min_ID = i;
			}
		}
		int threshold = (teack_stand[min_ID].roi.width * teack_stand[min_ID].roi.height)>>2;
		if (min_distance<threshold)
		{
			//teack_stand[min_ID].roi.x = ITC_IMIN(roi.x, teack_stand[min_ID].roi.x);
			//teack_stand[min_ID].roi.y = ITC_IMAX(roi.y, teack_stand[min_ID].roi.y);
			//teack_stand[min_ID].roi.width = ITC_IMAX(roi.width, teack_stand[min_ID].roi.width);
			//teack_stand[min_ID].roi.height = ITC_IMAX(roi.height, teack_stand[min_ID].roi.height);
			track_intersect_rect(&teack_stand[min_ID].roi, &roi);
			teack_stand[count_trackObj].centre = itcPoint(x, y);
			teack_stand[min_ID].count_teack++;
			teack_stand[min_ID].flag_matching = 1;
			return;
		}
	}

	//add
	teack_stand[count_trackObj].centre = itcPoint(x, y);
	teack_stand[count_trackObj].roi = roi;
	teack_stand[count_trackObj].count_teack = 1;
	teack_stand[count_trackObj].count_up = 0;
	teack_stand[count_trackObj].count_down = 0;
	teack_stand[count_trackObj].flag_Stand = 0;
	teack_stand[count_trackObj].flag_matching = 1;
	count_trackObj++;
}

void stuTrack_analyze_ROI(Itc_Mat_t* mhi, Teack_Stand_t teack_stand[], int &count_trackObj)
{
	int i = 0;
	int direct = 0;
	for (int i = 0; i < count_trackObj; i++)
	{
		if (teack_stand[i].flag_Stand==0)
		{
			if(!teack_stand[i].flag_matching)
			{
				if (track_calculateDirect_ROI(mhi, teack_stand[i].roi, direct))
					teack_stand[i].count_up++;
				else
					teack_stand[i].count_up--;
			}
			
			if (stuTrack_judgeStand_ROI(mhi, teack_stand[i]))	//确定是否站立
			{	
				teack_stand[i].flag_Stand = 1;
			}
			else
			{
				if (teack_stand[i].count_up < 0)				//删除非站立roi
				{
					teack_stand[i] = teack_stand[--count_trackObj];
					i--;
				}
			}
		}
		else
		{
			//检测有没有坐下
			track_calculateDirect_ROI(mhi, teack_stand[i].roi, direct);
			if (direct == 2)
			{
				teack_stand[i].count_down++;
				if (teack_stand[i].count_down>2)
				{
					teack_stand[i] = teack_stand[--count_trackObj];
					i--;
				}
			}
		}
		teack_stand[i].flag_matching = 0;
	}
}

bool stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, Teack_Stand_t teack_stand)
{
	int ratio_lengthWidth = (teack_stand.roi.width > teack_stand.roi.height) ? (teack_stand.roi.width / teack_stand.roi.height) : (teack_stand.roi.height / teack_stand.roi.width);
	if ((teack_stand.count_up + teack_stand.count_teack) > 4 && ratio_lengthWidth<2)
	{
		int x1 = teack_stand.roi.x;
		int y1 = teack_stand.roi.y;
		int width =teack_stand.roi.width/3;
		int height = ITC_IMIN(teack_stand.roi.height, teack_stand.roi.width/2);

		int step = mhi->step;
		uchar *img = (uchar*)(mhi->data.ptr + step*(y1 + teack_stand.roi.height - height));

		int sum_value[3] = { 0,0,0 };
		int x2 = x1 + width;
		int i = 0, j = 0;
		for (i = 0; i < height; i++)
		{
			x1 = teack_stand.roi.x;
			x2 = x1 + width;
			for (j = x1; j < x2; j++)
			{
				sum_value[0] += (img[j]>0 ? 1 : 0);
			}
			x1 = x2;
			x2 += width;
			for (j = x1; j < x2; j++)
			{
				sum_value[1] += (img[j]>0 ? 1 : 0);
			}
			x1 = x2;
			x2 += width;
			for (j = x1; j < x2; j++)
			{
				sum_value[2] += (img[j]>0 ? 1 : 0);
			}
			img += step;
		}
		printf("比例：%d,%d,%d\n", sum_value[0], sum_value[1], sum_value[2]);
		if (sum_value[1]/2>sum_value[0] + sum_value[2])
			return true;
	}
	return false;
}


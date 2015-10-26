#include "stuTrack_track_img.h"

int stuTrack_filtrate_contours(ItcContour** pContour, int size_Threshold[], ItcRect *rect_arr)
{
	if (rect_arr == NULL || *pContour == NULL)
		return 0;

	int count_rect = 0;

	ItcContour *Contour = *pContour;
	do
	{
		ItcRect rect = Contour->rect;
		int centre_y = rect.y + rect.height;
		if (rect.width > size_Threshold[centre_y] &&
			rect.height > size_Threshold[centre_y] &&
			rect.height > rect.width)					//筛选
		{
			*(rect_arr + count_rect) = rect;
			count_rect++;
		}
		Contour = (ItcContour*)Contour->h_next;
	} while (Contour != *pContour);

	int i = 0, j = 0;
	if (count_rect < 100)
	{
		for (i = 0; i < count_rect; i++)
		{
			for (j = i + 1; j < count_rect; j++)
			{
				if (track_intersect_rect(rect_arr + i, rect_arr + j, -1))		//判断是否相交，如果相交则直接合并
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

bool stuTrack_matchingSatnd_ROI(Itc_Mat_t* mhi, ItcRect roi, StuTrack_Stand_t track_stand[], int &count_trackObj, int direct_threshold[], int direct_range)
{
	int standard_direct = direct_threshold[roi.x + roi.width / 2];
	int direct = 0;
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);

	int flag_ROI = track_calculateDirect_ROI(mhi, roi, direct);
	if (count_trackObj > 0)
	{
		int min_ID = 0;
		int min_distance = INT_MAX;
		int distance = 0;
		for (int i = 0; i < count_trackObj; i++)
		{
			distance = (x - track_stand[i].centre.x)*(x - track_stand[i].centre.x) + (y - track_stand[i].centre.y)*(y - track_stand[i].centre.y);
			if (min_distance>distance)
			{
				min_distance = distance;
				min_ID = i;
			}
		}

		int threshold = (track_stand[min_ID].roi.width * track_stand[min_ID].roi.height) >> 3;
		ItcRect _roi = roi;
		bool intersect_flag = track_intersect_rect(&_roi, &track_stand[min_ID].roi, -3);
		if (min_distance < threshold && (flag_ROI == 1))
		{
			if ((abs(track_stand[min_ID].direction - direct) < direct_range))
			{
				track_stand[min_ID].count_up++;
			}
			else
			{
				track_stand[min_ID].count_up--;
			}
			track_stand[min_ID].direction = (track_stand[min_ID].direction + direct) / 2;
			track_stand[min_ID].direction = ITC_IMAX(track_stand[min_ID].direction, standard_direct - direct_range / 2);
			track_stand[min_ID].direction = ITC_IMIN(track_stand[min_ID].direction, standard_direct + direct_range / 2);
			track_stand[min_ID].roi = _roi;
			track_stand[min_ID].count_teack++;
			track_stand[min_ID].centre = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
			track_stand[min_ID].flag_matching = 1;
			return true;
		}
		else if (intersect_flag)
		{
			//如果两个roi是相交的,但是距离不符合，说明新的roi包围住了teack_stand的roi,那就忽略这个
			track_stand[min_ID].count_up = 0;
		}
	}

	//add
	if ((abs(standard_direct - direct) < direct_range + 5) && (flag_ROI == 1))
	{
		direct = ITC_IMAX(direct, standard_direct - direct_range / 2);
		track_stand[count_trackObj].direction = ITC_IMIN(direct, standard_direct + direct_range / 2);
		track_stand[count_trackObj].centre = itcPoint(x, y);
		track_stand[count_trackObj].roi = roi;
		track_stand[count_trackObj].count_teack = 1;
		track_stand[count_trackObj].count_up = 1;
		track_stand[count_trackObj].count_down = 0;
		track_stand[count_trackObj].flag_Stand = 0;
		track_stand[count_trackObj].flag_matching = 1;
		count_trackObj++;
		return true;
	}
	return false;
}

void stuTrack_analyze_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t track_stand[], int &count_trackObj, int direct_threshold[], int direct_range)
{
	int i = 0;
	int direct = 0;
	int standard_direct = 0;
	int flag_ROI = 0;
	for (int i = 0; i < count_trackObj; i++)
	{
		if (track_stand[i].flag_Stand == 0)
		{
			if (!track_stand[i].flag_matching)
			{
				standard_direct = direct_threshold[track_stand[i].roi.x + track_stand[i].roi.width / 2];
				flag_ROI = track_calculateDirect_ROI(mhi, track_stand[i].roi, direct);
				if ((abs(track_stand[i].direction - direct) < direct_range) && (flag_ROI == 1))
				{
					track_stand[i].count_up++;
				}
				else
				{
					track_stand[i].count_up--;
				}
			}

			if (stuTrack_judgeStand_ROI(mhi, track_stand[i]))	//确定是否站立
			{
				printf("起立：%d,%d,%d,%d\n", track_stand[i].roi.x, track_stand[i].roi.y, track_stand[i].roi.width, track_stand[i].roi.height);
				track_stand[i].flag_Stand = 1;
			}
			else
			{
				if (track_stand[i].count_up - track_stand[i].count_teack < 0)				//删除非站立roi
				{
					track_stand[i] = track_stand[--count_trackObj];
					i--;
					continue;
				}
			}
		}
		else
		{
			//检测有没有坐下
			standard_direct = direct_threshold[track_stand[i].roi.x + track_stand[i].roi.width / 2] + 180;
			standard_direct = standard_direct > 360 ? standard_direct - 360 : standard_direct;
			flag_ROI = track_calculateDirect_ROI(mhi, track_stand[i].roi, direct);
			if ((abs(standard_direct - direct)< direct_range + 30) && (flag_ROI == 1))
			{
				track_stand[i].count_down++;
				if (track_stand[i].count_down>4)
				{
					printf("坐下：%d,%d\n", track_stand[i].roi.x, track_stand[i].roi.y);
					track_stand[i] = track_stand[--count_trackObj];
					i--;
					continue;
				}
			}
		}
		track_stand[i].flag_matching = 0;
	}
}

int stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t track_stand)
{
	double ratio_lengthWidth = (track_stand.roi.width > track_stand.roi.height) ? (((double)track_stand.roi.width) / track_stand.roi.height) : (((double)track_stand.roi.height) / track_stand.roi.width);
	if (((track_stand.count_up > 5 && track_stand.count_teack > 3) || track_stand.count_up > 20)
		&& ratio_lengthWidth <= 2.1)
	{
		//int x1 = teack_stand.roi.x;
		//int y1 = teack_stand.roi.y;
		//int width = teack_stand.roi.width / 3;
		//int height = ITC_IMIN(teack_stand.roi.height, teack_stand.roi.width / 2);
		////if (x1 < mhi->cols / 3)
		////{
		////	x1 = ITC_IMAX(x1 - width, 0);
		////}
		////else if (x1 + teack_stand.roi.width > mhi->cols - mhi->cols / 3)
		////{
		////	x1 = ITC_IMIN(x1 + width, mhi->cols - teack_stand.roi.width - 1);
		////}
		//int step = mhi->step;
		//uchar *img = (uchar*)(mhi->data.ptr + step*(y1 + teack_stand.roi.height - height));
		//int sum_value[3] = { 0, 0, 0 };
		//int x2 = x1 + width;
		//int i = 0, j = 0;
		//for (i = 0; i < height; i++)
		//{
		//	x2 = x1 + width;
		//	for (j = x1; j < x2; j++)
		//	{
		//		sum_value[0] += (img[j]>0 ? 1 : 0);
		//	}
		//	x2 += width;
		//	for (; j < x2; j++)
		//	{
		//		sum_value[1] += (img[j]>0 ? 1 : 0);
		//	}
		//	x2 += width;
		//	for (; j < x2; j++)
		//	{
		//		sum_value[2] += (img[j]>0 ? 1 : 0);
		//	}
		//	img += step;
		//}
		//if (sum_value[1] > sum_value[0] + sum_value[2])
		//{
		//	printf("起立，比例：%d,%d,%d\n", sum_value[0], sum_value[1], sum_value[2]);
		return 1;
		//}
		//printf("未起立，比例：%d,%d,%d\n", sum_value[0], sum_value[1], sum_value[2]);
	}
	//else if (teack_stand.startVertex_Y - teack_stand.roi.y>teack_stand.max_height/4)
	//{
	//	printf("站立2");
	//	return 2;
	//}
	return 0;
}

void stuTrack_proStandDown_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t track_stand[], int &count_trackObj, ItcRect rect_arr[], int &count_rect, int direct_threshold[], int direct_range)
{
	int i = 0;
	for (i = 0; i < count_rect; i++)
	{
		if (stuTrack_matchingSatnd_ROI(mhi, rect_arr[i], track_stand, count_trackObj, direct_threshold, direct_range))
		{
			rect_arr[i] = rect_arr[--count_rect];
			i--;
		}
	}
	stuTrack_analyze_ROI(mhi, track_stand, count_trackObj, direct_threshold, direct_range);			//分析预选的起立roi
}

size_t img_size = 0;
int count_trackObj_stand = 0;
StuTrack_Stand_t stuTrack_stand[100];

int count_trackObj_bigMove = 0;
StuTrack_BigMoveObj_t stuTrack_bigMOveObj[100];

ItcMemStorage* stuTrack_storage = NULL;

int stuTrack_rect_count;
ItcRect stuTrack_rect_arr[100];

Itc_Mat_t *tempMat = NULL;
Itc_Mat_t *currMat = NULL;
Itc_Mat_t *lastMat = NULL;
Itc_Mat_t *mhiMat = NULL;
Itc_Mat_t *maskMat = NULL;

//需要设置的参数
int stuTrack_size_threshold[270];		//运动目标大小过滤阈值
int stuTrack_direct_range;
int stuTrack_direct_threshold[480];

void stuTrack_initializeTrack(int height, int width)
{
	//分配内存
	currMat = itc_create_mat(height, width, ITC_8UC1);
	lastMat = itc_create_mat(height, width, ITC_8UC1);
	mhiMat = itc_create_mat(height, width, ITC_8UC1);
	maskMat = itc_create_mat(height, width, ITC_8UC1);
	stuTrack_storage = itcCreateMemStorage(0);

	//初始化参数
	img_size = height*width;

	for (int i = 0; i < 270; i++)
	{
		stuTrack_size_threshold[i] = 18 + (i / 10);
	}
	stuTrack_direct_range = 10;
	for (int i = 0; i < 240; i++)
	{
		int direct = (480 - i * 2) >> 5;
		stuTrack_direct_threshold[i] = 270 - direct;
		stuTrack_direct_threshold[479 - i] = 270 + direct;
	}
}

void stuTrack_main(char* imageData)
{
	static int _count = 0;//统计帧数
	ItcContour* firstContour = NULL;
	memcpy(currMat->data.ptr, imageData, img_size);
	if (_count>1)
	{
		itcClearMemStorage(stuTrack_storage);
		track_update_MHI(currMat, lastMat, mhiMat, 15, maskMat, 248);
		track_find_contours(maskMat, &firstContour, stuTrack_storage);
		stuTrack_rect_count = stuTrack_filtrate_contours(&firstContour, stuTrack_size_threshold, stuTrack_rect_arr);
		stuTrack_proStandDown_ROI(mhiMat, stuTrack_stand, count_trackObj_stand, stuTrack_rect_arr, stuTrack_rect_count, stuTrack_direct_threshold, stuTrack_direct_range);
	}
	tempMat = currMat;
	currMat = lastMat;
	lastMat = tempMat;
	_count++;
}

void stuTrack_stopTrack()
{
	itcReleaseMemStorage(&stuTrack_storage);

	itc_release_mat(&currMat);
	itc_release_mat(&lastMat);
	itc_release_mat(&mhiMat);
	itc_release_mat(&maskMat);
}

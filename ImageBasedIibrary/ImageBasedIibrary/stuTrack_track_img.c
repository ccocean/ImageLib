#include "stuTrack_track_img.h"

size_t img_size = 0;
int count_trackObj_stand = 0;
StuTrack_Stand_t* stuTrack_stand=NULL;

int count_trackObj_bigMove = 0;
StuTrack_BigMoveObj_t* stuTrack_bigMOveObj=NULL;

int count_stuTrack_rect = 0;
Track_Rect_t *stuTrack_rect_arr = NULL;

Track_MemStorage_t* stuTrack_storage = NULL;

Itc_Mat_t *tempMat = NULL;
Itc_Mat_t *currMat = NULL;
Itc_Mat_t *lastMat = NULL;
Itc_Mat_t *mhiMat = NULL;
Itc_Mat_t *maskMat = NULL;

//需要设置的参数
int *stuTrack_size_threshold=NULL;		//运动目标大小过滤阈值
int stuTrack_direct_range = 10;
int *stuTrack_direct_threshold=NULL;

int stuTrack_filtrate_contours(Track_Contour_t** pContour)
{
	if (*pContour == NULL)
		return 0;

	int count_rect = 0;

	int stuTrack_filtrate_contours(Track_Contour_t** pContour);
	Track_Contour_t	*Contour = *pContour;
	do
	{
		Track_Rect_t rect = Contour->rect;
		int centre_y = rect.y + rect.height;
		if (rect.width > stuTrack_size_threshold[centre_y] &&
			rect.height > rect.width &&
			count_rect < MALLOC_ELEMENT_COUNT)					//筛选
		{
			*(stuTrack_rect_arr + count_rect) = rect;
			count_rect++;
		}
		Contour = (Track_Contour_t*)Contour->h_next;
	} while (Contour != *pContour);

	int i = 0, j = 0;
	if (count_rect < 100)
	{
		for (i = 0; i < count_rect; i++)
		{
			for (j = i + 1; j < count_rect; j++)
			{
				if (track_intersect_rect(stuTrack_rect_arr + i, stuTrack_rect_arr + j, -1))		//判断是否相交，如果相交则直接合并
				{
					count_rect--;
					*(stuTrack_rect_arr + j) = *(stuTrack_rect_arr + count_rect);
					j--;
				}
			}
		}
	}

	return count_rect;
}

int stuTrack_matchingSatnd_ROI(Itc_Mat_t* mhi, Track_Rect_t roi)
{
	int standard_direct = stuTrack_direct_threshold[roi.x + (roi.width >> 1)];
	int direct = 0;
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);

	int flag_ROI = track_calculateDirect_ROI(mhi, roi, &direct);
	if (flag_ROI == 1)
	{
		if (count_trackObj_stand > 0)
		{
			int min_ID = 0;
			int min_distance = INT_MAX;
			int distance = 0;
			for (int i = 0; i < count_trackObj_stand; i++)
			{
				distance = (x - stuTrack_stand[i].centre.x)*(x - stuTrack_stand[i].centre.x) + (y - stuTrack_stand[i].centre.y)*(y - stuTrack_stand[i].centre.y);
				if (min_distance>distance)
				{
					min_distance = distance;
					min_ID = i;
				}
			}

			int threshold = (stuTrack_stand[min_ID].roi.width * stuTrack_stand[min_ID].roi.height) >> 3;
			Track_Rect_t _roi = roi;
			if (min_distance < threshold)
			{
				track_intersect_rect(&_roi, &stuTrack_stand[min_ID].roi, -3);
				if ((abs(stuTrack_stand[min_ID].direction - direct) < stuTrack_direct_range))
				{
					stuTrack_stand[min_ID].count_up++;
				}
				else
				{
					stuTrack_stand[min_ID].count_up--;
				}
				stuTrack_stand[min_ID].count_teack++;
				stuTrack_stand[min_ID].flag_matching = 1;
				stuTrack_stand[min_ID].direction = (stuTrack_stand[min_ID].direction + direct) >> 1;
				stuTrack_stand[min_ID].direction = ITC_IMAX(stuTrack_stand[min_ID].direction, standard_direct - (stuTrack_direct_range >> 1));
				stuTrack_stand[min_ID].direction = ITC_IMIN(stuTrack_stand[min_ID].direction, standard_direct + (stuTrack_direct_range >> 1));
				stuTrack_stand[min_ID].roi = _roi;
				stuTrack_stand[min_ID].centre = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
				stuTrack_stand[min_ID].current_tClock = clock();
				return 1;
			}
			//else if (intersect_flag)
			//{
			//	//如果两个roi是相交的,但是距离不符合，说明新的roi包围住了teack_stand的roi,那就忽略这个
			//	stuTrack_stand[min_ID].count_up = 0;
			//}
		}

		if (abs(standard_direct - direct) < (stuTrack_direct_range + 5) && count_trackObj_stand < MALLOC_ELEMENT_COUNT)
		{
			//add
			direct = ITC_IMAX(direct, standard_direct - (stuTrack_direct_range >> 1));
			stuTrack_stand[count_trackObj_stand].direction = ITC_IMIN(direct, standard_direct + (stuTrack_direct_range >> 1));
			stuTrack_stand[count_trackObj_stand].count_teack = 1;
			stuTrack_stand[count_trackObj_stand].count_up = 1;
			stuTrack_stand[count_trackObj_stand].count_down = 0;
			stuTrack_stand[count_trackObj_stand].flag_Stand = 0;
			stuTrack_stand[count_trackObj_stand].flag_matching = 1;
			stuTrack_stand[count_trackObj_stand].centre = itcPoint(x, y);
			stuTrack_stand[count_trackObj_stand].roi = roi;
			stuTrack_stand[count_trackObj_stand].start_tClock = stuTrack_stand[count_trackObj_stand].current_tClock = clock();
			count_trackObj_stand++;
			return 1;
		}
	}

	int intersect_flag = 0;
	for (int i = 0; i < count_trackObj_stand; i++)
	{
		intersect_flag = track_intersect_rect(&roi, &stuTrack_stand[i].roi, -(roi.width >> 1));
	}
	if (intersect_flag == 0)
	{
		if (count_trackObj_bigMove > 0)
		{
			int k = -1;
			Track_Rect_t _roi = roi;
			for (int i = 0; i < count_trackObj_bigMove; i++)
			{
				if (track_intersect_rect(&_roi, &stuTrack_bigMOveObj[i].roi, -(_roi.width >> 1)))
				{
					k = i;
					break;
				}
			}
			if (k >= 0)
			{
				stuTrack_bigMOveObj[k].count_track++;
				stuTrack_bigMOveObj[k].roi = roi;
				stuTrack_bigMOveObj[k].current_position = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
				stuTrack_bigMOveObj[k].current_tClock = clock();
				return 2;
			}
		}
		int centre_y = roi.y + roi.width;
		int size_threshold1 = stuTrack_size_threshold[centre_y] + (stuTrack_size_threshold[centre_y] >> 2);
		int size_threshold2 = stuTrack_size_threshold[centre_y] + (stuTrack_size_threshold[centre_y] >> 1);
		if ((roi.width >  size_threshold1 && roi.height > size_threshold2)
			&& count_trackObj_bigMove < MALLOC_ELEMENT_COUNT)
		{
			stuTrack_bigMOveObj[count_trackObj_bigMove].count_track = 1;
			stuTrack_bigMOveObj[count_trackObj_bigMove].flag_bigMove = 0;
			stuTrack_bigMOveObj[count_trackObj_bigMove].roi = roi;
			stuTrack_bigMOveObj[count_trackObj_bigMove].origin_position = stuTrack_bigMOveObj[count_trackObj_bigMove].current_position = itcPoint(x, y);
			stuTrack_bigMOveObj[count_trackObj_bigMove].start_tClock = stuTrack_bigMOveObj[count_trackObj_bigMove].current_tClock = clock();	
			count_trackObj_bigMove++;
			return 2;
		}
	}
	return 0;
}

void stuTrack_analyze_ROI(Itc_Mat_t* mhi)
{
	int i = 0;
	int direct = 0;
	int standard_direct = 0;
	int flag_ROI = 0;
	for (int i = 0; i < count_trackObj_stand; i++)
	{
		if (stuTrack_stand[i].flag_Stand != 1)
		{
			if (!stuTrack_stand[i].flag_matching)
			{
				standard_direct = stuTrack_direct_threshold[stuTrack_stand[i].roi.x + (stuTrack_stand[i].roi.width >> 1)];
				flag_ROI = track_calculateDirect_ROI(mhi, stuTrack_stand[i].roi, &direct);
				if ((abs(stuTrack_stand[i].direction - direct) < stuTrack_direct_range) && (flag_ROI == 1))
				{
					stuTrack_stand[i].count_up++;
				}
			}

			if (stuTrack_judgeStand_ROI(mhi, stuTrack_stand[i]))	//确定是否站立
			{
				printf("起立：%d,%d,%d,%d\n", stuTrack_stand[i].roi.x, stuTrack_stand[i].roi.y, stuTrack_stand[i].roi.width, stuTrack_stand[i].roi.height);
				stuTrack_stand[i].flag_Stand = 1;
			}
		}
		else
		{
			//检测有没有坐下
			standard_direct = stuTrack_direct_threshold[stuTrack_stand[i].roi.x + (stuTrack_stand[i].roi.width >> 1)] + 180;
			standard_direct = standard_direct > 360 ? standard_direct - 360 : standard_direct;
			flag_ROI = track_calculateDirect_ROI(mhi, stuTrack_stand[i].roi, &direct);
			if ((abs(standard_direct - direct)< stuTrack_direct_range + 30) && (flag_ROI == 1))
			{
				stuTrack_stand[i].count_down++;
				if (stuTrack_stand[i].count_down>5)
				{
					printf("坐下：%d,%d\n", stuTrack_stand[i].roi.x, stuTrack_stand[i].roi.y);
					stuTrack_stand[i].flag_Stand = -1;
					stuTrack_stand[i].count_up = 0;
					continue;
				}
			}
		}
		
		if (stuTrack_stand[i].flag_Stand != 1)
		{
			clock_t _time = clock() - stuTrack_stand[i].current_tClock;
			if ( _time > 300)				//删除非站立roi
			{
				stuTrack_stand[i] = stuTrack_stand[--count_trackObj_stand];
				i--;
				continue;
			}
		}
		stuTrack_stand[i].flag_matching = 0;
	}

	//分析移动的目标
	for (i = 0; i < count_trackObj_bigMove; i++)
	{
		clock_t _time = clock() - stuTrack_bigMOveObj[i].current_tClock;
		if (_time > 1000)
		{
			stuTrack_bigMOveObj[i] = stuTrack_bigMOveObj[--count_trackObj_bigMove];
			i--;
			continue;
		}
		if (stuTrack_bigMOveObj[i].flag_bigMove == 0)
		{
			_time = stuTrack_bigMOveObj[i].current_tClock - stuTrack_bigMOveObj[i].start_tClock;
			standard_direct = stuTrack_direct_threshold[stuTrack_bigMOveObj[i].roi.x + (stuTrack_bigMOveObj[i].roi.width >> 1)];
			int distance = (stuTrack_bigMOveObj[i].origin_position.x - stuTrack_bigMOveObj[i].current_position.x)*(stuTrack_bigMOveObj[i].origin_position.x - stuTrack_bigMOveObj[i].current_position.x)
				+ (stuTrack_bigMOveObj[i].origin_position.y - stuTrack_bigMOveObj[i].current_position.y)*(stuTrack_bigMOveObj[i].origin_position.y - stuTrack_bigMOveObj[i].current_position.y);
			int dis_T = (stuTrack_bigMOveObj[i].roi.width*stuTrack_bigMOveObj[i].roi.width);
			//printf("判断移动目标：%d,%d\n", distance, dis_T);
			if ((distance>dis_T && _time>500))
			{
				printf("发现移动目标：%d,%d\n", stuTrack_bigMOveObj[i].roi.x, stuTrack_bigMOveObj[i].roi.y);
				stuTrack_bigMOveObj[i].flag_bigMove = 1;
			}	
		}
	}
}

int stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t track_stand)
{
	double ratio_lengthWidth = (((double)track_stand.roi.height) / track_stand.roi.width);
	if (((track_stand.count_up > 5 && track_stand.count_teack > 3) || track_stand.count_up > 15)
		&& ratio_lengthWidth <= 2.1)
	{
		return 1;
	}
	return 0;
}

void stuTrack_proStandDown_ROI(Itc_Mat_t* mhi)
{
	int i = 0;
	for (i = 0; i < count_stuTrack_rect; i++)
	{
		if (stuTrack_matchingSatnd_ROI(mhi, stuTrack_rect_arr[i]))
		{
			stuTrack_rect_arr[i] = stuTrack_rect_arr[--count_stuTrack_rect];
			i--;
		}
	}
	stuTrack_analyze_ROI(mhi);			//分析候选roi
}

void stuTrack_initializeTrack(int height, int width)
{
	//分配内存
	currMat = itc_create_mat(height, width, ITC_8UC1);
	lastMat = itc_create_mat(height, width, ITC_8UC1);
	mhiMat = itc_create_mat(height, width, ITC_8UC1);
	maskMat = itc_create_mat(height, width, ITC_8UC1);
	stuTrack_storage = itcCreateMemStorage(0);

	stuTrack_stand = (StuTrack_Stand_t*)malloc(sizeof(StuTrack_Stand_t)* MALLOC_ELEMENT_COUNT);
	stuTrack_bigMOveObj = (StuTrack_BigMoveObj_t*)malloc(sizeof(StuTrack_BigMoveObj_t)* MALLOC_ELEMENT_COUNT);
	stuTrack_rect_arr = (Track_Rect_t*)malloc(sizeof(Track_Rect_t)* MALLOC_ELEMENT_COUNT);

	memset(stuTrack_stand, 0, sizeof(StuTrack_Stand_t)* MALLOC_ELEMENT_COUNT);
	memset(stuTrack_bigMOveObj, 0, sizeof(StuTrack_BigMoveObj_t)* MALLOC_ELEMENT_COUNT);
	memset(stuTrack_rect_arr, 0, sizeof(Track_Rect_t)* MALLOC_ELEMENT_COUNT);

	//初始化参数
	img_size = height*width;

	stuTrack_size_threshold = (int *)malloc(sizeof(int)* STUTRACK_IMG_HEIGHT);
	for (int i = 0; i < STUTRACK_IMG_HEIGHT; i++)
	{
		stuTrack_size_threshold[i] = ITC_IMIN(ITC_IMAX((-3 + (i >> 2)), 15), 55);
	}

	stuTrack_direct_range = 10;
	stuTrack_direct_threshold = (int *)malloc(sizeof(int)* STUTRACK_IMG_WIDTH);
	for (int i = 0; i < STUTRACK_IMG_WIDTH/2; i++)
	{
		int direct = (480 - i * 2) >> 5;
		stuTrack_direct_threshold[i] = 270 - direct;
		stuTrack_direct_threshold[479 - i] = 270 + direct;
	}
}

void stuTrack_main(char* imageData)
{
	static int _count = 0;//统计帧数
	Track_Contour_t* firstContour = NULL;
	memcpy(currMat->data.ptr, imageData, img_size);
	if (_count>1)
	{
		itcClearMemStorage(stuTrack_storage);
		track_update_MHI(currMat, lastMat, mhiMat, 15, maskMat, 248);
		track_find_contours(maskMat, &firstContour, stuTrack_storage);
		count_stuTrack_rect = stuTrack_filtrate_contours(&firstContour);
		stuTrack_proStandDown_ROI(mhiMat);
	}
	tempMat = currMat;
	currMat = lastMat;
	lastMat = tempMat;
	_count++;
}

void stuTrack_stopTrack()
{
	if (stuTrack_stand != NULL)
	{
		free(stuTrack_stand);
		stuTrack_stand = NULL;
	}

	if (stuTrack_bigMOveObj != NULL)
	{
		free(stuTrack_bigMOveObj);
		stuTrack_bigMOveObj = NULL;
	}

	if (stuTrack_rect_arr != NULL)
	{
		free(stuTrack_rect_arr);
		stuTrack_rect_arr = NULL;
	}

	if (stuTrack_size_threshold != NULL)
	{
		free(stuTrack_size_threshold);
		stuTrack_size_threshold = NULL;
	}

	if (stuTrack_direct_threshold != NULL)
	{
		free(stuTrack_direct_threshold);
		stuTrack_direct_threshold = NULL;
	}

	itcReleaseMemStorage(&stuTrack_storage);

	itc_release_mat(&currMat);
	itc_release_mat(&lastMat);
	itc_release_mat(&mhiMat);
	itc_release_mat(&maskMat);
}

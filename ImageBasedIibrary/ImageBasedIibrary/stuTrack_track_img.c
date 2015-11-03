#include "stuTrack_track_img.h"

int stuTrack_filtrate_contours(StuITRACK_Params *inst, Track_Contour_t** pContour)
{
	if (*pContour == NULL)
		return 0;

	int count_rect = 0;
	int *stuTrack_size_threshold = inst->stuTrack_size_threshold;
	StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;

	Track_Rect_t *stuTrack_rect_arr = interior_params_p->stuTrack_rect_arr;
	Track_Contour_t	*Contour = *pContour;
	do
	{
		Track_Rect_t rect = Contour->rect;
		int centre_y = rect.y + rect.height;
		if (rect.width > stuTrack_size_threshold[centre_y] &&
			rect.height > stuTrack_size_threshold[centre_y] &&
			count_rect < MALLOC_ELEMENT_COUNT)					//筛选
		{
			*(stuTrack_rect_arr + count_rect) = rect;
			count_rect++;
		}
		Contour = (Track_Contour_t*)Contour->h_next;
	} while (Contour != *pContour);

	int i = 0, j = 0;
	for (i = 0; i < count_rect; i++)
	{
		for (j = i + 1; j < count_rect; j++)
		{
			if (track_intersect_rect(stuTrack_rect_arr + i, stuTrack_rect_arr + j, -1))	//判断是否相交，如果相交则直接合并
			{
				count_rect--;
				*(stuTrack_rect_arr + j) = *(stuTrack_rect_arr + count_rect);
				j--;
			}
		}
	}

	return count_rect;
}

int stuTrack_matchingSatnd_ROI(StuITRACK_Params *inst, Track_Rect_t roi)
{
	//
	int *stuTrack_size_threshold = inst->stuTrack_size_threshold;
	int stuTrack_direct_range = inst->stuTrack_direct_range;
	int *stuTrack_direct_threshold = inst->stuTrack_direct_threshold;
	float stuTrack_move_threshold = inst->stuTrack_move_threshold;

	StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;

	int standard_direct = stuTrack_direct_threshold[roi.x + (roi.width >> 1)];
	int direct = 0;
	//中心点
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);
	int i = 0;
	int flag_ROI = track_calculateDirect_ROI((Itc_Mat_t *)interior_params_p->mhiMat, roi, &direct);
	if (flag_ROI == 1 && roi.height>roi.width)
	{
		if (interior_params_p->count_trackObj_stand > 0)
		{
			int min_ID = 0;
			int min_distance = INT_MAX;
			int distance = 0;
			for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
			{
				distance = (x - interior_params_p->stuTrack_stand[i].centre.x)*(x - interior_params_p->stuTrack_stand[i].centre.x) + (y - interior_params_p->stuTrack_stand[i].centre.y)*(y - interior_params_p->stuTrack_stand[i].centre.y);
				if (min_distance>distance)
				{
					min_distance = distance;
					min_ID = i;
				}
			}

			int threshold = (interior_params_p->stuTrack_stand[min_ID].roi.width * interior_params_p->stuTrack_stand[min_ID].roi.height) >> 3;
			Track_Rect_t _roi = roi;
			if (min_distance < threshold)
			{
				track_intersect_rect(&_roi, &(interior_params_p->stuTrack_stand[min_ID].roi), -3);
				if ((abs(interior_params_p->stuTrack_stand[min_ID].direction - direct) < stuTrack_direct_range))
				{
					interior_params_p->stuTrack_stand[min_ID].count_up++;
				}
				else
				{
					interior_params_p->stuTrack_stand[min_ID].count_up--;
				}
				interior_params_p->stuTrack_stand[min_ID].count_teack++;
				interior_params_p->stuTrack_stand[min_ID].flag_matching = 1;
				interior_params_p->stuTrack_stand[min_ID].direction = (interior_params_p->stuTrack_stand[min_ID].direction + direct) >> 1;
				interior_params_p->stuTrack_stand[min_ID].direction = ITC_IMAX(interior_params_p->stuTrack_stand[min_ID].direction, standard_direct - (stuTrack_direct_range >> 1));
				interior_params_p->stuTrack_stand[min_ID].direction = ITC_IMIN(interior_params_p->stuTrack_stand[min_ID].direction, standard_direct + (stuTrack_direct_range >> 1));
				interior_params_p->stuTrack_stand[min_ID].roi = _roi;
				interior_params_p->stuTrack_stand[min_ID].centre = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
				interior_params_p->stuTrack_stand[min_ID].current_tClock = clock();
				return 1;
			}
		}

		if (abs(standard_direct - direct) < (stuTrack_direct_range + 5) && interior_params_p->count_trackObj_stand < MALLOC_ELEMENT_COUNT)
		{
			//add
			direct = ITC_IMAX(direct, standard_direct - (stuTrack_direct_range >> 1));
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].direction = ITC_IMIN(direct, standard_direct + (stuTrack_direct_range >> 1));
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].count_teack = 1;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].count_up = 1;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].count_down = 0;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].flag_Stand = 0;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].flag_matching = 1;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].centre = itcPoint(x, y);
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].roi = roi;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].start_tClock = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].current_tClock = clock();
			interior_params_p->count_trackObj_stand++;
			return 1;
		}
	}

	int intersect_flag = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		intersect_flag = track_intersect_rect(&roi, &interior_params_p->stuTrack_stand[i].roi, -(roi.width >> 1));
	}
	if (intersect_flag == 0)
	{
		int centre_y = roi.y + (roi.height >> 1);
		int size_threshold1 = stuTrack_size_threshold[centre_y] + (stuTrack_size_threshold[centre_y] >> 2);
		int size_threshold2 = stuTrack_size_threshold[centre_y] + (stuTrack_size_threshold[centre_y] >> 1);
		if ((roi.width >  size_threshold1 && roi.height > size_threshold2))
		{
			if (interior_params_p->count_trackObj_bigMove > 0)
			{
				int k = -1;
				Track_Rect_t _roi = roi;
				for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
				{
					if (track_intersect_rect(&_roi, &interior_params_p->stuTrack_bigMOveObj[i].roi, -(_roi.width >> 1)))
					{
						k = i;
						break;
					}
				}
				if (k >= 0)
				{
					interior_params_p->stuTrack_bigMOveObj[k].count_track++;
					interior_params_p->stuTrack_bigMOveObj[k].roi = roi;
					interior_params_p->stuTrack_bigMOveObj[k].current_position = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
					interior_params_p->stuTrack_bigMOveObj[k].current_tClock = clock();
					return 2;
				}
			}
			if (interior_params_p->count_trackObj_bigMove < MALLOC_ELEMENT_COUNT)
			{
				printf("add bigMove：origin:%d,%d\n", x, y);
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].count_track = 1;
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].flag_bigMove = 0;
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].dis_threshold = ITC_IMIN(roi.width, roi.height) * stuTrack_move_threshold;
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].roi = roi;
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].origin_position = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].current_position = itcPoint(x, y);
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].start_tClock = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].current_tClock = clock();
				interior_params_p->count_trackObj_bigMove++;
				return 2;
			}
		}
	}
	return 0;
}

void stuTrack_analyze_ROI(StuITRACK_Params *inst)
{
	int *stuTrack_size_threshold = inst->stuTrack_size_threshold;
	int stuTrack_direct_range = inst->stuTrack_direct_range;
	int *stuTrack_direct_threshold = inst->stuTrack_direct_threshold;
	int stuTrack_sitdownCount_threshold = inst->stuTrack_sitdownCount_threshold;
	int sturTrack_moveDelayed_threshold = inst->sturTrack_moveDelayed_threshold;

	StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;
	Itc_Mat_t *mhi = (Itc_Mat_t *)interior_params_p->mhiMat;

	int i = 0;
	int direct = 0;
	int standard_direct = 0;
	int flag_ROI = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (interior_params_p->stuTrack_stand[i].flag_Stand != 1)
		{
			//检测有没有起立
			if (!interior_params_p->stuTrack_stand[i].flag_matching)
			{
				standard_direct = stuTrack_direct_threshold[interior_params_p->stuTrack_stand[i].centre.x];
				flag_ROI = track_calculateDirect_ROI(mhi, interior_params_p->stuTrack_stand[i].roi, &direct);
				if ((abs(interior_params_p->stuTrack_stand[i].direction - direct) < stuTrack_direct_range) && (flag_ROI == 1))
				{
					interior_params_p->stuTrack_stand[i].count_up++;
				}
			}

			if (stuTrack_judgeStand_ROI(inst, interior_params_p->stuTrack_stand[i]))	//确定是否站立
			{
				//printf("stand up：%d,%d,%d,%d\n", stuTrack_stand[i].centre.x, stuTrack_stand[i].centre.y, stuTrack_stand[i].roi.width, stuTrack_stand[i].roi.height);
				interior_params_p->stuTrack_stand[i].flag_Stand = 1;
			}
		}
		else
		{
			//检测有没有坐下
			standard_direct = stuTrack_direct_threshold[interior_params_p->stuTrack_stand[i].centre.x];
			standard_direct = standard_direct > 180 ? standard_direct - 180 : standard_direct + 180;		//计算与起立方向相反的角度
			flag_ROI = track_calculateDirect_ROI(mhi, interior_params_p->stuTrack_stand[i].roi, &direct);
			if ((abs(standard_direct - direct)< stuTrack_direct_range + 30) && (flag_ROI == 1))
			{
				interior_params_p->stuTrack_stand[i].count_down++;
				if (interior_params_p->stuTrack_stand[i].count_down>stuTrack_sitdownCount_threshold)
				{
					//printf("sit down：%d,%d\n", stuTrack_stand[i].centre.x, stuTrack_stand[i].centre.y);
					interior_params_p->stuTrack_stand[i].flag_Stand = -1;
					interior_params_p->stuTrack_stand[i].count_up = 0;
					continue;
				}
			}
		}
		
		if (interior_params_p->stuTrack_stand[i].flag_Stand != 1)
		{
			clock_t _time = clock() - interior_params_p->stuTrack_stand[i].current_tClock;
			if ( _time > 300)				//删除非站立roi
			{
				interior_params_p->stuTrack_stand[i] = interior_params_p->stuTrack_stand[--(interior_params_p->count_trackObj_stand)];
				i--;
				continue;
			}
		}
		interior_params_p->stuTrack_stand[i].flag_matching = 0;
	}

	//分析移动的目标
	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{
		clock_t _time = clock() - interior_params_p->stuTrack_bigMOveObj[i].current_tClock;
		if (_time > sturTrack_moveDelayed_threshold)
		{
			printf("delete bigMove:origin:%d,%d,current:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].current_position.x, interior_params_p->stuTrack_bigMOveObj[i].current_position.y);
			interior_params_p->stuTrack_bigMOveObj[i] = interior_params_p->stuTrack_bigMOveObj[--(interior_params_p->count_trackObj_bigMove)];
			i--;
			continue;
		}
		if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == 0)
		{
			int diff_x = abs(interior_params_p->stuTrack_bigMOveObj[i].origin_position.x - interior_params_p->stuTrack_bigMOveObj[i].current_position.x);
			int diff_y = abs(interior_params_p->stuTrack_bigMOveObj[i].origin_position.y - interior_params_p->stuTrack_bigMOveObj[i].current_position.y);
			if (diff_x>interior_params_p->stuTrack_bigMOveObj[i].dis_threshold || diff_y>interior_params_p->stuTrack_bigMOveObj[i].dis_threshold)
			{
				_time = interior_params_p->stuTrack_bigMOveObj[i].current_tClock - interior_params_p->stuTrack_bigMOveObj[i].start_tClock;
				if (_time > 500)
				{
					printf("find Move：origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].roi.width, interior_params_p->stuTrack_bigMOveObj[i].roi.height);
					interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove = 1;
				}
			}
			else 
			{
				int centre_y = interior_params_p->stuTrack_bigMOveObj[i].roi.y + interior_params_p->stuTrack_bigMOveObj[i].roi.width;
				int size_threshold = stuTrack_size_threshold[centre_y] + stuTrack_size_threshold[centre_y];
				if ((interior_params_p->stuTrack_bigMOveObj[i].roi.width > size_threshold && interior_params_p->stuTrack_bigMOveObj[i].roi.height > size_threshold))
				{
					printf("find big：origin:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y);
					interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove = 2;
				}
			}
		}
	}
}

int stuTrack_judgeStand_ROI(StuITRACK_Params *inst, StuTrack_Stand_t track_stand)
{
	int stuTrack_standCount_threshold = inst->stuTrack_standCount_threshold;
	double ratio_lengthWidth = (((double)track_stand.roi.height) / track_stand.roi.width);
	if (((track_stand.count_up > stuTrack_standCount_threshold && track_stand.count_teack > stuTrack_standCount_threshold - 2) 
		|| track_stand.count_up > stuTrack_standCount_threshold + 10)
		&& ratio_lengthWidth <= 2.1)
	{
		return 1;
	}
	return 0;
}

void stuTrack_proStandDown_ROI(StuITRACK_Params *inst)
{
	StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;
	int i = 0;
	for (i = 0; i < interior_params_p->count_stuTrack_rect; i++)
	{
		if (stuTrack_matchingSatnd_ROI(inst, interior_params_p->stuTrack_rect_arr[i]))
		{
			interior_params_p->stuTrack_rect_arr[i] = interior_params_p->stuTrack_rect_arr[--(interior_params_p->count_stuTrack_rect)];
			i--;
		}
	}
	stuTrack_analyze_ROI(inst);			//分析候选roi
}

void stuTrack_initializeTrack(StuITRACK_Params *inst)
{
	stuTrack_stopTrack(inst);
	//分配内存
	inst->interior_params = malloc(sizeof(StuITRACK_InteriorParams));//分配全局变量內存
	memset(inst->interior_params, 0, sizeof(StuITRACK_InteriorParams));

	StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;
	interior_params_p->currMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	interior_params_p->lastMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	interior_params_p->mhiMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	interior_params_p->maskMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	interior_params_p->stuTrack_storage = itcCreateMemStorage(0);
	interior_params_p->stuTrack_stand = (StuTrack_Stand_t*)malloc(sizeof(StuTrack_Stand_t)* MALLOC_ELEMENT_COUNT);
	interior_params_p->stuTrack_bigMOveObj = (StuTrack_BigMoveObj_t*)malloc(sizeof(StuTrack_BigMoveObj_t)* MALLOC_ELEMENT_COUNT);
	interior_params_p->stuTrack_rect_arr = (Track_Rect_t*)malloc(sizeof(Track_Rect_t)* MALLOC_ELEMENT_COUNT);

	//初始化自有的内部参数
	interior_params_p->_count = 0;
	interior_params_p->img_size = inst->height*inst->width;
	memset(interior_params_p->stuTrack_stand, 0, sizeof(StuTrack_Stand_t)* MALLOC_ELEMENT_COUNT);
	memset(interior_params_p->stuTrack_bigMOveObj, 0, sizeof(StuTrack_BigMoveObj_t)* MALLOC_ELEMENT_COUNT);
	memset(interior_params_p->stuTrack_rect_arr, 0, sizeof(Track_Rect_t)* MALLOC_ELEMENT_COUNT);
	interior_params_p->count_trackObj_stand = 0;
	interior_params_p->count_trackObj_bigMove = 0;
	interior_params_p->count_stuTrack_rect = 0;

	int i = 0;
	if (inst->flag_setting == FALSE)
	{
		//默认的参数
		inst->stuTrack_move_threshold = 1.2;			//判定是移动目标的偏离阈值（比值）
		inst->stuTrack_standCount_threshold = 5;		//判定为起立的帧数阈值
		inst->stuTrack_sitdownCount_threshold = 5;		//判定为坐下的帧数阈值
		inst->sturTrack_moveDelayed_threshold = 1000;

		inst->stuTrack_size_threshold = (int *)malloc(sizeof(int)* inst->height);
		for (i = 0; i < inst->height; i++)
		{
			inst->stuTrack_size_threshold[i] = ITC_IMIN(ITC_IMAX(((-7 + (i >> 2))), 21), 78);
		}

		inst->stuTrack_direct_range = 10;
		inst->stuTrack_direct_threshold = (int *)malloc(sizeof(int)* inst->width);
		for (i = 0; i < inst->width / 2; i++)
		{
			int direct = (inst->width - i * 2) >> 5;
			inst->stuTrack_direct_threshold[i] = 270 - direct;
			inst->stuTrack_direct_threshold[479 - i] = 270 + direct;
		}
	}
}

void stuTrack_process(StuITRACK_Params *inst, char* imageData)
{
	StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;
	Track_Contour_t* firstContour = NULL;
	memcpy(interior_params_p->currMat->data.ptr, imageData, interior_params_p->img_size);
	if (interior_params_p->_count>1)
	{
		itcClearMemStorage(interior_params_p->stuTrack_storage);
		track_update_MHI(interior_params_p->currMat, interior_params_p->lastMat, interior_params_p->mhiMat, 15, interior_params_p->maskMat, 248);
		track_find_contours(interior_params_p->maskMat, &firstContour, interior_params_p->stuTrack_storage);
		interior_params_p->count_stuTrack_rect = stuTrack_filtrate_contours(inst, &firstContour);
		stuTrack_proStandDown_ROI(inst);
	}
	interior_params_p->tempMat = interior_params_p->currMat;
	interior_params_p->currMat = interior_params_p->lastMat;
	interior_params_p->lastMat = interior_params_p->tempMat;
	interior_params_p->_count++;
}

void stuTrack_stopTrack(StuITRACK_Params *inst)
{
	if (inst->interior_params != NULL)
	{
		StuITRACK_InteriorParams *interior_params_p = (StuITRACK_InteriorParams *)inst->interior_params;
		if (interior_params_p->stuTrack_stand != NULL)
		{
			free(interior_params_p->stuTrack_stand);
			interior_params_p->stuTrack_stand = NULL;
		}

		if (interior_params_p->stuTrack_bigMOveObj != NULL)
		{
			free(interior_params_p->stuTrack_bigMOveObj);
			interior_params_p->stuTrack_bigMOveObj = NULL;
		}

		if (interior_params_p->stuTrack_rect_arr != NULL)
		{
			free(interior_params_p->stuTrack_rect_arr);
			interior_params_p->stuTrack_rect_arr = NULL;
		}
		interior_params_p->count_trackObj_stand = 0;
		interior_params_p->count_trackObj_bigMove = 0;
		interior_params_p->count_stuTrack_rect = 0;

		itcReleaseMemStorage(&interior_params_p->stuTrack_storage);

		itc_release_mat(&interior_params_p->currMat);
		itc_release_mat(&interior_params_p->lastMat);
		itc_release_mat(&interior_params_p->mhiMat);
		itc_release_mat(&interior_params_p->maskMat);

		if (inst->flag_setting == FALSE)
		{
			if (inst->stuTrack_size_threshold != NULL)
			{
				free(inst->stuTrack_size_threshold);
				inst->stuTrack_size_threshold = NULL;
			}

			if (inst->stuTrack_direct_threshold != NULL)
			{
				free(inst->stuTrack_direct_threshold);
				inst->stuTrack_direct_threshold = NULL;
			}
		}

		free(inst->interior_params);
		inst->interior_params = NULL;
	}
}

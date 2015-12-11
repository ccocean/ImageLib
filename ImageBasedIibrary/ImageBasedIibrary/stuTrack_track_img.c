#include "stuTrack_track_img.h"
#include "itcTrack_draw_img.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define STATE_STUTRACK_NULL_FLAG	0
#define STATE_STUTRACK_STANDUP_FLAG	1
#define STATE_STUTRACK_SITDOWN_FLAG	(-1)
#define STATE_STUTRACK_MOVE_FLAG	1
#define STATE_STUTRACK_BIG_FLAG		2

#define EXPAND_STUTRACK_INTERSECT_RECT (-3)
static void stuTrack_filtrate_contours(StuITRACK_InteriorParams* interior_params_p, Track_Contour_t** pContour)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_filtrate_contours\n");
	//轮廓筛选
	if (*pContour == NULL || interior_params_p == NULL)
	{	
		return;
	}

	Track_Rect_t *stuTrack_rect_arr = interior_params_p->stuTrack_rect_arr;
	Track_Contour_t	*Contour = *pContour;
	int count_rect = 0;
	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	do
	{
		Track_Rect_t rect = Contour->rect;
		int centre_y = rect.y + rect.height;
		if (rect.width > stuTrack_size_threshold[centre_y] &&
			rect.height > stuTrack_size_threshold[centre_y] &&
			count_rect < COUNT_STUTRACK_MALLOC_ELEMENT)					//筛选
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
			if (track_intersect_rect(stuTrack_rect_arr + i, stuTrack_rect_arr + j, EXPAND_STUTRACK_INTERSECT_RECT))	//判断是否相交，如果相交则直接合并
			{
				count_rect--;
				*(stuTrack_rect_arr + j) = *(stuTrack_rect_arr + count_rect);
				j--;
			}
		}
	}
	interior_params_p->count_stuTrack_rect = count_rect;
}

#define EXPADN_STURECK_ADDSATND_DIRECT_RANGE	10
static int stuTrack_matchingSatnd_ROI(StuITRACK_InteriorParams* interior_params_p, Track_Rect_t roi)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_matchingSatnd_ROI\n");
	//匹配roi
	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	int *stuTrack_direct_threshold = interior_params_p->stuTrack_direct_threshold;
	int stuTrack_direct_range = interior_params_p->stuTrack_direct_range;
	double stuTrack_move_threshold = interior_params_p->stuTrack_move_threshold;
	StuTrack_Stand_t* stuTrack_stand = interior_params_p->stuTrack_stand;
	StuTrack_BigMoveObj_t* stuTrack_bigMOveObj = interior_params_p->stuTrack_bigMOveObj;

	int standard_direct = stuTrack_direct_threshold[roi.x + (roi.width >> 1)];
	int direct = 0;
	//中心点
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);
	int i = 0;
	int flag_ROI = track_calculateDirect_ROI((Itc_Mat_t *)interior_params_p->mhiMat, roi, &direct);
	if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
	{
		_PRINTF("flag_ROI:%d,角度：%d\n", flag_ROI, direct);
	}
	if (flag_ROI == 1)
	{
		if (interior_params_p->count_trackObj_stand > 0)
		{
			int min_ID = 0;
			int min_distance = INT_MAX;
			int distance = 0;
			int diff_x = 0;
			int diff_y = 0;
			for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
			{
				diff_x = x - stuTrack_stand[i].centre.x;
				diff_y = y - stuTrack_stand[i].centre.y;
				distance = diff_x * diff_x + diff_y * diff_y;
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
				track_intersect_rect(&_roi, &(interior_params_p->stuTrack_stand[min_ID].roi), EXPAND_STUTRACK_INTERSECT_RECT);
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
				{	
					_PRINTF("匹配环节：%d！角度：原角度:%d,当前角度:%d，范围:%d\n", stuTrack_stand[min_ID].count_teack, stuTrack_stand[min_ID].direction, direct, stuTrack_direct_range);
				}
				if ((abs(stuTrack_stand[min_ID].direction - direct) <= stuTrack_direct_range))
				{
					stuTrack_stand[min_ID].count_up++;
				}
				else
				{
					stuTrack_stand[min_ID].count_up--;
				}
				stuTrack_stand[min_ID].count_teack++;
				stuTrack_stand[min_ID].flag_matching = TRUE;
				stuTrack_stand[min_ID].direction = (stuTrack_stand[min_ID].direction + direct) >> 1;
				stuTrack_stand[min_ID].direction = ITC_IMAX(stuTrack_stand[min_ID].direction, standard_direct - (stuTrack_direct_range >> 1));
				stuTrack_stand[min_ID].direction = ITC_IMIN(stuTrack_stand[min_ID].direction, standard_direct + (stuTrack_direct_range >> 1));
				stuTrack_stand[min_ID].roi = _roi;
				stuTrack_stand[min_ID].centre = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
				stuTrack_stand[min_ID].current_tClock = gettime();
				return 1;
			}
		}
		
		if (abs(standard_direct - direct) < (stuTrack_direct_range + EXPADN_STURECK_ADDSATND_DIRECT_RANGE) && interior_params_p->count_trackObj_stand < COUNT_STUTRACK_MALLOC_ELEMENT)
		{
			//add
			if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
			{	
				_PRINTF("add stand：origin:%d,%d,size:%d,%d\n", x, y, roi.width, roi.height);
			}
			direct = ITC_IMAX(direct, standard_direct - (stuTrack_direct_range >> 1));
			stuTrack_stand[interior_params_p->count_trackObj_stand].direction = ITC_IMIN(direct, standard_direct + (stuTrack_direct_range >> 1));
			stuTrack_stand[interior_params_p->count_trackObj_stand].count_teack = 1;
			stuTrack_stand[interior_params_p->count_trackObj_stand].count_up = 1;
			stuTrack_stand[interior_params_p->count_trackObj_stand].count_down = 0;
			stuTrack_stand[interior_params_p->count_trackObj_stand].flag_Stand = STATE_STUTRACK_NULL_FLAG;
			stuTrack_stand[interior_params_p->count_trackObj_stand].flag_matching = TRUE;
			stuTrack_stand[interior_params_p->count_trackObj_stand].centre = itcPoint(x, y);
			stuTrack_stand[interior_params_p->count_trackObj_stand].roi = roi;
			stuTrack_stand[interior_params_p->count_trackObj_stand].origin_y = roi.y;
			stuTrack_stand[interior_params_p->count_trackObj_stand].start_tClock = stuTrack_stand[interior_params_p->count_trackObj_stand].current_tClock = gettime();
			interior_params_p->count_trackObj_stand++;
			return 1;
		}
	}

	int intersect_flag = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (stuTrack_stand[i].flag_Stand == STATE_STUTRACK_NULL_FLAG)
		{	
			//与起立预选区不要重叠
			intersect_flag = track_intersect_rect(&roi, &stuTrack_stand[i].roi, -(roi.width >> 1));
		}
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
					//此处待优化
					if (track_intersect_rect(&_roi, &stuTrack_bigMOveObj[i].roi, -(_roi.width >> 1)))
					{
						int maxWidth = ITC_IMAX(stuTrack_bigMOveObj[i].roi.width, roi.width)/2;
						int maxHeight = ITC_IMAX(stuTrack_bigMOveObj[i].roi.height, roi.height) / 2;

						if (abs(stuTrack_bigMOveObj[i].roi.width - roi.width)<maxWidth
							&& abs(stuTrack_bigMOveObj[i].roi.height - roi.height)<maxHeight)
						{
							k = i;
							break;
						}
					}
				}
				if (k >= 0)
				{
					stuTrack_bigMOveObj[k].count_track++;
					stuTrack_bigMOveObj[k].roi = roi;
					stuTrack_bigMOveObj[k].current_position = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
					stuTrack_bigMOveObj[k].current_tClock = gettime();
					return 2;
				}
			}
			if (interior_params_p->count_trackObj_bigMove < COUNT_STUTRACK_MALLOC_ELEMENT)
			{
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
				{	
					_PRINTF("add bigMove：origin:%d,%d,size:%d,%d,移动阈值:%d,大小阈值:%d\n", x, y, roi.width, roi.height, (int)(ITC_IMIN(roi.width, roi.height) * stuTrack_move_threshold), size_threshold1);
				}
				stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].count_track = 1;
				stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].flag_bigMove = STATE_STUTRACK_NULL_FLAG;
				stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].dis_threshold = (int)(ITC_IMIN(roi.width, roi.height) * stuTrack_move_threshold);
				stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].roi = roi;
				stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].origin_position = stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].current_position = itcPoint(x, y);
				stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].start_tClock = stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].current_tClock = gettime();
				interior_params_p->count_trackObj_bigMove++;
				return 2;
			}
		}
	}
	return 0;
}

#define EXPADN_STURECK_STAND_COUNTUP_THRESHOLD	10
#define EXPADN_STURECK_STAND_COUNTTAK_THRESHOLD	(-2)
#define THRESHOLD_STURECK_RATIO_HENIGHTWIDTH	(2.5)
static itc_BOOL stuTrack_judgeStand_ROI(StuITRACK_InteriorParams* interior_params_p, StuTrack_Stand_t track_stand)
{
	//判断是否起立
	if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
	{
		_PRINTF("判断：跟踪次数：%d, 起立帧：%d,高度差：%d,高：%d\n", track_stand.count_teack, track_stand.count_up, track_stand.origin_y - track_stand.roi.y, track_stand.roi.height);
	}
	int stuTrack_standCount_threshold = interior_params_p->stuTrack_standCount_threshold;
	double ratio_lengthWidth = (((double)track_stand.roi.height) / track_stand.roi.width);
	if (((track_stand.count_up > stuTrack_standCount_threshold && track_stand.count_teack > (stuTrack_standCount_threshold + EXPADN_STURECK_STAND_COUNTTAK_THRESHOLD))
		|| track_stand.count_up > (stuTrack_standCount_threshold + EXPADN_STURECK_STAND_COUNTUP_THRESHOLD))
		&& track_stand.origin_y - track_stand.roi.y>(track_stand.roi.height>>3)
		//&& (ratio_lengthWidth - THRESHOLD_STURECK_RATIO_HENIGHTWIDTH) <= ITC_DBL_EPSILON
		)
	{
		return TRUE;
	}
	return FALSE;
}

#define THRESHOLD_STURECK_MOVETIME_DELETE_TIME	1000
#define THRESHOLD_STURECK_STANDTIME_DELETE_TIME	300
#define EXPADN_STURECK_SITDOWN_DIRECT	30
static void stuTrack_analyze_ROI(StuITRACK_InteriorParams* interior_params_p)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_analyze_ROI\n");
	//分析候选区域
	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	int *stuTrack_direct_threshold = interior_params_p->stuTrack_direct_threshold;
	int stuTrack_direct_range = interior_params_p->stuTrack_direct_range;
	int stuTrack_sitdownCount_threshold = interior_params_p->stuTrack_sitdownCount_threshold;
	unsigned int stuTrack_moveDelayed_threshold = (unsigned int)interior_params_p->stuTrack_moveDelayed_threshold;
	StuTrack_Stand_t* stuTrack_stand = interior_params_p->stuTrack_stand;
	StuTrack_BigMoveObj_t* stuTrack_bigMOveObj = interior_params_p->stuTrack_bigMOveObj;
	Itc_Mat_t *mhi = (Itc_Mat_t *)interior_params_p->mhiMat;

	int direct = 0;
	int standard_direct = 0;
	int flag_ROI = 0;
	int i = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (stuTrack_stand[i].flag_Stand != STATE_STUTRACK_STANDUP_FLAG)
		{
			//检测有没有起立
			if (stuTrack_stand[i].flag_matching == FALSE)
			{
				standard_direct = stuTrack_direct_threshold[stuTrack_stand[i].centre.x];
				flag_ROI = track_calculateDirect_ROI(mhi, stuTrack_stand[i].roi, &direct);
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
				{
					_PRINTF("分析环节！角度：原角度:%d,当前角度:%d，范围:%d\n", stuTrack_stand[i].direction, direct, stuTrack_direct_range);
				}
				if ((flag_ROI == 1) && ((abs(stuTrack_stand[i].direction - direct)) <= stuTrack_direct_range))
				{
					stuTrack_stand[i].count_up++;
				}
			}
			if (stuTrack_judgeStand_ROI(interior_params_p, stuTrack_stand[i]))	//确定是否站立
			{
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_1)
				{
					_PRINTF("stand up：origin:%d,%d,size:%d,%d\n", stuTrack_stand[i].centre.x, stuTrack_stand[i].centre.y, stuTrack_stand[i].roi.width, stuTrack_stand[i].roi.height);
				}
				//设置起立的标记
				interior_params_p->result_flag |= RESULT_STUTRACK_STANDUP_FLAG;
				stuTrack_stand[i].flag_Stand = STATE_STUTRACK_STANDUP_FLAG;

				//把与当前起立区域重叠的移动跟踪目标删除
				int width2 = stuTrack_stand[i].roi.width*stuTrack_stand[i].roi.width;
				int height2 = stuTrack_stand[i].roi.height*stuTrack_stand[i].roi.height;
				int diff_x_curr = 0;
				int diff_y_curr = 0;
				int k = 0;
				for (k = 0; k < interior_params_p->count_trackObj_bigMove; k++)
				{
					diff_x_curr = stuTrack_stand[i].centre.x - stuTrack_bigMOveObj[k].current_position.x;
					diff_y_curr = stuTrack_stand[i].centre.y - stuTrack_bigMOveObj[k].current_position.y;
					diff_x_curr *= diff_x_curr;
					diff_y_curr *= diff_y_curr;
					if ((diff_x_curr < width2 && diff_x_curr < height2)
						&& abs(stuTrack_stand[i].roi.width - stuTrack_bigMOveObj[k].roi.width) < (stuTrack_stand[i].roi.width >> 2)
						&& abs(stuTrack_stand[i].roi.height - stuTrack_bigMOveObj[k].roi.height) < (stuTrack_stand[i].roi.height >> 2)
						)
					{
						//删除重叠区域
						stuTrack_bigMOveObj[k] = stuTrack_bigMOveObj[--(interior_params_p->count_trackObj_bigMove)];
						k--;
						continue;
					}
				}
			}
		}
		else if (stuTrack_stand[i].flag_Stand == STATE_STUTRACK_STANDUP_FLAG)
		{
			//检测有没有坐下
			standard_direct = stuTrack_direct_threshold[stuTrack_stand[i].centre.x];
			standard_direct = (standard_direct > ITC_180DEGREE) ? (standard_direct - ITC_180DEGREE) : (standard_direct + ITC_180DEGREE);		//计算与起立方向相反的角度
			flag_ROI = track_calculateDirect_ROI(mhi, stuTrack_stand[i].roi, &direct);
			if ((flag_ROI == 1) && ((abs(standard_direct - direct))<= stuTrack_direct_range + EXPADN_STURECK_SITDOWN_DIRECT))
			{
				stuTrack_stand[i].count_down++;
			}
			if (stuTrack_stand[i].count_down > stuTrack_sitdownCount_threshold)
			{
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_1)
				{
					_PRINTF("sit down：origin:%d,%d,size:%d,%d\n", stuTrack_stand[i].centre.x, stuTrack_stand[i].centre.y, stuTrack_stand[i].roi.width, stuTrack_stand[i].roi.height);
				}
				//设置坐下的标记
				interior_params_p->result_flag |= RESULT_STUTRACK_SITDOWN_FLAG;
				stuTrack_stand[i].flag_Stand = STATE_STUTRACK_SITDOWN_FLAG;
				stuTrack_stand[i].count_teack = 0;
				stuTrack_stand[i].count_up = 0;
				stuTrack_stand[i].count_down = 0;
				stuTrack_stand[i].flag_matching = FALSE;
				continue;
			}
		}
		stuTrack_stand[i].flag_matching = FALSE;

		//判断是否需要删除候选区
		if (stuTrack_stand[i].flag_Stand != STATE_STUTRACK_STANDUP_FLAG)
		{
			unsigned int _time = gettime() - stuTrack_stand[i].current_tClock;
			if (_time > THRESHOLD_STURECK_STANDTIME_DELETE_TIME)				//删除非站立roi
			{
				stuTrack_stand[i] = stuTrack_stand[--(interior_params_p->count_trackObj_stand)];
				i--;
			}
		}
		else
		{
			//有可能变成移动的目标了
			int width2 = stuTrack_stand[i].roi.width*stuTrack_stand[i].roi.width;
			int height2 = stuTrack_stand[i].roi.height*stuTrack_stand[i].roi.height;
			int diff_x_origin = 0;
			int diff_y_origin = 0;
			int diff_x_curr = 0;
			int diff_y_curr = 0;
			int k = 0;
			for (k = 0; k < interior_params_p->count_trackObj_bigMove; k++)
			{
				if (stuTrack_bigMOveObj[k].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
				{
					diff_x_origin = stuTrack_stand[i].centre.x - stuTrack_bigMOveObj[k].origin_position.x;
					diff_y_origin = stuTrack_stand[i].centre.y - stuTrack_bigMOveObj[k].origin_position.y;
					diff_x_origin *= diff_x_origin;
					diff_y_origin *= diff_y_origin;
					if ((diff_x_origin<width2 && diff_y_origin<height2)
						&& abs(stuTrack_stand[i].roi.width - stuTrack_bigMOveObj[k].roi.width) < (stuTrack_stand[i].roi.width >> 1)
						&& abs(stuTrack_stand[i].roi.height - stuTrack_bigMOveObj[k].roi.height) < (stuTrack_stand[i].roi.height >> 1)
						)
					{
						//与移动目标在重叠,计算移动目标当前的位置是否已经远离
						diff_x_curr = stuTrack_stand[i].centre.x - stuTrack_bigMOveObj[k].current_position.x;
						diff_y_curr = stuTrack_stand[i].centre.y - stuTrack_bigMOveObj[k].current_position.y;
						diff_x_curr *= diff_x_curr;
						diff_y_curr *= diff_y_curr;
						if (diff_x_curr>(width2 >> 1) || diff_y_curr>(height2 >> 1))
						{
							//目标已经离开
							if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_1)
							{
								_PRINTF("stand to bigMove：origin:%d,%d,size:%d,%d\n", stuTrack_stand[i].centre.x, stuTrack_stand[i].centre.y, stuTrack_stand[i].roi.width, stuTrack_stand[i].roi.height);
							}
							stuTrack_stand[i] = stuTrack_stand[--(interior_params_p->count_trackObj_stand)];
							i--;
						}
						break;//跳出k循环
					}
				}
			}
		}
	}

	//分析移动的目标
	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{
		unsigned int _time = gettime() - stuTrack_bigMOveObj[i].current_tClock;
		if (_time > THRESHOLD_STURECK_MOVETIME_DELETE_TIME)
		{
			if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
			{
				_PRINTF("delete bigMove:origin:%d,%d,current:%d,%d\n", stuTrack_bigMOveObj[i].origin_position.x, stuTrack_bigMOveObj[i].origin_position.y, stuTrack_bigMOveObj[i].current_position.x, stuTrack_bigMOveObj[i].current_position.y);
			}
			if (stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
			{	
				interior_params_p->result_flag |= RESULT_STUTRACK_STOPMOVE_FLAG;//设置停止运动的标记
			}
			//删除长时间不运动的目标
			stuTrack_bigMOveObj[i] = stuTrack_bigMOveObj[--(interior_params_p->count_trackObj_bigMove)];
			i--;
			continue;
		}
		if (stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_NULL_FLAG)
		{
			int diff_x = abs(stuTrack_bigMOveObj[i].origin_position.x - stuTrack_bigMOveObj[i].current_position.x);
			int diff_y = abs(stuTrack_bigMOveObj[i].origin_position.y - stuTrack_bigMOveObj[i].current_position.y);
			if (diff_x>stuTrack_bigMOveObj[i].dis_threshold || diff_y>stuTrack_bigMOveObj[i].dis_threshold)
			{
				_time = stuTrack_bigMOveObj[i].current_tClock - stuTrack_bigMOveObj[i].start_tClock;
				if (_time > stuTrack_moveDelayed_threshold)
				{
					if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_1)
					{
						_PRINTF("find Move：origin:%d,%d,size:%d,%d\n", stuTrack_bigMOveObj[i].origin_position.x, stuTrack_bigMOveObj[i].origin_position.y, stuTrack_bigMOveObj[i].roi.width, stuTrack_bigMOveObj[i].roi.height);
					}
					interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//设置移动目标的标记
					stuTrack_bigMOveObj[i].flag_bigMove = STATE_STUTRACK_MOVE_FLAG;
				}
			}
			else 
			{
				int centre_y = stuTrack_bigMOveObj[i].current_position.y;
				int size_threshold = stuTrack_size_threshold[centre_y] + stuTrack_size_threshold[centre_y];
				if (stuTrack_bigMOveObj[i].roi.width > size_threshold && stuTrack_bigMOveObj[i].roi.height > size_threshold)
				{
					if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_1)
					{
						_PRINTF("find big：origin:%d,%d,size:%d,%d\n", stuTrack_bigMOveObj[i].origin_position.x, stuTrack_bigMOveObj[i].origin_position.y, stuTrack_bigMOveObj[i].roi.width, stuTrack_bigMOveObj[i].roi.height);
					}
					interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//设置移动目标的标记
					stuTrack_bigMOveObj[i].flag_bigMove = STATE_STUTRACK_BIG_FLAG;	
				}
			}
		}
	}
}

static void stuTrack_proStandDown_ROI(StuITRACK_InteriorParams* interior_params_p)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_proStandDown_ROI\n");
	//匹配和分析候选区域
	int i = 0;
	for (i = 0; i < interior_params_p->count_stuTrack_rect; i++)
	{
		stuTrack_matchingSatnd_ROI(interior_params_p, interior_params_p->stuTrack_rect_arr[i]);
	}
	stuTrack_analyze_ROI(interior_params_p);			//分析候选roi
}

static void stuTrack_drawShow_imgData(StuITRACK_InteriorParams* interior_params_p, itc_uchar* imageData, itc_uchar* bufferuv)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_drawShow_imgData\n");
	if (interior_params_p->stuTrack_Draw_flag == FALSE)
	{	
		return;
	}

	//画出结果
	int i = 0;
	Track_Size_t *srcimg_size = &interior_params_p->srcimg_size;	//原始图像大小
	StuTrack_Stand_t* stuTrack_stand = interior_params_p->stuTrack_stand;
	StuTrack_BigMoveObj_t* stuTrack_bigMOveObj = interior_params_p->stuTrack_bigMOveObj;

	Track_Rect_t *rect;
	Track_Point_t *current_position;
	Track_Point_t *origin_position;

#ifdef _WIN32
	int YUV420_type = TRACK_DRAW_YUV420P;
#else
	int YUV420_type = TRACK_DRAW_YUV420SP;
#endif

	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{
		rect = &stuTrack_bigMOveObj[i].roi;
		current_position = &stuTrack_bigMOveObj[i].current_position;
		origin_position = &stuTrack_bigMOveObj[i].origin_position;
		if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
		{
			if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_MOVE_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->pink_colour, YUV420_type);
			}
			else if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_BIG_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->blue_colour, YUV420_type);
			}
			//track_draw_line(imageData, bufferuv, srcimg_size, current_position, origin_position, &interior_params_p->green_colour, YUV420_type);//
		}
		else
		{
			//候选的移动目标
			track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->lilac_colour, YUV420_type);
		}
	}

	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		rect = &stuTrack_stand[i].roi;
		if (interior_params_p->stuTrack_stand[i].flag_Stand != STATE_STUTRACK_NULL_FLAG)
		{
			if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_STANDUP_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->red_colour, YUV420_type);
			}
			else if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_SITDOWN_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->dullred_colour, YUV420_type);
			}
		}
		else
		{
			//候选的起立区
			track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->yellow_colour, YUV420_type);
		}
	}

	Track_Rect_t *stuTrack_rect_arr = interior_params_p->stuTrack_rect_arr;
	for (i = 0; i < interior_params_p->count_stuTrack_rect; i++)
	{
		int direct;
		track_calculateDirect_ROI((Itc_Mat_t *)interior_params_p->mhiMat, stuTrack_rect_arr[i], &direct);
		//_PRINTF("角度：%d\n", direct);
		int x1 = 20 * cos(direct*ITC_PI / ITC_180DEGREE);
		int y1 = 20 * sin(direct*ITC_PI / ITC_180DEGREE);
		Track_Point_t pt1 = { 0, 0 };
		pt1.x = stuTrack_rect_arr[i].x + stuTrack_rect_arr[i].width / 2;
		pt1.y = stuTrack_rect_arr[i].y + stuTrack_rect_arr[i].height / 2;
		Track_Point_t pt2 = { 0, 0 };
		pt2.x = pt1.x + x1;
		pt2.y = pt1.y + y1;
		track_draw_line(imageData, bufferuv, srcimg_size, &pt1, &pt2, &interior_params_p->green_colour, YUV420_type);
		track_draw_point(imageData, bufferuv, srcimg_size, &pt1, &interior_params_p->red_colour, YUV420_type);
	}
}

static void stuTrack_reslut(StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_reslut\n");
	//填写返回结果结构体
	if (interior_params_p->result_flag != RESULT_STUTRACK_NULL_FLAG)
	{
		return_params->result_flag = interior_params_p->result_flag | RESULT_STUTRACK_NEWCHANGE_FLAG;		//当前帧的变化,设置有变化的标记
	}
	return_params->count_trackObj_stand = 0;		//移动目标个数
	return_params->count_trackObj_bigMove = 0;	//起立目标个数
	int i = 0;
	int id = 0;
	for (i = 0; i < interior_params_p->count_trackObj_bigMove;i++)
	{
		if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
		{
			//将最新的目标位置返回
			id = i;
			return_params->count_trackObj_bigMove++;
		}
	}
	if (return_params->count_trackObj_bigMove>0)
	{
		Track_Point_t pt = { 0, 0 };
		perspectiveConvert(&(interior_params_p->stuTrack_bigMOveObj[id].current_position), &pt, interior_params_p->transformationMatrix);
		return_params->move_position.x = pt.x;
		return_params->move_position.y = pt.y;
		return_params->moveObj_size.width = interior_params_p->stuTrack_bigMOveObj[id].roi.width;
		return_params->moveObj_size.height = interior_params_p->stuTrack_bigMOveObj[id].roi.height;

		int size = ITC_IMAX(return_params->moveObj_size.width, return_params->moveObj_size.height);
		return_params->stretchingCoefficient_bigMove=(int)(interior_params_p->stretchingAB[0] * size + interior_params_p->stretchingAB[1]);
		return_params->stretchingCoefficient_bigMove = return_params->stretchingCoefficient_bigMove > 0 ? return_params->stretchingCoefficient_bigMove : 0;
	}

	id = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_STANDUP_FLAG)
		{
			//位置指向最新的站立区域
			id = i;
			return_params->count_trackObj_stand++;
		}
	}
	if (return_params->count_trackObj_stand > 0)
	{
		Track_Point_t pt = {0,0};
		perspectiveConvert(&(interior_params_p->stuTrack_stand[id].centre), &pt, interior_params_p->transformationMatrix);
		return_params->stand_position.x = pt.x;
		return_params->stand_position.y = pt.y;
		return_params->standObj_size.width = interior_params_p->stuTrack_stand[id].roi.width;
		return_params->standObj_size.height = interior_params_p->stuTrack_stand[id].roi.height;

		int size = ITC_IMAX(return_params->standObj_size.width, return_params->standObj_size.height);
		return_params->stretchingCoefficient_stand = (int)(interior_params_p->stretchingAB[0] * size + interior_params_p->stretchingAB[1]);
		return_params->stretchingCoefficient_stand = return_params->stretchingCoefficient_stand > 0 ? return_params->stretchingCoefficient_stand : 0;
	}
}

static void stuTrack_Copy_matData(StuITRACK_InteriorParams* interior_params_p, itc_uchar* srcData)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_resizeCopy_matData\n");
	int y = 0;
	int height = interior_params_p->img_size.height;
	int dst_step = interior_params_p->img_size.width;
	int src_step = interior_params_p->srcimg_size.width;
	if (dst_step > src_step)
	{
		_PRINTF("The image cache size error!\n");
		return;
	}

	itc_uchar* dst_p = interior_params_p->currMat->data.ptr;
	for (y = 0; y < height; y++)
	{
		memcpy(dst_p, srcData, sizeof(itc_uchar)* dst_step);
		dst_p += dst_step;
		srcData += src_step;
	}
}
#define CHECH_STURRACK_RESULT_OK 0
#define CHECH_STURRACK_RESULT_ERROR1 1
#define CHECH_STURRACK_RESULT_ERROR2 2
#define CHECH_STURRACK_RESULT_ERROR3 3
#define CHECH_STURRACK_RESULT_ERROR4 4
#define CHECH_STURRACK_RESULT_ERROR5 5
#define CHECH_STURRACK_RESULT_ERROR6 6
#define CHECH_STURRACK_RESULT_ERROR7 7
static int stuTrack_check_clientParams(const StuITRACK_ClientParams_t* clientParams_p)
{

	if (clientParams_p->stuTrack_vertex[0].x < 0 ||
		clientParams_p->stuTrack_vertex[1].x < 0 ||
		clientParams_p->stuTrack_vertex[2].x < 0 ||
		clientParams_p->stuTrack_vertex[3].x < 0 ||
		clientParams_p->stuTrack_vertex[0].y < 0 ||
		clientParams_p->stuTrack_vertex[1].y < 0 ||
		clientParams_p->stuTrack_vertex[2].y < 0 ||
		clientParams_p->stuTrack_vertex[3].y < 0)
	{
		return CHECH_STURRACK_RESULT_ERROR1;
	}

	if (clientParams_p->stuTrack_move_threshold < 0.01)
	{
		return CHECH_STURRACK_RESULT_ERROR2;
	}

	if (clientParams_p->stuTrack_standCount_threshold < 0)
	{
		return CHECH_STURRACK_RESULT_ERROR3;
	}

	if (clientParams_p->stuTrack_sitdownCount_threshold < 0)
	{
		return CHECH_STURRACK_RESULT_ERROR4;
	}

	if (clientParams_p->stuTrack_moveDelayed_threshold < 0)
	{
		return CHECH_STURRACK_RESULT_ERROR5;
	}

	if (clientParams_p->stuTrack_direct_range < 0)
	{
		return CHECH_STURRACK_RESULT_ERROR6;
	}

	if (clientParams_p->height <= 0 || clientParams_p->width <= 0)
	{
		return CHECH_STURRACK_RESULT_ERROR7;
	}

	return CHECH_STURRACK_RESULT_OK;
}

itc_BOOL stuTrack_initializeTrack(const StuITRACK_Params * inst, StuITRACK_InteriorParams* interior_params_p)
{
	if (inst == NULL || interior_params_p == NULL)
	{
		return FALSE;
	}
	stuTrack_stopTrack(inst, interior_params_p);

	if (inst->systemParams.callbackmsg_func != NULL)
	{	interior_params_p->callbackmsg_func = inst->systemParams.callbackmsg_func;}
	else
	{	interior_params_p->callbackmsg_func = printf;}
	interior_params_p->srcimg_size.width = 0;
	interior_params_p->srcimg_size.height = 0;
	interior_params_p->initialize_flag = FALSE;

	double size_threshold_a, size_threshold_b;
	double direct_threshold_a, direct_threshold_b;
	if ((inst->clientParams.flag_setting == TRUE) && (stuTrack_check_clientParams(&inst->clientParams) == CHECH_STURRACK_RESULT_OK))
	{
		//非默认参数
		int y1 = (inst->clientParams.stuTrack_vertex[0].y + inst->clientParams.stuTrack_vertex[1].y) / 2;
		int y2 = (inst->clientParams.stuTrack_vertex[2].y + inst->clientParams.stuTrack_vertex[3].y) / 2;
		if (y1 == y2)
		{
			_PRINTF("The input parameter error:y1 == y2!\n");
			return FALSE;
		}
		int width1 = (inst->clientParams.stuTrack_stuWidth_standard[0] + inst->clientParams.stuTrack_stuWidth_standard[1]) / 2;
		int width2 = (inst->clientParams.stuTrack_stuWidth_standard[2] + inst->clientParams.stuTrack_stuWidth_standard[3]) / 2;
		size_threshold_a = ((double)(width1 - width2)) / (y1 - y2);
		size_threshold_b = width1 - size_threshold_a*y1;

		int x1 = (inst->clientParams.stuTrack_vertex[0].x + inst->clientParams.stuTrack_vertex[3].x) / 2;
		int x2 = (inst->clientParams.stuTrack_vertex[1].x + inst->clientParams.stuTrack_vertex[2].x) / 2;
		if (x1 == x2)
		{
			_PRINTF("The input parameter error:x1 == x2!\n");
			return FALSE;
		}
		int direct1 = (ITC_NORM_ANGLE360(inst->clientParams.stuTrack_direct_standard[0]) + ITC_NORM_ANGLE360(inst->clientParams.stuTrack_direct_standard[3])) / 2;
		int direct2 = (ITC_NORM_ANGLE360(inst->clientParams.stuTrack_direct_standard[1]) + ITC_NORM_ANGLE360(inst->clientParams.stuTrack_direct_standard[2])) / 2;
		direct_threshold_a = ((double)(direct1 - direct2)) / (x1 - x2);
		direct_threshold_b = direct1 - direct_threshold_a*x1;

		interior_params_p->stuTrack_debugMsg_flag			= inst->clientParams.stuTrack_debugMsg_flag;
		interior_params_p->stuTrack_Draw_flag				= inst->clientParams.stuTrack_Draw_flag;
		interior_params_p->stuTrack_move_threshold			= inst->clientParams.stuTrack_move_threshold;			//判定是移动目标的偏离阈值（相对于自身宽度的比值）
		interior_params_p->stuTrack_standCount_threshold	= inst->clientParams.stuTrack_standCount_threshold;		//判定为起立的帧数阈值
		interior_params_p->stuTrack_sitdownCount_threshold	= inst->clientParams.stuTrack_sitdownCount_threshold;	//判定为坐下的帧数阈值
		interior_params_p->stuTrack_moveDelayed_threshold	= inst->clientParams.stuTrack_moveDelayed_threshold;
		interior_params_p->stuTrack_direct_range			= inst->clientParams.stuTrack_direct_range;
		interior_params_p->img_size.width	= (inst->clientParams.width  + ITC_IMAGE_ALIGN - 1)&~(ITC_IMAGE_ALIGN - 1);		//对齐到8位
		interior_params_p->img_size.height	= (inst->clientParams.height + ITC_IMAGE_ALIGN - 1)&~(ITC_IMAGE_ALIGN - 1);

		interior_params_p->transformationMatrix = itc_create_mat(3, 3, ITC_64FC1);
		interior_params_p->transformationMatrix->data.db[0] = inst->clientParams.transformationMatrix[0];
		interior_params_p->transformationMatrix->data.db[1] = inst->clientParams.transformationMatrix[1];
		interior_params_p->transformationMatrix->data.db[2] = inst->clientParams.transformationMatrix[2];
		interior_params_p->transformationMatrix->data.db[3] = inst->clientParams.transformationMatrix[3];
		interior_params_p->transformationMatrix->data.db[4] = inst->clientParams.transformationMatrix[4];
		interior_params_p->transformationMatrix->data.db[5] = inst->clientParams.transformationMatrix[5];
		interior_params_p->transformationMatrix->data.db[6] = inst->clientParams.transformationMatrix[6];
		interior_params_p->transformationMatrix->data.db[7] = inst->clientParams.transformationMatrix[7];
		interior_params_p->transformationMatrix->data.db[8] = inst->clientParams.transformationMatrix[8];

		interior_params_p->stretchingAB[0] = inst->clientParams.stretchingAB[0];
		interior_params_p->stretchingAB[1] = inst->clientParams.stretchingAB[1];
	}
	else
	{
		//默认的参数
		size_threshold_a	= A_STUTRACK_SIZE_THRESHOLD_PARAMS;
		size_threshold_b	= B_STUTRACK_SIZE_THRESHOLD_PARAMS;
		direct_threshold_a	= A_STUTRACK_DIRECT_THRESHOLD_PARAMS;
		direct_threshold_b	= B_STUTRACK_DIRECT_THRESHOLD_PARAMS;
		interior_params_p->stuTrack_debugMsg_flag			= 1;
		interior_params_p->stuTrack_Draw_flag				= TRUE;
		interior_params_p->stuTrack_move_threshold			= THRESHOLD_STUTRACK_MOVE_DEFALUT_PARAMS;			//判定是移动目标的偏离阈值（相对于自身宽度的比值）
		interior_params_p->stuTrack_standCount_threshold	= THRESHOLD_STUTRACK_STANDCOUNT_DEFALUT_PARAMS;		//判定为起立的帧数阈值
		interior_params_p->stuTrack_sitdownCount_threshold	= THRESHOLD_STUTRACK_SITDOWNCOUNT_DEFALUT_PARAMS;	//判定为坐下的帧数阈值
		interior_params_p->stuTrack_moveDelayed_threshold	= THRESHOLD_STUTRACK_MOVEDELAYED_DEFALUT_PARAMS;
		interior_params_p->stuTrack_direct_range			= RANGE_STUTRACK_STANDDIRECT_DEFALUT_PARAMS;
		interior_params_p->img_size.width					= WIDTH_STUTRACK_IMG_;
		interior_params_p->img_size.height					= HEIGHT_STUTRACK_IMG_;

		interior_params_p->transformationMatrix=itc_create_mat(3, 3, ITC_64FC1);
		interior_params_p->transformationMatrix->data.db[0] = 1.0;
		interior_params_p->transformationMatrix->data.db[1] = 0.0;
		interior_params_p->transformationMatrix->data.db[2] = 0.0;
		interior_params_p->transformationMatrix->data.db[3] = 0.0;
		interior_params_p->transformationMatrix->data.db[4] = 1.0;
		interior_params_p->transformationMatrix->data.db[5] = 0.0;
		interior_params_p->transformationMatrix->data.db[6] = 0.0;
		interior_params_p->transformationMatrix->data.db[7] = 0.0;
		interior_params_p->transformationMatrix->data.db[8] = 1.0;
		interior_params_p->stretchingAB[0] = -1.0;
		interior_params_p->stretchingAB[1] = 0.0;
	}

	//初始化自有的内部统计参数
	interior_params_p->_count = 0;
	interior_params_p->count_trackObj_stand = 0;
	interior_params_p->count_trackObj_bigMove = 0;
	interior_params_p->count_stuTrack_rect = 0;

	//分配内存
	interior_params_p->currMat = itc_create_mat(interior_params_p->img_size.height, interior_params_p->img_size.width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->currMat, FALSE);

	interior_params_p->lastMat = itc_create_mat(interior_params_p->img_size.height, interior_params_p->img_size.width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->lastMat, FALSE);

	interior_params_p->mhiMat = itc_create_mat(interior_params_p->img_size.height, interior_params_p->img_size.width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->mhiMat, FALSE);

	interior_params_p->maskMat = itc_create_mat(interior_params_p->img_size.height, interior_params_p->img_size.width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->maskMat, FALSE);

	interior_params_p->stuTrack_storage = itcCreateMemStorage(0);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_storage, FALSE);

	interior_params_p->stuTrack_stand = (StuTrack_Stand_t*)itcAlloc(sizeof(StuTrack_Stand_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_stand, FALSE);

	interior_params_p->stuTrack_bigMOveObj = (StuTrack_BigMoveObj_t*)itcAlloc(sizeof(StuTrack_BigMoveObj_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_bigMOveObj, FALSE);

	interior_params_p->stuTrack_rect_arr = (Track_Rect_t*)itcAlloc(sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_rect_arr, FALSE);

	interior_params_p->stuTrack_size_threshold = (int *)itcAlloc(sizeof(int)* interior_params_p->img_size.height);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_size_threshold, FALSE);

	interior_params_p->stuTrack_direct_threshold = (int *)itcAlloc(sizeof(int)* interior_params_p->img_size.width);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_direct_threshold, FALSE);

	memset(interior_params_p->stuTrack_stand, 0, sizeof(StuTrack_Stand_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_bigMOveObj, 0, sizeof(StuTrack_BigMoveObj_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_rect_arr, 0, sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_size_threshold, 0, sizeof(int)* interior_params_p->img_size.height);
	memset(interior_params_p->stuTrack_direct_threshold, 0, sizeof(int)* interior_params_p->img_size.width);

	int x = 0;
	for (x = 0; x < interior_params_p->img_size.width; x++)
	{
		//初始化标准起立角度
		interior_params_p->stuTrack_direct_threshold[x] = (int)(COMPUTER_STUTRACK_DIRECT_THRESHOLD_PARAMS(x, direct_threshold_a, direct_threshold_b) + 0.5);
	}
	int y = 0;
	for (y = 0; y < interior_params_p->img_size.height; y++)
	{
		//初始化筛选尺寸阈值
		interior_params_p->stuTrack_size_threshold[y] = (int)(COMPUTER_STUTRACK_SIZE_THRESHOLD_PARAMS(y, size_threshold_a, size_threshold_b) + 0.5);
	}
	
	//初始化绘制颜色
	interior_params_p->pink_colour = colour_RGB2YUV(255, 0, 255);	/*粉红*/
	interior_params_p->blue_colour = colour_RGB2YUV(0, 0, 255);		/*纯蓝*/
	interior_params_p->lilac_colour = colour_RGB2YUV(155, 155, 255);/*淡紫*/
	interior_params_p->green_colour = colour_RGB2YUV(0, 255, 0);	/*纯绿*/
	interior_params_p->red_colour = colour_RGB2YUV(255, 0, 0);		/*纯红*/
	interior_params_p->dullred_colour = colour_RGB2YUV(127, 0, 0);	/*暗红*/
	interior_params_p->yellow_colour = colour_RGB2YUV(255, 255, 0);	/*纯黄*/

	interior_params_p->initialize_flag = TRUE;
	return TRUE;
}

#define THRESHOLD_STUTRACK_FRAME_DIFF	12
#define THERSHOLD_STUTRAKC_HMI_MASK		243
void stuTrack_process(const StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params, char* imageData, char* bufferuv)
{
	if (imageData == NULL || return_params == NULL || interior_params_p == NULL || inst == NULL )
	{
		return;
	}
	else if (interior_params_p->initialize_flag == FALSE)
	{
		_PRINTF("Failed to initialize!\n");
		return;
	}
	ITC_FUNCNAME("FUNCNAME:stuTrack_process\n");
	if (interior_params_p->srcimg_size.width == 0 || interior_params_p->srcimg_size.height == 0)																									 
	{																																																 
		if (inst->systemParams.nsrcWidth == 0 || inst->systemParams.nsrcHeight == 0)																												 
		{																																															 
			_PRINTF("The original image size error!\n");																																			 
			return;																																													 
		}																																															 
		interior_params_p->srcimg_size.width = inst->systemParams.nsrcWidth;																														 
		interior_params_p->srcimg_size.height = inst->systemParams.nsrcHeight;									
	}

	stuTrack_Copy_matData(interior_params_p, (itc_uchar*)imageData);
	interior_params_p->result_flag = RESULT_STUTRACK_NULL_FLAG;	//清空变化状态
	Track_Contour_t* firstContour = NULL;
	if (interior_params_p->_count>1)
	{
		itcClearMemStorage(interior_params_p->stuTrack_storage);
		track_update_MHI(interior_params_p->currMat, interior_params_p->lastMat, interior_params_p->mhiMat, THRESHOLD_STUTRACK_FRAME_DIFF, interior_params_p->maskMat, THERSHOLD_STUTRAKC_HMI_MASK);
		track_find_contours(interior_params_p->maskMat, &firstContour, interior_params_p->stuTrack_storage);
		stuTrack_filtrate_contours(interior_params_p,&firstContour);
		stuTrack_proStandDown_ROI(interior_params_p);
	}

	ITC_SWAP(interior_params_p->currMat, interior_params_p->lastMat, interior_params_p->tempMat);
	interior_params_p->_count++;

	stuTrack_drawShow_imgData(interior_params_p, (itc_uchar*)imageData, (itc_uchar*)bufferuv);	//绘制处理效果
	stuTrack_reslut(interior_params_p, return_params);									//填写返回结果
}

void stuTrack_stopTrack(const StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	if (inst == NULL || interior_params_p == NULL)
	{
		return;
	}
	ITC_FUNCNAME("FUNCNAME:stuTrack_stopTrack\n");

	if (interior_params_p->stuTrack_size_threshold != NULL)
	{
		itcFree_(interior_params_p->stuTrack_size_threshold);
		interior_params_p->stuTrack_size_threshold = NULL;
	}

	if (interior_params_p->stuTrack_direct_threshold != NULL)
	{
		itcFree_(interior_params_p->stuTrack_direct_threshold);
		interior_params_p->stuTrack_direct_threshold = NULL;
	}

	if (interior_params_p->stuTrack_stand != NULL)
	{
		itcFree_(interior_params_p->stuTrack_stand);
		interior_params_p->stuTrack_stand = NULL;
	}

	if (interior_params_p->stuTrack_bigMOveObj != NULL)
	{
		itcFree_(interior_params_p->stuTrack_bigMOveObj);
		interior_params_p->stuTrack_bigMOveObj = NULL;
	}

	if (interior_params_p->stuTrack_rect_arr != NULL)
	{
		itcFree_(interior_params_p->stuTrack_rect_arr);
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
	interior_params_p->initialize_flag = FALSE;
}

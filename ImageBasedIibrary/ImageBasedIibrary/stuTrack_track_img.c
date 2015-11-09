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
static void stuTrack_filtrate_contours(StuITRACK_InteriorParams* interior_params_p,Track_Contour_t** pContour)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_filtrate_contours\n");
	//轮廓筛选
	if (*pContour == NULL || interior_params_p == NULL)
		return;

	int count_rect = 0;
	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	Track_Rect_t *stuTrack_rect_arr = interior_params_p->stuTrack_rect_arr;
	Track_Contour_t	*Contour = *pContour;
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
			int diff_x = 0;
			int diff_y = 0;
			for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
			{
				diff_x = x - interior_params_p->stuTrack_stand[i].centre.x;
				diff_y = y - interior_params_p->stuTrack_stand[i].centre.y;
				distance = diff_x*diff_x + diff_y + diff_y;
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
				track_intersect_rect(&_roi, &(interior_params_p->stuTrack_stand[min_ID].roi), EXPAND_STUTRACK_INTERSECT_RECT);
				//_PRINTF("角度：原角度:%d,当前角度:%d，范围:%d\n", interior_params_p->stuTrack_stand[min_ID].direction, direct, stuTrack_direct_range);
				if ((abs(interior_params_p->stuTrack_stand[min_ID].direction - direct) <= stuTrack_direct_range))
				{
					interior_params_p->stuTrack_stand[min_ID].count_up++;
				}
				else
				{
					interior_params_p->stuTrack_stand[min_ID].count_up--;
				}
				interior_params_p->stuTrack_stand[min_ID].count_teack++;
				interior_params_p->stuTrack_stand[min_ID].flag_matching = TRUE;
				interior_params_p->stuTrack_stand[min_ID].direction = (interior_params_p->stuTrack_stand[min_ID].direction + direct) >> 1;
				interior_params_p->stuTrack_stand[min_ID].direction = ITC_IMAX(interior_params_p->stuTrack_stand[min_ID].direction, standard_direct - (stuTrack_direct_range >> 1));
				interior_params_p->stuTrack_stand[min_ID].direction = ITC_IMIN(interior_params_p->stuTrack_stand[min_ID].direction, standard_direct + (stuTrack_direct_range >> 1));
				interior_params_p->stuTrack_stand[min_ID].roi = _roi;
				interior_params_p->stuTrack_stand[min_ID].centre = itcPoint(_roi.x + (_roi.width >> 1), _roi.y + (_roi.height >> 1));
				interior_params_p->stuTrack_stand[min_ID].current_tClock = clock();
				return 1;
			}
		}

		if (abs(standard_direct - direct) < (stuTrack_direct_range + EXPADN_STURECK_ADDSATND_DIRECT_RANGE) && interior_params_p->count_trackObj_stand < COUNT_STUTRACK_MALLOC_ELEMENT)
		{
			//add
			//_PRINTF("add stand：origin:%d,%d,size:%d,%d\n", x, y, roi.width, roi.height);
			direct = ITC_IMAX(direct, standard_direct - (stuTrack_direct_range >> 1));
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].direction = ITC_IMIN(direct, standard_direct + (stuTrack_direct_range >> 1));
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].count_teack = 1;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].count_up = 1;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].count_down = 0;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].flag_Stand = STATE_STUTRACK_NULL_FLAG;
			interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand].flag_matching = TRUE;
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
					//此处待优化
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
			if (interior_params_p->count_trackObj_bigMove < COUNT_STUTRACK_MALLOC_ELEMENT)
			{
				//_PRINTF("add bigMove：origin:%d,%d,size:%d,%d\n", x, y, roi.width, roi.height);
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].count_track = 1;
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].flag_bigMove = STATE_STUTRACK_NULL_FLAG;
				interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove].dis_threshold = (int)(ITC_IMIN(roi.width, roi.height) * stuTrack_move_threshold);
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

#define EXPADN_STURECK_STAND_COUNTUP_THRESHOLD	10
#define EXPADN_STURECK_STAND_COUNTTAK_THRESHOLD	(-2)
#define THRESHOLD_STURECK_RATIO_HENIGHTWIDTH	(2.1)
static BOOL stuTrack_judgeStand_ROI(StuITRACK_InteriorParams* interior_params_p, StuTrack_Stand_t track_stand)
{
	//判断是否起立
	int stuTrack_standCount_threshold = interior_params_p->stuTrack_standCount_threshold;
	double ratio_lengthWidth = (((double)track_stand.roi.height) / track_stand.roi.width);
	if (((track_stand.count_up > stuTrack_standCount_threshold && track_stand.count_teack > (stuTrack_standCount_threshold + EXPADN_STURECK_STAND_COUNTTAK_THRESHOLD))
		|| track_stand.count_up > (stuTrack_standCount_threshold + EXPADN_STURECK_STAND_COUNTUP_THRESHOLD))
		&& (ratio_lengthWidth - THRESHOLD_STURECK_RATIO_HENIGHTWIDTH) <= DBL_EPSILON)
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
	int stuTrack_moveDelayed_threshold = interior_params_p->stuTrack_moveDelayed_threshold;

	Itc_Mat_t *mhi = (Itc_Mat_t *)interior_params_p->mhiMat;

	int direct = 0;
	int standard_direct = 0;
	int flag_ROI = 0;
	int i = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (interior_params_p->stuTrack_stand[i].flag_Stand != STATE_STUTRACK_STANDUP_FLAG)
		{
			//检测有没有起立
			if (interior_params_p->stuTrack_stand[i].flag_matching == FALSE)
			{
				standard_direct = stuTrack_direct_threshold[interior_params_p->stuTrack_stand[i].centre.x];
				flag_ROI = track_calculateDirect_ROI(mhi, interior_params_p->stuTrack_stand[i].roi, &direct);
				if ((flag_ROI == 1) && ((abs(interior_params_p->stuTrack_stand[i].direction - direct)) < stuTrack_direct_range))
				{
					interior_params_p->stuTrack_stand[i].count_up++;
				}
			}
			//_PRINTF("判断：%d, %d\n", interior_params_p->stuTrack_stand[i].count_teack, interior_params_p->stuTrack_stand[i].count_up);
			if (stuTrack_judgeStand_ROI(interior_params_p, interior_params_p->stuTrack_stand[i]))	//确定是否站立
			{
				_PRINTF("stand up：origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_stand[i].centre.x, interior_params_p->stuTrack_stand[i].centre.y, interior_params_p->stuTrack_stand[i].roi.width, interior_params_p->stuTrack_stand[i].roi.height);
				//设置起立的标记
				interior_params_p->result_flag |= RESULT_STUTRACK_STANDUP_FLAG;
				interior_params_p->stuTrack_stand[i].flag_Stand = STATE_STUTRACK_STANDUP_FLAG;
			}
		}
		else
		{
			//检测有没有坐下
			standard_direct = stuTrack_direct_threshold[interior_params_p->stuTrack_stand[i].centre.x];
			standard_direct = (standard_direct > ITC_DEGREES) ? (standard_direct - ITC_DEGREES) : (standard_direct + ITC_DEGREES);		//计算与起立方向相反的角度
			flag_ROI = track_calculateDirect_ROI(mhi, interior_params_p->stuTrack_stand[i].roi, &direct);
			if ((flag_ROI == 1) && ((abs(standard_direct - direct))< stuTrack_direct_range + EXPADN_STURECK_SITDOWN_DIRECT))
			{
				interior_params_p->stuTrack_stand[i].count_down++;
				if (interior_params_p->stuTrack_stand[i].count_down>stuTrack_sitdownCount_threshold)
				{
					_PRINTF("sit down：origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_stand[i].centre.x, interior_params_p->stuTrack_stand[i].centre.y, interior_params_p->stuTrack_stand[i].roi.width, interior_params_p->stuTrack_stand[i].roi.height);
					//设置坐下的标记
					interior_params_p->result_flag |= RESULT_STUTRACK_SITDOWN_FLAG;
					interior_params_p->stuTrack_stand[i].flag_Stand = STATE_STUTRACK_SITDOWN_FLAG;
					interior_params_p->stuTrack_stand[i].count_teack = 0;
					interior_params_p->stuTrack_stand[i].count_up = 0;
					interior_params_p->stuTrack_stand[i].count_down = 0;
					interior_params_p->stuTrack_stand[i].flag_matching = FALSE;
					continue;
				}
			}
		}
		
		if (interior_params_p->stuTrack_stand[i].flag_Stand != STATE_STUTRACK_STANDUP_FLAG)
		{
			clock_t _time = clock() - interior_params_p->stuTrack_stand[i].current_tClock;
			if (_time > THRESHOLD_STURECK_STANDTIME_DELETE_TIME)				//删除非站立roi
			{
				interior_params_p->stuTrack_stand[i] = interior_params_p->stuTrack_stand[--(interior_params_p->count_trackObj_stand)];
				i--;
				continue;
			}
		}
		interior_params_p->stuTrack_stand[i].flag_matching = FALSE;
	}

	//分析移动的目标
	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{
		clock_t _time = clock() - interior_params_p->stuTrack_bigMOveObj[i].current_tClock;
		if (_time > THRESHOLD_STURECK_MOVETIME_DELETE_TIME)
		{
			//_PRINTF("delete bigMove:origin:%d,%d,current:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].current_position.x, interior_params_p->stuTrack_bigMOveObj[i].current_position.y);
			if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
			{	
				//设置停止运动的标记
				interior_params_p->result_flag |= RESULT_STUTRACK_STOPMOVE_FLAG;
			}
			//删除长时间不运动的目标
			interior_params_p->stuTrack_bigMOveObj[i] = interior_params_p->stuTrack_bigMOveObj[--(interior_params_p->count_trackObj_bigMove)];
			i--;
			continue;
		}
		if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_NULL_FLAG)
		{
			int diff_x = abs(interior_params_p->stuTrack_bigMOveObj[i].origin_position.x - interior_params_p->stuTrack_bigMOveObj[i].current_position.x);
			int diff_y = abs(interior_params_p->stuTrack_bigMOveObj[i].origin_position.y - interior_params_p->stuTrack_bigMOveObj[i].current_position.y);
			if (diff_x>interior_params_p->stuTrack_bigMOveObj[i].dis_threshold || diff_y>interior_params_p->stuTrack_bigMOveObj[i].dis_threshold)
			{
				_time = interior_params_p->stuTrack_bigMOveObj[i].current_tClock - interior_params_p->stuTrack_bigMOveObj[i].start_tClock;
				if (_time > stuTrack_moveDelayed_threshold)
				{
					_PRINTF("find Move：origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].roi.width, interior_params_p->stuTrack_bigMOveObj[i].roi.height);
					//设置移动目标的标记
					interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;
					interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove = STATE_STUTRACK_MOVE_FLAG;
				}
			}
			else 
			{
				int centre_y = interior_params_p->stuTrack_bigMOveObj[i].roi.y + interior_params_p->stuTrack_bigMOveObj[i].roi.width;
				int size_threshold = stuTrack_size_threshold[centre_y] + stuTrack_size_threshold[centre_y];
				int size_threshold2 = size_threshold + stuTrack_size_threshold[centre_y];
				if ((interior_params_p->stuTrack_bigMOveObj[i].roi.width > size_threshold && interior_params_p->stuTrack_bigMOveObj[i].roi.height > size_threshold)
					|| interior_params_p->stuTrack_bigMOveObj[i].roi.height > size_threshold2)
				{
					_PRINTF("find big：origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].roi.width, interior_params_p->stuTrack_bigMOveObj[i].roi.height);
					interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove = STATE_STUTRACK_BIG_FLAG;
					interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//设置移动目标的标记
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
		if (stuTrack_matchingSatnd_ROI(interior_params_p, interior_params_p->stuTrack_rect_arr[i]))
		{
			interior_params_p->stuTrack_rect_arr[i] = interior_params_p->stuTrack_rect_arr[--(interior_params_p->count_stuTrack_rect)];
			i--;
		}
	}
	stuTrack_analyze_ROI(interior_params_p);			//分析候选roi
}

#define DEFINTION_DRAWCOLOUR_SETTING \
	Track_Colour_t pink_colour    = colour_RGB2YUV(255,   0, 255);/*粉红*/			\
	Track_Colour_t blue_colour    = colour_RGB2YUV(  0,   0, 255);/*纯蓝*/			\
	Track_Colour_t lilac_colour   = colour_RGB2YUV(155, 155, 255);/*淡紫*/			\
	Track_Colour_t green_colour   = colour_RGB2YUV(  0, 255,   0);/*纯绿*/			\
	Track_Colour_t red_colour     = colour_RGB2YUV(255,   0,   0);/*纯红*/			\
	Track_Colour_t dullred_colour = colour_RGB2YUV(127,   0,   0);/*暗红*/			\
	Track_Colour_t yellow_colour  = colour_RGB2YUV(255, 255,   0);/*纯黄*/

static void stuTrack_drawShow_imgData(StuITRACK_InteriorParams* interior_params_p, uchar* imageData, uchar* bufferuv)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_drawShow_imgData\n");
	//画出结果
	int i = 0;
	double zoom_scale = interior_params_p->stuTrack_zoom_scale;
	Track_Rect_t rect = { 0, 0, 0, 0 };
	Track_Point_t current_position = { 0, 0 };
	Track_Point_t origin_position = { 0, 0 };

#ifdef _WIN32
	int YUV420_type = TRACK_DRAW_YUV420P;
#endif
#ifndef _WIN32
	int YUV420_type = TRACK_DRAW_YUV420SP;
#endif

	DEFINTION_DRAWCOLOUR_SETTING;
	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{
		rect.x = interior_params_p->stuTrack_bigMOveObj[i].roi.x*zoom_scale;
		rect.y = interior_params_p->stuTrack_bigMOveObj[i].roi.y*zoom_scale;
		rect.height = interior_params_p->stuTrack_bigMOveObj[i].roi.height*zoom_scale;
		rect.width = interior_params_p->stuTrack_bigMOveObj[i].roi.width*zoom_scale;
		current_position.x = interior_params_p->stuTrack_bigMOveObj[i].current_position.x*zoom_scale;
		current_position.y = interior_params_p->stuTrack_bigMOveObj[i].current_position.y*zoom_scale;
		origin_position.x = interior_params_p->stuTrack_bigMOveObj[i].origin_position.x*zoom_scale;
		origin_position.y = interior_params_p->stuTrack_bigMOveObj[i].origin_position.y*zoom_scale;
		if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
		{
			if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_MOVE_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, &interior_params_p->srcimg_size, &rect, &pink_colour, YUV420_type);
			}
			else if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_BIG_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, &interior_params_p->srcimg_size, &rect, &blue_colour, YUV420_type);
			}
			track_draw_line(imageData, bufferuv, &interior_params_p->srcimg_size, &current_position, &origin_position, &green_colour, YUV420_type);
		}
		else
		{
			track_draw_rectangle(imageData, bufferuv, &interior_params_p->srcimg_size, &rect, &lilac_colour, YUV420_type);
		}
	}

	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		rect.x = interior_params_p->stuTrack_stand[i].roi.x*zoom_scale;
		rect.y = interior_params_p->stuTrack_stand[i].roi.y*zoom_scale;
		rect.height = interior_params_p->stuTrack_stand[i].roi.height*zoom_scale;
		rect.width = interior_params_p->stuTrack_stand[i].roi.width*zoom_scale;
		if (interior_params_p->stuTrack_stand[i].flag_Stand != STATE_STUTRACK_NULL_FLAG)
		{
			if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_STANDUP_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, &interior_params_p->srcimg_size, &rect, &red_colour, YUV420_type);
			}
			else if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_SITDOWN_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, &interior_params_p->srcimg_size, &rect, &dullred_colour, YUV420_type);
			}
		}
		else
		{
			track_draw_rectangle(imageData, bufferuv, &interior_params_p->srcimg_size, &rect, &yellow_colour, YUV420_type);
		}
	}
}

static void stuTrack_reslut(StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_reslut\n");
	//填写返回结果结构体
	if (interior_params_p->result_flag != RESULT_STUTRACK_NULL_FLAG)
	{
		_PRINTF("new change！\n");
		return_params->result_flag = interior_params_p->result_flag | RESULT_STUTRACK_NEWCHANGE_FLAG;		//当前帧的变化,设置有变化的标记
		return_params->count_trackObj_stand = interior_params_p->count_trackObj_stand;		//移动目标个数
		return_params->count_trackObj_bigMove = interior_params_p->count_trackObj_bigMove;	//起立目标个数
		if (RESULT_STUTRACK_IF_MOVE(return_params->result_flag))
		{
			//发现移动目标，将最新的目标位置返回
			return_params->move_position.x = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].current_position.x;
			return_params->move_position.y = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].current_position.y;
			return_params->moveObj_size.width = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].roi.width;
			return_params->moveObj_size.height = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].roi.height;
		}
		if (RESULT_STUTRACK_IF_STANDUP(return_params->result_flag))
		{
			//发现起立目标，位置指向最新的站立区域
			return_params->stand_position.x = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].centre.x;
			return_params->stand_position.y = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].centre.y;
			return_params->standObj_size.width = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].roi.width;
			return_params->standObj_size.height = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].roi.height;
		}
	}
}

static void stuTrack_resizeCopy_matData(StuITRACK_InteriorParams* interior_params_p, uchar* srcData)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_resizeCopy_matData\n");
	int x = 0, y = 0;
	int height = interior_params_p->img_size.height;
	int width = interior_params_p->img_size.width;
	const uchar* src_p;
	uchar* dstData = interior_params_p->currMat->data.ptr;
	for (y = 0; y < height; y++)
	{
		src_p = srcData + interior_params_p->srcimg_size.width*interior_params_p->stuTrack_resize_taby[y];//
		for (x = 0; x < width; x++)
		{
			dstData[x] = src_p[interior_params_p->stuTrack_resize_tabx[x]];
		}
		dstData += interior_params_p->currMat->step;
	}
}

BOOL stuTrack_initializeTrack(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	if (inst == NULL || interior_params_p == NULL)
	{
		return FALSE;
	}
	ITC_FUNCNAME("FUNCNAME:stuTrack_initializeTrack\n");
	stuTrack_stopTrack(inst, interior_params_p);

	if (inst->systemParams.callbackmsg_func!=NULL)
		interior_params_p->callbackmsg_func = inst->systemParams.callbackmsg_func;
	else
		interior_params_p->callbackmsg_func = printf;
	interior_params_p->srcimg_size.width = 0;
	interior_params_p->srcimg_size.height = 0;

	interior_params_p->initialize_flag = FALSE;

	float size_threshold_a, size_threshold_b;
	float direct_threshold_a, direct_threshold_b;
	if (inst->clientParams.flag_setting == TRUE)
	{
		//非默认参数
		int y1 = (inst->clientParams.stuTrack_vertex[0].y + inst->clientParams.stuTrack_vertex[1].y) / 2;
		int y2 = (inst->clientParams.stuTrack_vertex[2].y + inst->clientParams.stuTrack_vertex[3].y) / 2;
		if (y1 == y2)
		{
			_PRINTF("The input parameter error:y1 == y2!\n");
			return;
		}
		int width1 = (inst->clientParams.stuTrack_stuWidth_standard[0] + inst->clientParams.stuTrack_stuWidth_standard[1]) / 2;
		int width2 = (inst->clientParams.stuTrack_stuWidth_standard[2] + inst->clientParams.stuTrack_stuWidth_standard[3]) / 2;
		size_threshold_a = ((double)(width1 - width2)) / (y1 - y2);
		size_threshold_b = width1 - size_threshold_a*y1;

		int x1 = (inst->clientParams.stuTrack_vertex[0].x + inst->clientParams.stuTrack_vertex[2].x) / 2;
		int x2 = (inst->clientParams.stuTrack_vertex[1].x + inst->clientParams.stuTrack_vertex[3].x) / 2;
		if (x1 == x2)
		{
			_PRINTF("The input parameter error:x1 == x2!\n");
			return;
		}
		int direct1 = (inst->clientParams.stuTrack_direct_standard[0] + inst->clientParams.stuTrack_direct_standard[2]) / 2;
		int direct2 = (inst->clientParams.stuTrack_direct_standard[1] + inst->clientParams.stuTrack_direct_standard[3]) / 2;
		direct_threshold_a = ((double)(direct1 - direct2)) / (x1 - x2);
		direct_threshold_b = direct1 - direct_threshold_a*direct_threshold_a;

		interior_params_p->stuTrack_debugMsg_flag			= inst->clientParams.stuTrack_debugMsg_flag;
		interior_params_p->stuTrack_Draw_flag				= inst->clientParams.stuTrack_Draw_flag;
		interior_params_p->stuTrack_move_threshold			= inst->clientParams.stuTrack_move_threshold;			//判定是移动目标的偏离阈值（相对于自身宽度的比值）
		interior_params_p->stuTrack_standCount_threshold	= inst->clientParams.stuTrack_standCount_threshold;		//判定为起立的帧数阈值
		interior_params_p->stuTrack_sitdownCount_threshold	= inst->clientParams.stuTrack_sitdownCount_threshold;	//判定为坐下的帧数阈值
		interior_params_p->stuTrack_moveDelayed_threshold	= inst->clientParams.stuTrack_moveDelayed_threshold;
		interior_params_p->stuTrack_direct_range			= inst->clientParams.stuTrack_direct_range;
		interior_params_p->img_size.width					= inst->clientParams.width;					//需要对齐
		interior_params_p->img_size.height					= inst->clientParams.height;
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

	interior_params_p->stuTrack_resize_tabx = (int *)itcAlloc(sizeof(int)* (interior_params_p->img_size.width + interior_params_p->img_size.height));
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_resize_tabx, FALSE);
	interior_params_p->stuTrack_resize_taby = interior_params_p->stuTrack_resize_tabx + inst->clientParams.width;

	memset(interior_params_p->stuTrack_stand, 0, sizeof(StuTrack_Stand_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_bigMOveObj, 0, sizeof(StuTrack_BigMoveObj_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_rect_arr, 0, sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_size_threshold, 0, sizeof(int)* interior_params_p->img_size.height);
	memset(interior_params_p->stuTrack_direct_threshold, 0, sizeof(int)* interior_params_p->img_size.width);
	memset(interior_params_p->stuTrack_resize_tabx, 0, sizeof(int)* (interior_params_p->img_size.width + interior_params_p->img_size.height));

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
	
	interior_params_p->initialize_flag = TRUE;
	return interior_params_p->initialize_flag;
}

#define THRESHOLD_STUTRACK_FRAME_DIFF	12
#define THERSHOLD_STUTRAKC_HMI_MASK		248
void stuTrack_process(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params, char* imageData, char* bufferuv)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_process\n");
	if (imageData == NULL || return_params == NULL || interior_params_p == NULL || inst == NULL )
	{
		return;
	}
	else if (interior_params_p->initialize_flag == FALSE)
	{
		_PRINTF("Failed to initialize!\n");
		return;
	}

	if (interior_params_p->srcimg_size.width == 0 || interior_params_p->srcimg_size.height == 0)																									 
	{																																																 
		if (inst->systemParams.nsrcWidth == 0 || inst->systemParams.nsrcHeight == 0)																												 
		{																																															 
			_PRINTF("The original image size error!\n");																																			 
			return;																																													 
		}																																															 
		interior_params_p->srcimg_size.width = inst->systemParams.nsrcWidth;																														 
		interior_params_p->srcimg_size.height = inst->systemParams.nsrcHeight;																														 
		interior_params_p->stuTrack_zoom_scale = ((double)inst->systemParams.nsrcWidth) / interior_params_p->img_size.width;																		 
		int x = 0, y = 0, t = 0;																																									 
		for (x = 0; x < interior_params_p->img_size.width; x++)																																		 
		{																																															 
			/*初始化缩放映射表*/																																									
			t = (interior_params_p->srcimg_size.width*x * 2 + ITC_IMIN(interior_params_p->srcimg_size.width, interior_params_p->img_size.width) - 1) / (interior_params_p->img_size.width * 2);		 
			t -= t >= interior_params_p->srcimg_size.width;																																			 
			interior_params_p->stuTrack_resize_tabx[x] = t;																																			 
		}																																															 
		for (y = 0; y < interior_params_p->img_size.height; y++)																																	 
		{																																															 
			/*初始化缩放映射表*/																																									
			t = (interior_params_p->srcimg_size.height*y * 2 + ITC_IMIN(interior_params_p->srcimg_size.height, interior_params_p->img_size.height) - 1) / (interior_params_p->img_size.height * 2);	 
			t -= t >= interior_params_p->srcimg_size.height;
			interior_params_p->stuTrack_resize_taby[y] = t;
		}
	}

	stuTrack_resizeCopy_matData(interior_params_p, (uchar*)imageData);
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

	stuTrack_drawShow_imgData(interior_params_p, (uchar*)imageData, (uchar*)bufferuv);	//绘制处理效果
	stuTrack_reslut(interior_params_p, return_params);									//填写返回结果
}

void stuTrack_stopTrack(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_stopTrack\n");
	if (inst == NULL || interior_params_p == NULL)
	{
		return;
	}

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

	if (interior_params_p->stuTrack_resize_tabx != NULL)
	{
		itcFree_(interior_params_p->stuTrack_resize_tabx);
		interior_params_p->stuTrack_resize_tabx = NULL;
		interior_params_p->stuTrack_resize_taby = NULL;
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

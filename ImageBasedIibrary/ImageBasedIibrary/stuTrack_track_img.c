#include "stuTrack_track_img.h"
#include "itcTrack_draw_img.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define STATE_STUTRACK_NULL_FLAG	0
#define STATE_STUTRACK_STANDUP_FLAG	1
#define STATE_STUTRACK_SITDOWN_FLAG	2
#define STATE_STUTRACK_MOVE_FLAG	4
#define STATE_STUTRACK_BIG_FLAG		8

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

#define EXPADN_STURECK_SITDOWN_DIRECT	20
static int stuTrack_matchingSatnd_ROI(StuITRACK_InteriorParams* interior_params_p, Track_Rect_t roi)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_matchingSatnd_ROI\n");
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);
	
	StuTrack_allState_t* stuTrack_allState = interior_params_p->stuTrack_allState;
	int count_trackObj_allState = interior_params_p->count_trackObj_allState;
	int stuTrack_direct_range = interior_params_p->stuTrack_direct_range;
	double stuTrack_move_threshold = interior_params_p->stuTrack_move_threshold;
	int *stuTrack_direct_threshold = interior_params_p->stuTrack_direct_threshold;

	int standard_direct = stuTrack_direct_threshold[roi.x + (roi.width >> 1)];
	int standard_direct_Dir = (standard_direct > ITC_180DEGREE) ? (standard_direct - ITC_180DEGREE) : (standard_direct + ITC_180DEGREE);

	int direct = 0;
	if (interior_params_p->count_trackObj_allState>0)
	{
		//进行匹配跟踪
		int min_ID = -1;
		int min_distance = INT_MAX;
		int distance = 0;
		int diff_x = 0;
		int diff_y = 0;
		int maxWidth = 0;
		int maxHeight = 0;
		int i = 0;
		for (i = 0; i < count_trackObj_allState; i++)
		{
			maxWidth = ITC_IMAX(stuTrack_allState[i].roi.width, roi.width) >> 1;
			maxHeight = ITC_IMAX(stuTrack_allState[i].roi.height, roi.height) >> 1;
			if (abs(stuTrack_allState[i].roi.width - roi.width) < maxWidth
				&& abs(stuTrack_allState[i].roi.height - roi.height) < maxHeight)
			{
				diff_x = x - stuTrack_allState[i].current_position.x;
				diff_y = y - stuTrack_allState[i].current_position.y;
				distance = diff_x * diff_x + diff_y * diff_y;
				if (min_distance>distance)
				{
					min_distance = distance;
					min_ID = i;
				}
			}
		}
		if (min_ID >= 0)
		{
			int dis_threshold = (stuTrack_allState[min_ID].roi.width * stuTrack_allState[min_ID].roi.height) >> 2;
			if (min_distance < dis_threshold)
			{
				stuTrack_allState[min_ID].count_teack++;
				stuTrack_allState[min_ID].current_tClock = gettime();
				stuTrack_allState[min_ID].current_position = itcPoint(x, y);
				//计算运动方向
				int flag_ROI = track_calculateDirect_ROI(interior_params_p->mhiMat, roi, &direct);
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
				{
					_PRINTF("matchingID:%d,flag_ROI:%d,angle：%d,standard_direct：%d,min_distance：%d,dis_threshold：%d\n", min_ID, flag_ROI, direct, standard_direct, min_distance, dis_threshold);
				}
				if (flag_ROI == 1)
				{
					//必须是偏向垂直方向的运动
					if ((stuTrack_allState[min_ID].flag_state != STATE_STUTRACK_STANDUP_FLAG)
						&& (abs(standard_direct - direct) <= stuTrack_direct_range))
					{
						stuTrack_allState[min_ID].count_up++;
					}
					else if (stuTrack_allState[min_ID].flag_state == STATE_STUTRACK_STANDUP_FLAG
						&& abs(standard_direct_Dir - direct) <= stuTrack_direct_range + EXPADN_STURECK_SITDOWN_DIRECT)
					{
						stuTrack_allState[min_ID].count_down++;
					}

					if (stuTrack_allState[min_ID].flag_state == STATE_STUTRACK_STANDUP_FLAG)
					{
						//如果已经是起立目标，那就合并区域，用于判断坐下
						track_intersect_rect(&roi, &stuTrack_allState[min_ID].roi, EXPAND_STUTRACK_INTERSECT_RECT);
					}
				}
				else if (stuTrack_allState[min_ID].flag_state == STATE_STUTRACK_STANDUP_FLAG)
				{
					//如果当前roi不是竖直方向的运动，那就计算原roi区域的运动情况
					if (track_calculateDirect_ROI(interior_params_p->mhiMat, stuTrack_allState[min_ID].roi, &direct) == 1)
					{
						if (abs(standard_direct_Dir - direct) <= stuTrack_direct_range + EXPADN_STURECK_SITDOWN_DIRECT)
						{
							stuTrack_allState[min_ID].count_down++;
						}
						track_intersect_rect(&roi, &stuTrack_allState[min_ID].roi, EXPAND_STUTRACK_INTERSECT_RECT);
					}
				}
				stuTrack_allState[min_ID].roi = roi;
				return 1;//匹配成功直接返回
			}	//	end if(min_distance < dis_threshold)
		}		//	end if(min_ID >= 0)
		if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
		{
			_PRINTF("no matching!ID:%d,roi size:%d,%d\n", i, roi.width, roi.height);
		}
	}
	
	if (interior_params_p->count_trackObj_allState < COUNT_STUTRACK_MALLOC_ELEMENT)
	{
		//add
		if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
		{
			_PRINTF("add bigMove：origin:%d,%d,size:%d,%d\n", x, y, roi.width, roi.height);
		}
		stuTrack_allState[count_trackObj_allState].count_teack = 1;
		stuTrack_allState[count_trackObj_allState].count_up = 0;
		stuTrack_allState[count_trackObj_allState].count_down = 0;
		stuTrack_allState[count_trackObj_allState].flag_state = STATE_STUTRACK_NULL_FLAG;
		stuTrack_allState[count_trackObj_allState].dis_threshold = (int)(ITC_IMIN(roi.width, roi.height) * stuTrack_move_threshold);
		stuTrack_allState[count_trackObj_allState].roi = roi;
		stuTrack_allState[count_trackObj_allState].start_tClock = stuTrack_allState[count_trackObj_allState].current_tClock = gettime();
		stuTrack_allState[count_trackObj_allState].origin_top_y = roi.y;
		stuTrack_allState[count_trackObj_allState].origin_position = stuTrack_allState[count_trackObj_allState].current_position = itcPoint(x, y);
		interior_params_p->count_trackObj_allState++;
	}
	return 1;
}

#define THRESHOLD_STURECK_MOVETIME_DELETE_TIME	1000
static void stuTrack_analyze_ROI(StuITRACK_InteriorParams* interior_params_p)
{
	StuTrack_allState_t* stuTrack_allState = interior_params_p->stuTrack_allState;

	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	int stuTrack_standCount_threshold = interior_params_p->stuTrack_standCount_threshold;
	int stuTrack_sitdownCount_threshold = interior_params_p->stuTrack_sitdownCount_threshold;
	unsigned int stuTrack_moveDelayed_threshold = (unsigned int)interior_params_p->stuTrack_moveDelayed_threshold;

	unsigned int _time;
	int i = 0;
	for (i = 0; i < interior_params_p->count_trackObj_allState; i++)
	{
		if (stuTrack_allState[i].flag_state != STATE_STUTRACK_STANDUP_FLAG)
		{
			//判断是否需要删除候选区
			_time = gettime() - stuTrack_allState[i].current_tClock;
			if (stuTrack_allState[i].flag_state != STATE_STUTRACK_STANDUP_FLAG
				&& _time > THRESHOLD_STURECK_MOVETIME_DELETE_TIME)
			{
				if (stuTrack_allState[i].flag_state == RESULT_STUTRACK_MOVE_FLAG
					|| stuTrack_allState[i].flag_state == STATE_STUTRACK_BIG_FLAG)
				{
					interior_params_p->result_flag |= RESULT_STUTRACK_STOPMOVE_FLAG;	//如果之前是运动的目标，则设置停止运动的标记
				}

				stuTrack_allState[i] = stuTrack_allState[--(interior_params_p->count_trackObj_allState)];
				i--;
				continue;
			}

			if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_3)
			{
				_PRINTF("analyzeID:%d,count_up:%d,count_down：%d,flag_state：%d\n", i, stuTrack_allState[i].count_up, stuTrack_allState[i].count_down, stuTrack_allState[i].flag_state);
			}

			if (stuTrack_allState[i].count_up>stuTrack_standCount_threshold
				&& stuTrack_allState[i].origin_top_y - stuTrack_allState[i].roi.y>(stuTrack_allState[i].roi.height >> 2)
				&& stuTrack_allState[i].roi.height>stuTrack_allState[i].roi.width)
			{
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
				{
					_PRINTF("stand up：origin:%d,%d,size:%d,%d\n", stuTrack_allState[i].current_position.x, stuTrack_allState[i].current_position.y, stuTrack_allState[i].roi.width, stuTrack_allState[i].roi.height);
				}
				//设置为起立
				stuTrack_allState[i].count_up = 0;
				stuTrack_allState[i].count_down = 0;
				stuTrack_allState[i].standUp_size = stuTrack_allState[i].roi.width*stuTrack_allState[i].roi.height;
				stuTrack_allState[i].standUp_position = stuTrack_allState[i].current_position;		//记录起立的位置
				interior_params_p->result_flag |= RESULT_STUTRACK_STANDUP_FLAG;
				stuTrack_allState[i].flag_state = STATE_STUTRACK_STANDUP_FLAG;
			}

			if (stuTrack_allState[i].flag_state == STATE_STUTRACK_NULL_FLAG || stuTrack_allState[i].flag_state == STATE_STUTRACK_SITDOWN_FLAG)
			{
				//不是起立，判断是否是移动目标
				//计算移动距离
				int diff_x = abs(stuTrack_allState[i].origin_position.x - stuTrack_allState[i].current_position.x);
				int diff_y = abs(stuTrack_allState[i].origin_position.y - stuTrack_allState[i].current_position.y);
				if (diff_x > stuTrack_allState[i].dis_threshold || diff_y > stuTrack_allState[i].dis_threshold)
				{
					_time = stuTrack_allState[i].current_tClock - stuTrack_allState[i].start_tClock;
					if (_time > stuTrack_moveDelayed_threshold)
					{
						if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
						{
							_PRINTF("find Move：origin:%d,%d,size:%d,%d\n", stuTrack_allState[i].origin_position.x, stuTrack_allState[i].origin_position.y, stuTrack_allState[i].roi.width, stuTrack_allState[i].roi.height);
						}
						interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//设置移动目标的标记
						stuTrack_allState[i].flag_state = STATE_STUTRACK_MOVE_FLAG;
					}
				}
				else
				{
					//大目标
					int centre_y = stuTrack_allState[i].roi.y + stuTrack_allState[i].roi.height;
					int size_threshold = stuTrack_size_threshold[centre_y] + stuTrack_size_threshold[centre_y];
					if (stuTrack_allState[i].roi.width > size_threshold && stuTrack_allState[i].roi.height > size_threshold)
					{
						if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
						{
							_PRINTF("find big：origin:%d,%d,size:%d,%d\n", stuTrack_allState[i].origin_position.x, stuTrack_allState[i].origin_position.y, stuTrack_allState[i].roi.width, stuTrack_allState[i].roi.height);
						}
						interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//设置移动目标的标记
						stuTrack_allState[i].flag_state = STATE_STUTRACK_BIG_FLAG;
					}
				}
			}	//end if (stuTrack_allState[i].flag_state == STATE_STUTRACK_NULL_FLAG || stuTrack_allState[i].flag_state == STATE_STUTRACK_SITDOWN_FLAG)
		}
		else //if (stuTrack_allState[i].flag_state != STATE_STUTRACK_STANDUP_FLAG)
		{
			//如果该区域已经起立，判断是否坐下
			if (stuTrack_allState[i].count_down > stuTrack_sitdownCount_threshold)
			{
				if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
				{
					_PRINTF("sit down：origin:%d,%d,size:%d,%d\n", stuTrack_allState[i].current_position.x, stuTrack_allState[i].current_position.y, stuTrack_allState[i].roi.width, stuTrack_allState[i].roi.height);
				}
				stuTrack_allState[i].count_up = 0;
				stuTrack_allState[i].count_down = 0;
				interior_params_p->result_flag |= RESULT_STUTRACK_SITDOWN_FLAG;
				stuTrack_allState[i].flag_state = STATE_STUTRACK_SITDOWN_FLAG;
			}
			else
			{
				//没有坐下，判断是否移动走了
				int size_threshold = stuTrack_allState[i].standUp_size;
				int diff_x = stuTrack_allState[i].standUp_position.x - stuTrack_allState[i].current_position.x;
				int diff_y = stuTrack_allState[i].standUp_position.y - stuTrack_allState[i].current_position.y;
				diff_x *= diff_x;
				diff_y *= diff_y;
				if (diff_x > (size_threshold >> 2) || diff_y > (size_threshold >> 2))
				{
					if (interior_params_p->stuTrack_debugMsg_flag >= SATUTRACK_PRINTF_LEVEL_2)
					{
						_PRINTF("stand to bigMove：origin:%d,%d,size:%d,%d\n", stuTrack_allState[i].current_position.x, stuTrack_allState[i].current_position.y, stuTrack_allState[i].roi.width, stuTrack_allState[i].roi.height);
					}
					//变成移动目标
					stuTrack_allState[i].count_up = 0;
					stuTrack_allState[i].count_down = 0;
					interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//设置移动目标的标记
					stuTrack_allState[i].flag_state = STATE_STUTRACK_MOVE_FLAG;
				}
			}
		} //end if(stuTrack_allState[i].flag_state != STATE_STUTRACK_STANDUP_FLAG)
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
	StuTrack_allState_t* stuTrack_allState = interior_params_p->stuTrack_allState;

	Track_Rect_t *rect;
	Track_Point_t *current_position;
	Track_Point_t *origin_position;

#ifdef _WIN32
	int YUV420_type = TRACK_DRAW_YUV420P;
#else
	int YUV420_type = TRACK_DRAW_YUV420SP;
#endif
	for (i = 0; i < interior_params_p->count_trackObj_allState; i++)
	{
		current_position = &(stuTrack_allState[i].current_position);
		origin_position = &(stuTrack_allState[i].origin_position);
		rect = &(stuTrack_allState[i].roi);
		if (stuTrack_allState[i].flag_state != STATE_STUTRACK_NULL_FLAG)
		{
			if (stuTrack_allState[i].flag_state == STATE_STUTRACK_STANDUP_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->red_colour, YUV420_type);
			}
			else if (stuTrack_allState[i].flag_state == STATE_STUTRACK_SITDOWN_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->yellow_colour, YUV420_type);
			}
			else if (stuTrack_allState[i].flag_state == STATE_STUTRACK_MOVE_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->pink_colour, YUV420_type);
			}
			else if (stuTrack_allState[i].flag_state == STATE_STUTRACK_BIG_FLAG)
			{
				track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->blue_colour, YUV420_type);
			}
			track_draw_point(imageData, bufferuv, srcimg_size, origin_position,&interior_params_p->red_colour, YUV420_type);
			track_draw_line(imageData, bufferuv, srcimg_size, origin_position, current_position, &interior_params_p->green_colour, YUV420_type);
		}
		else
		{
			track_draw_rectangle(imageData, bufferuv, srcimg_size, rect, &interior_params_p->lilac_colour, YUV420_type);
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

static stuTrackReturn stuTrack_reslut(StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params)
{
	ITC_FUNCNAME("FUNCNAME:stuTrack_reslut\n");
	stuTrackReturn flag_return = RETURN_STUTRACK_noNEED_PROCESS;
	//填写返回结果结构体
	if (interior_params_p->result_flag != RESULT_STUTRACK_NULL_FLAG)
	{
		flag_return = RETURN_STUTRACK_NEED_PROCESS;
		return_params->result_flag = interior_params_p->result_flag | RESULT_STUTRACK_NEWCHANGE_FLAG;		//当前帧的变化,设置有变化的标记
	}

	return_params->count_trackObj_stand = 0;		//移动目标个数
	return_params->count_trackObj_bigMove = 0;		//起立目标个数
	int i = 0;
	int id_stuUp = 0;
	int id_stuMove = 0;
	StuTrack_allState_t* stuTrack_allState = interior_params_p->stuTrack_allState;
	for (i = 0; i < interior_params_p->count_trackObj_allState; i++)
	{
		if (stuTrack_allState[i].flag_state != STATE_STUTRACK_NULL_FLAG)
		{
			if (stuTrack_allState[i].flag_state == STATE_STUTRACK_STANDUP_FLAG)
			{
				id_stuUp = i;
				return_params->count_trackObj_stand++;
			}
			else if (stuTrack_allState[i].flag_state == STATE_STUTRACK_MOVE_FLAG || stuTrack_allState[i].flag_state == STATE_STUTRACK_BIG_FLAG)
			{
				id_stuMove = i;
				return_params->count_trackObj_bigMove++;
			}
		}
	}
	
	if (return_params->count_trackObj_stand > 0)
	{
		flag_return = RETURN_STUTRACK_NEED_PROCESS;
		Track_Point_t pt = { 0, 0 };
		perspectiveConvert(&(interior_params_p->stuTrack_allState[id_stuUp].current_position), &pt, interior_params_p->transformationMatrix);
		return_params->stand_position.x = pt.x;
		return_params->stand_position.y = pt.y;
		return_params->standObj_size.width = interior_params_p->stuTrack_allState[id_stuUp].roi.width;
		return_params->standObj_size.height = interior_params_p->stuTrack_allState[id_stuUp].roi.height;

		int size = ITC_IMAX(return_params->standObj_size.width, return_params->standObj_size.height);
		return_params->stretchingCoefficient_stand = (int)(interior_params_p->stretchingAB[0] * size + interior_params_p->stretchingAB[1]);
		return_params->stretchingCoefficient_stand = return_params->stretchingCoefficient_stand > 0 ? return_params->stretchingCoefficient_stand : 0;
	}
	if (return_params->count_trackObj_bigMove > 0)
	{
		flag_return = RETURN_STUTRACK_NEED_PROCESS;
		Track_Point_t pt = { 0, 0 };
		perspectiveConvert(&(interior_params_p->stuTrack_allState[id_stuMove].current_position), &pt, interior_params_p->transformationMatrix);
		return_params->move_position.x = pt.x;
		return_params->move_position.y = pt.y;
		return_params->moveObj_size.width = interior_params_p->stuTrack_allState[id_stuMove].roi.width;
		return_params->moveObj_size.height = interior_params_p->stuTrack_allState[id_stuMove].roi.height;

		int size = ITC_IMAX(return_params->moveObj_size.width, return_params->moveObj_size.height);
		return_params->stretchingCoefficient_bigMove = (int)(interior_params_p->stretchingAB[0] * size + interior_params_p->stretchingAB[1]);
		return_params->stretchingCoefficient_bigMove = return_params->stretchingCoefficient_bigMove > 0 ? return_params->stretchingCoefficient_bigMove : 0;
	}
	return flag_return;
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
		interior_params_p->transformationMatrix->data.db[4] = -1.0;
		interior_params_p->transformationMatrix->data.db[5] = 0.0;
		interior_params_p->transformationMatrix->data.db[6] = 0.0;
		interior_params_p->transformationMatrix->data.db[7] = 0.0;
		interior_params_p->transformationMatrix->data.db[8] = 1.0;
		interior_params_p->stretchingAB[0] = -1.0;
		interior_params_p->stretchingAB[1] = 0.0;
	}

	//初始化自有的内部统计参数
	interior_params_p->_count = 0;
	interior_params_p->count_stuTrack_rect = 0;
	interior_params_p->count_trackObj_allState = 0;

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

	interior_params_p->stuTrack_rect_arr = (Track_Rect_t*)itcAlloc(sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_rect_arr, FALSE);

	interior_params_p->stuTrack_allState = (StuTrack_allState_t*)itcAlloc(sizeof(StuTrack_allState_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_allState, FALSE);

	interior_params_p->stuTrack_size_threshold = (int *)itcAlloc(sizeof(int)* interior_params_p->img_size.height);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_size_threshold, FALSE);

	interior_params_p->stuTrack_direct_threshold = (int *)itcAlloc(sizeof(int)* interior_params_p->img_size.width);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_direct_threshold, FALSE);

	memset(interior_params_p->stuTrack_rect_arr, 0, sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_allState, 0, sizeof(StuTrack_allState_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
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
stuTrackReturn stuTrack_process(const StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params, char* imageData, char* bufferuv)
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

	return stuTrack_reslut(interior_params_p, return_params);									//填写跟踪结果
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

	if (interior_params_p->stuTrack_rect_arr != NULL)
	{
		itcFree_(interior_params_p->stuTrack_rect_arr);
		interior_params_p->stuTrack_rect_arr = NULL;
	}

	if (interior_params_p->stuTrack_allState != NULL)
	{
		itcFree_(interior_params_p->stuTrack_allState);
		interior_params_p->stuTrack_allState = NULL;
	}

	interior_params_p->count_stuTrack_rect = 0;
	interior_params_p->count_trackObj_allState = 0;

	itcReleaseMemStorage(&interior_params_p->stuTrack_storage);
	itc_release_mat(&interior_params_p->currMat);
	itc_release_mat(&interior_params_p->lastMat);
	itc_release_mat(&interior_params_p->mhiMat);
	itc_release_mat(&interior_params_p->maskMat);
	interior_params_p->initialize_flag = FALSE;
}

#include "stuTrack_track_img.h"
#include "itcTrack_draw_img.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <string.h>

#define STATE_STUTRACK_NULL_FLAG	0
#define STATE_STUTRACK_STANDUP_FLAG	1
#define STATE_STUTRACK_SITDOWN_FLAG	(-1)
#define STATE_STUTRACK_MOVE_FLAG	1
#define STATE_STUTRACK_BIG_FLAG		2

#define EXPAND_STUTRACK_INTERSECT_RECT (-3)
static void stuTrack_filtrate_contours(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p,Track_Contour_t** pContour)
{
	//����ɸѡ
	if (*pContour == NULL || inst == NULL || interior_params_p == NULL)
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
			count_rect < COUNT_STUTRACK_MALLOC_ELEMENT)					//ɸѡ
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
			if (track_intersect_rect(stuTrack_rect_arr + i, stuTrack_rect_arr + j, EXPAND_STUTRACK_INTERSECT_RECT))	//�ж��Ƿ��ཻ������ཻ��ֱ�Ӻϲ�
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
static int stuTrack_matchingSatnd_ROI(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, Track_Rect_t roi)
{
	//ƥ��roi
	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	int *stuTrack_direct_threshold = interior_params_p->stuTrack_direct_threshold;

	int stuTrack_direct_range = inst->stuTrack_direct_range;
	float stuTrack_move_threshold = inst->stuTrack_move_threshold;

	int standard_direct = stuTrack_direct_threshold[roi.x + (roi.width >> 1)];
	int direct = 0;
	//���ĵ�
	int x = roi.x + (roi.width >> 1);
	int y = roi.y + (roi.height >> 1);
	unsigned int i = 0;
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
				//_PRINTF("�Ƕȣ�ԭ�Ƕ�:%d,��ǰ�Ƕ�:%d����Χ:%d\n", interior_params_p->stuTrack_stand[min_ID].direction, direct, stuTrack_direct_range);
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
			//_PRINTF("add stand��origin:%d,%d,size:%d,%d\n", x, y, roi.width, roi.height);
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
					//�˴����Ż�
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
				//_PRINTF("add bigMove��origin:%d,%d,size:%d,%d\n", x, y, roi.width, roi.height);
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
static BOOL stuTrack_judgeStand_ROI(StuITRACK_Params *inst, StuTrack_Stand_t track_stand)
{
	//�ж��Ƿ�����
	unsigned int stuTrack_standCount_threshold = inst->stuTrack_standCount_threshold;
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
static void stuTrack_analyze_ROI(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	//������ѡ����
	int *stuTrack_size_threshold = interior_params_p->stuTrack_size_threshold;
	int *stuTrack_direct_threshold = interior_params_p->stuTrack_direct_threshold;

	unsigned int stuTrack_direct_range = inst->stuTrack_direct_range;
	unsigned int stuTrack_sitdownCount_threshold = inst->stuTrack_sitdownCount_threshold;
	unsigned int stuTrack_moveDelayed_threshold = inst->stuTrack_moveDelayed_threshold;

	Itc_Mat_t *mhi = (Itc_Mat_t *)interior_params_p->mhiMat;

	int direct = 0;
	int standard_direct = 0;
	int flag_ROI = 0;
	unsigned int i = 0;
	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (interior_params_p->stuTrack_stand[i].flag_Stand != STATE_STUTRACK_STANDUP_FLAG)
		{
			//�����û������
			if (interior_params_p->stuTrack_stand[i].flag_matching == FALSE)
			{
				standard_direct = stuTrack_direct_threshold[interior_params_p->stuTrack_stand[i].centre.x];
				flag_ROI = track_calculateDirect_ROI(mhi, interior_params_p->stuTrack_stand[i].roi, &direct);
				if ((flag_ROI == 1) && (((unsigned int)(abs(interior_params_p->stuTrack_stand[i].direction - direct))) < stuTrack_direct_range))
				{
					interior_params_p->stuTrack_stand[i].count_up++;
				}
			}
			//_PRINTF("�жϣ�%d, %d\n", interior_params_p->stuTrack_stand[i].count_teack, interior_params_p->stuTrack_stand[i].count_up);
			if (stuTrack_judgeStand_ROI(inst, interior_params_p->stuTrack_stand[i]))	//ȷ���Ƿ�վ��
			{
				_PRINTF("stand up��origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_stand[i].centre.x, interior_params_p->stuTrack_stand[i].centre.y, interior_params_p->stuTrack_stand[i].roi.width, interior_params_p->stuTrack_stand[i].roi.height);
				//���������ı��
				interior_params_p->result_flag |= RESULT_STUTRACK_STANDUP_FLAG;
				interior_params_p->stuTrack_stand[i].flag_Stand = STATE_STUTRACK_STANDUP_FLAG;
			}
		}
		else
		{
			//�����û������
			standard_direct = stuTrack_direct_threshold[interior_params_p->stuTrack_stand[i].centre.x];
			standard_direct = (standard_direct > ITC_DEGREES) ? (standard_direct - ITC_DEGREES) : (standard_direct + ITC_DEGREES);		//���������������෴�ĽǶ�
			flag_ROI = track_calculateDirect_ROI(mhi, interior_params_p->stuTrack_stand[i].roi, &direct);
			if ((flag_ROI == 1) && (((unsigned int)(abs(standard_direct - direct)))< stuTrack_direct_range + EXPADN_STURECK_SITDOWN_DIRECT))
			{
				interior_params_p->stuTrack_stand[i].count_down++;
				if (interior_params_p->stuTrack_stand[i].count_down>stuTrack_sitdownCount_threshold)
				{
					_PRINTF("sit down��origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_stand[i].centre.x, interior_params_p->stuTrack_stand[i].centre.y, interior_params_p->stuTrack_stand[i].roi.width, interior_params_p->stuTrack_stand[i].roi.height);
					//�������µı��
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
			if (_time > THRESHOLD_STURECK_STANDTIME_DELETE_TIME)				//ɾ����վ��roi
			{
				interior_params_p->stuTrack_stand[i] = interior_params_p->stuTrack_stand[--(interior_params_p->count_trackObj_stand)];
				i--;
				continue;
			}
		}
		interior_params_p->stuTrack_stand[i].flag_matching = FALSE;
	}

	//�����ƶ���Ŀ��
	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{
		clock_t _time = clock() - interior_params_p->stuTrack_bigMOveObj[i].current_tClock;
		if (_time > THRESHOLD_STURECK_MOVETIME_DELETE_TIME)
		{
			//_PRINTF("delete bigMove:origin:%d,%d,current:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].current_position.x, interior_params_p->stuTrack_bigMOveObj[i].current_position.y);
			if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
			{	
				//����ֹͣ�˶��ı��
				interior_params_p->result_flag |= RESULT_STUTRACK_STOPMOVE_FLAG;
			}
			//ɾ����ʱ�䲻�˶���Ŀ��
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
				if (((unsigned int)_time) > stuTrack_moveDelayed_threshold)
				{
					//_PRINTF("find Move��origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].roi.width, interior_params_p->stuTrack_bigMOveObj[i].roi.height);
					//�����ƶ�Ŀ��ı��
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
					//_PRINTF("find big��origin:%d,%d,size:%d,%d\n", interior_params_p->stuTrack_bigMOveObj[i].origin_position.x, interior_params_p->stuTrack_bigMOveObj[i].origin_position.y, interior_params_p->stuTrack_bigMOveObj[i].roi.width, interior_params_p->stuTrack_bigMOveObj[i].roi.height);
					interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove = STATE_STUTRACK_BIG_FLAG;
					interior_params_p->result_flag |= RESULT_STUTRACK_MOVE_FLAG;//�����ƶ�Ŀ��ı��
				}
			}
		}
	}
}

static void stuTrack_proStandDown_ROI(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	//ƥ��ͷ�����ѡ����
	unsigned int i = 0;
	for (i = 0; i < interior_params_p->count_stuTrack_rect; i++)
	{
		if (stuTrack_matchingSatnd_ROI(inst, interior_params_p, interior_params_p->stuTrack_rect_arr[i]))
		{
			interior_params_p->stuTrack_rect_arr[i] = interior_params_p->stuTrack_rect_arr[--(interior_params_p->count_stuTrack_rect)];
			i--;
		}
	}
	stuTrack_analyze_ROI(inst, interior_params_p);			//������ѡroi
}

#define DEFINTION_DRAWCOLOUR_SETTING \
Trcak_Colour_t pink_colour		= colour_RGB2YUV(255,   0, 255);/*�ۺ�*/			\
Trcak_Colour_t blue_colour		= colour_RGB2YUV(  0,   0, 255);/*����*/			\
Trcak_Colour_t lilac_colour		= colour_RGB2YUV(155, 155, 255);/*����*/			\
Trcak_Colour_t green_colour		= colour_RGB2YUV(  0, 255,   0);/*����*/			\
Trcak_Colour_t red_colour		= colour_RGB2YUV(255,   0,   0);/*����*/			\
Trcak_Colour_t dullred_colour	= colour_RGB2YUV(127,   0,   0);/*����*/			\
Trcak_Colour_t yellow_colour	= colour_RGB2YUV(255, 255,   0);/*����*/

static void stuTrack_drawShow_imgData(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, char* imageData)
{
	//�������
	unsigned int i = 0;
	Track_Size_t img_size = { inst->width, inst->height };
	DEFINTION_DRAWCOLOUR_SETTING;
	for (i = 0; i < interior_params_p->count_trackObj_bigMove; i++)
	{

		if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove != STATE_STUTRACK_NULL_FLAG)
		{
			if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_MOVE_FLAG)
			{
				track_draw_rectangle(imageData, &img_size, &(interior_params_p->stuTrack_bigMOveObj[i].roi), &pink_colour);
			}
			else if (interior_params_p->stuTrack_bigMOveObj[i].flag_bigMove == STATE_STUTRACK_BIG_FLAG)
			{
				track_draw_rectangle(imageData, &img_size, &(interior_params_p->stuTrack_bigMOveObj[i].roi), &blue_colour);
			}
			track_draw_line(imageData, &img_size, &(interior_params_p->stuTrack_bigMOveObj[i].current_position), &(interior_params_p->stuTrack_bigMOveObj[i].origin_position), &green_colour);
		}
		else
		{
			track_draw_rectangle(imageData, &img_size, &(interior_params_p->stuTrack_bigMOveObj[i].roi), &lilac_colour);
		}
	}

	for (i = 0; i < interior_params_p->count_trackObj_stand; i++)
	{
		if (interior_params_p->stuTrack_stand[i].flag_Stand != STATE_STUTRACK_NULL_FLAG)
		{
			if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_STANDUP_FLAG)
			{
				track_draw_rectangle(imageData, &img_size, &(interior_params_p->stuTrack_stand[i].roi), &red_colour);
			}
			else if (interior_params_p->stuTrack_stand[i].flag_Stand == STATE_STUTRACK_SITDOWN_FLAG)
			{
				track_draw_rectangle(imageData, &img_size, &(interior_params_p->stuTrack_stand[i].roi), &dullred_colour);
			}
		}
		else
		{
			track_draw_rectangle(imageData, &img_size, &(interior_params_p->stuTrack_stand[i].roi), &yellow_colour);
		}
	}
}

static void stuTrack_reslut(StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params)
{
	//��д���ؽ���ṹ��
	if (interior_params_p->result_flag != RESULT_STUTRACK_NULL_FLAG)
	{
		_PRINTF("new change��\n");
		return_params->result_flag |= RESULT_STUTRACK_NEWCHANGE_FLAG;
		return_params->result_flag = interior_params_p->result_flag;						//��ǰ֡�ı仯
		return_params->count_trackObj_stand = interior_params_p->count_trackObj_stand;		//�ƶ�Ŀ�����
		return_params->count_trackObj_bigMove = interior_params_p->count_trackObj_bigMove;	//����Ŀ�����
		if (RESULT_STUTRACK_IF_MOVE(return_params->result_flag))
		{
			//�����ƶ�Ŀ�꣬�����µ�Ŀ��λ�÷���
			return_params->move_position.x = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].current_position.x;
			return_params->move_position.y = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].current_position.y;
			return_params->moveObj_size.width = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].roi.width;
			return_params->moveObj_size.height = interior_params_p->stuTrack_bigMOveObj[interior_params_p->count_trackObj_bigMove - 1].roi.height;
		}
		if (RESULT_STUTRACK_IF_STANDUP(return_params->result_flag))
		{
			//��������Ŀ�꣬λ��ָ�����µ�վ������
			return_params->stand_position.x = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].centre.x;
			return_params->stand_position.y = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].centre.y;
			return_params->standObj_size.width = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].roi.width;
			return_params->standObj_size.height = interior_params_p->stuTrack_stand[interior_params_p->count_trackObj_stand - 1].roi.height;
		}
	}
}

void stuTrack_initializeTrack(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	if (inst == NULL || interior_params_p == NULL)
	{
		interior_params_p->initialize_flag = FALSE;
		return;
	}
	stuTrack_stopTrack(inst, interior_params_p);
	interior_params_p->initialize_flag = FALSE;
	if (inst->height != HEIGHT_STUTRACK_IMG_ || inst->width != WIDTH_STUTRACK_IMG_)
	{
		return;
	}
	//�����ڴ�
	interior_params_p->currMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->currMat, ITC_RETURN);

	interior_params_p->lastMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->lastMat, ITC_RETURN);

	interior_params_p->mhiMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->mhiMat, ITC_RETURN);

	interior_params_p->maskMat = itc_create_mat(inst->height, inst->width, ITC_8UC1);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->maskMat, ITC_RETURN);

	interior_params_p->stuTrack_storage = itcCreateMemStorage(0);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_storage, ITC_RETURN);

	interior_params_p->stuTrack_stand = (StuTrack_Stand_t*)malloc(sizeof(StuTrack_Stand_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_stand, ITC_RETURN);

	interior_params_p->stuTrack_bigMOveObj = (StuTrack_BigMoveObj_t*)malloc(sizeof(StuTrack_BigMoveObj_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_bigMOveObj, ITC_RETURN);

	interior_params_p->stuTrack_rect_arr = (Track_Rect_t*)malloc(sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_rect_arr, ITC_RETURN);

	interior_params_p->stuTrack_size_threshold = (int *)malloc(sizeof(int)* inst->height);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_size_threshold, ITC_RETURN);

	interior_params_p->stuTrack_direct_threshold = (int *)malloc(sizeof(int)* inst->width);
	JUDEGE_STUREACK_IF_NULL(interior_params_p->stuTrack_direct_threshold, ITC_RETURN);

	//��ʼ�����е��ڲ�����
	interior_params_p->_count = 0;
	interior_params_p->img_size = inst->height*inst->width;
	interior_params_p->count_trackObj_stand = 0;
	interior_params_p->count_trackObj_bigMove = 0;
	interior_params_p->count_stuTrack_rect = 0;
	memset(interior_params_p->stuTrack_stand, 0, sizeof(StuTrack_Stand_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_bigMOveObj, 0, sizeof(StuTrack_BigMoveObj_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_rect_arr, 0, sizeof(Track_Rect_t)* COUNT_STUTRACK_MALLOC_ELEMENT);
	memset(interior_params_p->stuTrack_size_threshold, 0, sizeof(int)* inst->height);
	memset(interior_params_p->stuTrack_direct_threshold, 0, sizeof(int)* inst->width);
	
	//Ĭ�ϵĲ���
	float size_threshold_a = A_STUTRACK_SIZE_THRESHOLD_PARAMS;
	float size_threshold_b = B_STUTRACK_SIZE_THRESHOLD_PARAMS;
	float direct_threshold_a = A_STUTRACK_DIRECT_THRESHOLD_PARAMS;
	float direct_threshold_b = B_STUTRACK_DIRECT_THRESHOLD_PARAMS;
	inst->stuTrack_move_threshold = THRESHOLD_STUTRACK_MOVE_DEFALUT_PARAMS;			//�ж����ƶ�Ŀ���ƫ����ֵ����ֵ��
	inst->stuTrack_standCount_threshold = THRESHOLD_STUTRACK_STANDCOUNT_DEFALUT_PARAMS;		//�ж�Ϊ������֡����ֵ
	inst->stuTrack_sitdownCount_threshold = THRESHOLD_STUTRACK_SITDOWNCOUNT_DEFALUT_PARAMS;	//�ж�Ϊ���µ�֡����ֵ
	inst->stuTrack_moveDelayed_threshold = THRESHOLD_STUTRACK_MOVEDELAYED_DEFALUT_PARAMS;
	inst->stuTrack_direct_range = RANGE_STUTRACK_STANDDIRECT_DEFALUT_PARAMS;

	if (inst->flag_setting == TRUE)
	{
		//��Ĭ�ϲ���
	}

	int i = 0;
	for (i = 0; i < inst->height; i++)
	{
		interior_params_p->stuTrack_size_threshold[i] = (int)(COMPUTER_STUTRACK_SIZE_THRESHOLD_PARAMS(i, size_threshold_a, size_threshold_b) + 0.5);
	}
	for (i = 0; i < inst->width; i++)
	{
		interior_params_p->stuTrack_direct_threshold[i] = (int)(COMPUTER_STUTRACK_DIRECT_THRESHOLD_PARAMS(i, direct_threshold_a, direct_threshold_b) + 0.5);
	}

	interior_params_p->initialize_flag = TRUE;
}

#define THRESHOLD_STUTRACK_FRAME_DIFF	12
#define THERSHOLD_STUTRAKC_HMI_MASK		248
void stuTrack_process(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params, char* imageData)
{
	if (imageData == NULL || return_params == NULL || interior_params_p == NULL || inst == NULL 
		|| interior_params_p->initialize_flag == FALSE)
	{	
		return;
	}
	memcpy(interior_params_p->currMat->data.ptr, imageData, interior_params_p->img_size);

	interior_params_p->result_flag = RESULT_STUTRACK_NULL_FLAG;	//��ձ仯״̬
	Track_Contour_t* firstContour = NULL;
	if (interior_params_p->_count>1)
	{
		itcClearMemStorage(interior_params_p->stuTrack_storage);
		track_update_MHI(interior_params_p->currMat, interior_params_p->lastMat, interior_params_p->mhiMat, THRESHOLD_STUTRACK_FRAME_DIFF, interior_params_p->maskMat, THERSHOLD_STUTRAKC_HMI_MASK);
		track_find_contours(interior_params_p->maskMat, &firstContour, interior_params_p->stuTrack_storage);
		stuTrack_filtrate_contours(inst, interior_params_p,&firstContour);
		stuTrack_proStandDown_ROI(inst, interior_params_p);
	}
	interior_params_p->tempMat = interior_params_p->currMat;
	interior_params_p->currMat = interior_params_p->lastMat;
	interior_params_p->lastMat = interior_params_p->tempMat;
	interior_params_p->_count++;

	stuTrack_drawShow_imgData(inst,interior_params_p,imageData);	//���ƴ���Ч��
	stuTrack_reslut(interior_params_p, return_params);				//��д���ؽ��
}

void stuTrack_stopTrack(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p)
{
	if (inst == NULL || interior_params_p == NULL)
	{
		return;
	}

	if (interior_params_p->stuTrack_size_threshold != NULL)
	{
		free(interior_params_p->stuTrack_size_threshold);
		interior_params_p->stuTrack_size_threshold = NULL;
	}

	if (interior_params_p->stuTrack_direct_threshold != NULL)
	{
		free(interior_params_p->stuTrack_direct_threshold);
		interior_params_p->stuTrack_direct_threshold = NULL;
	}

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
}

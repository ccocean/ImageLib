#ifndef stuTrack_track_img_h__
#define stuTrack_track_img_h__

#include "itctype.h"
#include "itcerror.h"
#include "itcdatastructs.h"
#include "itcCore.h"
#include "stuTrack_settings_parameter.h"

#ifdef  __cplusplus
extern "C" {
#endif

#define COUNT_STUTRACK_MALLOC_ELEMENT 10

#define SATUTRACK_PRINTF_LEVEL_1 1
#define SATUTRACK_PRINTF_LEVEL_2 2
#define SATUTRACK_PRINTF_LEVEL_3 3
#define SATUTRACK_PRINTF_LEVEL_4 4

#define _PRINTF											\
	if (interior_params_p->callbackmsg_func == NULL)		\
	{	interior_params_p->callbackmsg_func = printf; }		\
	((callbackmsg)(interior_params_p->callbackmsg_func))

#define ITC_RETURN
#define JUDEGE_STUREACK_IF_NULL(p,r)			\
if ((p) == NULL)								\
{												\
	_PRINTF("Memory allocation error!\n");		\
	stuTrack_stopTrack(inst, interior_params_p);\
	return r;									\
}

#define ITC_FUNCNAME(name) \
	if ((interior_params_p->stuTrack_debugMsg_flag) >= SATUTRACK_PRINTF_LEVEL_4)\
	{																			\
		_PRINTF(name);															\
	}


typedef int(*_callbackmsg)(const char *format, ...);//用于输出调试信息的函数指针

typedef struct StuTrack_Stand_t
{
	int direction;
	int count_teack;	//
	int count_up;		//
	int count_down;		//
	int flag_Stand;		//起立标志
	int flag_matching;	//匹配标志
	Track_Point_t centre;
	Track_Rect_t roi;
	int origin_y;
	unsigned long start_tClock;
	unsigned long current_tClock;
}StuTrack_Stand_t;

typedef struct StuTrack_BigMoveObj_t
{
	int count_track;
	int flag_bigMove;		//标志
	int dis_threshold;		//认为是移动目标的阈值
	Track_Rect_t roi;
	unsigned long start_tClock;
	unsigned long current_tClock;
	Track_Point_t origin_position;
	Track_Point_t current_position;
}StuTrack_BigMoveObj_t;

typedef struct StuTrack_allState_t
{
	int flag_state;			//状态标志
	int flag_matching;		//匹配标志
	int count_teack;		//
	int count_up;			//
	int count_down;			//
	int dis_threshold;		//认为是移动目标的阈值
	Track_Rect_t roi;
	unsigned long start_tClock;
	unsigned long current_tClock;
	int origin_top_y;
	Track_Point_t origin_position;
	Track_Point_t current_position;
	Track_Point_t standUp_position;
	int standUp_size;
}StuTrack_allState_t;

typedef struct _StuITRACK_InteriorParams
{
	itc_BOOL initialize_flag;
	unsigned int _count;		//统计帧数
	Track_Size_t img_size;		//处理图像大小
	Track_Size_t srcimg_size;	//原始图像大小

	int result_flag;					//当前帧变化状态

	int count_stuTrack_rect;			//运动区域计数
	Track_Rect_t *stuTrack_rect_arr;

	int count_trackObj_allState;		//跟踪的目标数
	StuTrack_allState_t* stuTrack_allState;

	int stuTrack_debugMsg_flag;					//调试信息输出等级
	int stuTrack_Draw_flag;						//是否绘制结果
	int stuTrack_direct_range;					//起立时允许的角度偏离范围
	int stuTrack_standCount_threshold;			//判定为起立的帧数阈值
	int stuTrack_sitdownCount_threshold;		//判定为坐下的帧数阈值
	int stuTrack_moveDelayed_threshold;			//移动目标保持跟踪的延时，超过这个时间无运动，则放弃跟踪(单位：毫秒)
	double stuTrack_move_threshold;				//判定是移动目标的偏离阈值（比值）
	int *stuTrack_size_threshold;				//运动目标大小过滤阈值（根据位置不同阈值不同）
	int *stuTrack_direct_threshold;				//起立的标准角度,大小为width

	Itc_Mat_t *transformationMatrix;				//图像坐标与云台相机的变换矩阵
	double stretchingAB[2];							//拉伸系数

	unsigned int last_upcast_time;					//记录上一次抛出状态信息的时间
	unsigned int move_camera_time;					//上一次移动相机的时间
	int old_move_Stopflag;							//是否有移动目标停止动作标记
	int OldResult_flag;								//上一次的返回结果
	int move_csucceed_flag;							//要移动镜头时设置该标记为false，成功发送移动信号后设置true

	TrackPrarms_Point_t old_move_position;			//移动目标保存的镜头位置
	int old_move_stretchingCoefficient;				//移动目标保存的拉伸系数
	TrackPrarms_Point_t old_standup_position;		//起立保存的镜头位置
	int old_standup_stretchingCoefficient;			//移动目标保存的拉伸系数

	Itc_Mat_t *tempMat;
	Itc_Mat_t *currMat;
	Itc_Mat_t *lastMat;
	Itc_Mat_t *mhiMat;
	Itc_Mat_t *maskMat;
	Track_MemStorage_t* stuTrack_storage;

	//用于绘制的颜色
	Track_Colour_t pink_colour;
	Track_Colour_t blue_colour;
	Track_Colour_t lilac_colour;
	Track_Colour_t green_colour;
	Track_Colour_t red_colour;
	Track_Colour_t dullred_colour;
	Track_Colour_t yellow_colour;

	_callbackmsg callbackmsg_func;					//用于信息输出的函数指针
}StuITRACK_InteriorParams;

typedef struct _StuITRACK_SystemParams
{
	int nsrcHeight;			//源图像高度
	int nsrcWidth;			//源图像宽度
	_callbackmsg callbackmsg_func;					//用于信息输出的函数指针
}StuITRACK_SystemParams_t;

typedef struct 	_StuITRACK_Params
{
	StuITRACK_SystemParams_t systemParams;
	StuITRACK_ClientParams_t clientParams;
}StuITRACK_Params;

//变化状态宏
#define RESULT_STUTRACK_NULL_FLAG		0
#define	RESULT_STUTRACK_STANDUP_FLAG	1
#define	RESULT_STUTRACK_SITDOWN_FLAG	2
#define	RESULT_STUTRACK_MOVE_FLAG		4
#define RESULT_STUTRACK_STOPMOVE_FLAG	8

//这几个宏对result_flag进行判断
#define RESULT_STUTRACK_IF_STANDUP(n)		((n & RESULT_STUTRACK_STANDUP_FLAG)== RESULT_STUTRACK_STANDUP_FLAG)		//判断是否有起立的动作
#define RESULT_STUTRACK_IF_SITDOWN(n)		((n & RESULT_STUTRACK_SITDOWN_FLAG)== RESULT_STUTRACK_SITDOWN_FLAG)		//判断是否有坐下的动作
#define RESULT_STUTRACK_IF_MOVE(n)			((n & RESULT_STUTRACK_MOVE_FLAG)== RESULT_STUTRACK_MOVE_FLAG)			//判断是否有新的移动目标
#define RESULT_STUTRACK_IF_STOPMOVE(n)		((n & RESULT_STUTRACK_STOPMOVE_FLAG)== RESULT_STUTRACK_STOPMOVE_FLAG)	//判断是否有移动目标停止运动

//默认输入参数值
#define SCALE_STURACK_DEFAULT_ZOOM						0.333333333333333333
#define THRESHOLD_STUTRACK_MOVE_DEFALUT_PARAMS			1.0
#define THRESHOLD_STUTRACK_STANDCOUNT_DEFALUT_PARAMS	5
#define THRESHOLD_STUTRACK_SITDOWNCOUNT_DEFALUT_PARAMS	5
#define THRESHOLD_STUTRACK_MOVEDELAYED_DEFALUT_PARAMS	1000
#define RANGE_STUTRACK_STANDDIRECT_DEFALUT_PARAMS		30

//默认的变换矩阵
#define MATRIX_STUTRACK_DEFAULT_PARAMS double m[9]=\
	{2.7, 0.0,-600.0,\
	 0.0, -2.8, 380.0,\
	 0.0, 0.0,   1.0};

#define MATRIX_STUTRACK_DEFAULT_PARAMS_AT(n) m[n]

#define STRETCHING_STUTRACK_DEFFAULT_PARAMS_A (-90)
#define STRETCHING_STUTRACK_DEFFAULT_PARAMS_B (15000)
//用于计算筛选阈值的线性方程参数
#define A_STUTRACK_SIZE_THRESHOLD_PARAMS		(0.23)
#define B_STUTRACK_SIZE_THRESHOLD_PARAMS		(-14.0)
#define DIRECT_STUTRACK_TRANSMUTABLILITY_RANGE	(0.0)
#define A_STUTRACK_DIRECT_THRESHOLD_PARAMS		(DIRECT_STUTRACK_TRANSMUTABLILITY_RANGE/WIDTH_STUTRACK_IMG_)
#define B_STUTRACK_DIRECT_THRESHOLD_PARAMS		(270-DIRECT_STUTRACK_TRANSMUTABLILITY_RANGE/2)
#define MINTHRESHOLD_STUTRACK_SIZE_THRESHOLD_PARAMS		18
#define MAXTHRESHOLD_STUTRACK_SIZE_THRESHOLD_PARAMS		60
#define MINTHRESHOLD_STUTRACK_DIRECT_THRESHOLD_PARAMS	225
#define MAXTHRESHOLD_STUTRACK_DIRECT_THRESHOLD_PARAMS	315

#define COMPUTER_STUTRACK_SIZE_THRESHOLD_PARAMS(n,a,b)  (ITC_MIN(ITC_MAX(((a *n + b)), MINTHRESHOLD_STUTRACK_SIZE_THRESHOLD_PARAMS), MAXTHRESHOLD_STUTRACK_SIZE_THRESHOLD_PARAMS))
#define COMPUTER_STUTRACK_DIRECT_THRESHOLD_PARAMS(n,a,b)  (ITC_MIN(ITC_MAX(((a *n + b)), MINTHRESHOLD_STUTRACK_DIRECT_THRESHOLD_PARAMS), MAXTHRESHOLD_STUTRACK_DIRECT_THRESHOLD_PARAMS))


itc_BOOL stuTrack_initializeTrack(const StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p);

typedef int stuTrackReturn;
#define RETURN_STUTRACK_NEED_PROCESS 1		//需要处理
#define RETURN_STUTRACK_noNEED_PROCESS 0	//不需要处理
//************************************
// 函数名称: stuTrack_Process
// 函数说明：函数处理每一帧的图像数据，对学生区域进行起立跟踪和移动跟踪
// 作    者：XueYB
// 作成日期：2015/12/12
// 返 回 值: RETURN_STUTRACK_NEED_PROCESS表示return_params是有变化的，RETURN_STUTRACK_noNEED_PROCESS表示没有变化不需要处理
// 参    数: inst是输入的系统参数，interior_params_p用保存跟踪变量和参数，return_params是输出的跟踪结果
//************************************
stuTrackReturn stuTrack_process(const StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, StuITRACK_OutParams_t* return_params, char* imageData, char* bufferuv);

void stuTrack_stopTrack(const StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p);

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif

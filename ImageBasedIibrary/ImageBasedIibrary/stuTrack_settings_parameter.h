#ifndef stuTrack_settings_parameter_h__
#define stuTrack_settings_parameter_h__

#ifdef  __cplusplus
extern "C" {
#endif

#define HEIGHT_STUTRACK_IMG_ 264
#define WIDTH_STUTRACK_IMG_	480

typedef struct _POINT
{
	int x;
	int y;
}TrackPrarms_Point_t;

typedef struct _SIZE
{
	int width;
	int height;
}TrackPrarms_Size_t;

typedef struct _StuITRACK_ClientParams
{
	int flag_setting;	//参数是否被设置
	int height;			//设置的图像高度
	int width;			//设置的图像宽度
	int stuTrack_debugMsg_flag;					//调试信息输出等级
	int stuTrack_Draw_flag;						//是否绘制结果
	int stuTrack_direct_standard[4];			//四个顶点位置竖直方向在图像中的角度
	int stuTrack_stuWidth_standard[4];			//四个顶点位置学生在图像中所占的宽度
	int stuTrack_direct_range;					//起立时允许的角度偏离范围
	int stuTrack_standCount_threshold;			//判定为起立的帧数阈值
	int stuTrack_sitdownCount_threshold;		//判定为坐下的帧数阈值
	int stuTrack_moveDelayed_threshold;			//移动目标保持跟踪的延时，超过这个时间无运动，则放弃跟踪(单位：毫秒)
	double stuTrack_move_threshold;				//判定是移动目标的偏离阈值（比值）
	TrackPrarms_Point_t stuTrack_vertex[4];		//学生区域四个顶点位置

	double transformationMatrix[9];				//图像坐标与云台相机的变换矩阵
	double stretchingAB[2];						//拉伸系数
}StuITRACK_ClientParams_t;

//---------------------------------------------------------------------输出参数相关


//-----------------------------------------------------------------------

#define RESULT_STUTRAKC_NULL_CAMERA				0			//表示可以结束学生画面了
#define RESULT_STUTRAKC_MOVE_CAMERA				1			//移动镜头
#define RESULT_STUTRAKC_FEATURE_CAMERA			2			//切学生特写
#define RESULT_STUTRAKC_PAN0RAMA_CAMERA			4			//学生全景
#define RESULT_STUTRAKC_noTCH_FEATURE_CAMERA	8			//需要老师不在讲台上再切特写
#define RESULT_STUTRAKC_noTCH_PAN0RAMA_CAMERA	16			//需要老师无动作时切全景

typedef struct _StuITRACK_OutParams
{
	int result_flag;						//当前需要的状态
	TrackPrarms_Point_t position;			//目标位置
	int stretchingCoefficient;				//拉伸系数
}StuITRACK_OutParams_t;

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif

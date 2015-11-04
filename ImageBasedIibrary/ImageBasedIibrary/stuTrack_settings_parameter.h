#ifndef stuTrack_settings_parameter_h__
#define stuTrack_settings_parameter_h__

#ifdef  __cplusplus
extern "C" {
#endif

typedef struct _POINT
{
	int x;
	int y;
}TrackPrarms_Point_t;

typedef struct 	_StuITRACK_Params
{
	int flag_setting;	//参数是否被设置
	int height;			//图像高度
	int width;			//图像宽度

	TrackPrarms_Point_t stuTrack_vertex[4];		//学生区域四个顶点位置
	int stuTrack_direct_standard[4];			//四个顶点位置竖直方向在图像中的角度
	int stuTrack_stuWidth_standard[4];			//四个顶点位置学生在图像中所占的宽度
	int stuTrack_direct_range;					//起立时允许的角度偏离范围
	float stuTrack_move_threshold;				//判定是移动目标的偏离阈值（比值）
	int stuTrack_standCount_threshold;			//判定为起立的帧数阈值
	int stuTrack_sitdownCount_threshold;		//判定为坐下的帧数阈值
	int sturTrack_moveDelayed_threshold;		//移动目标保持跟踪的延时，超过这个时间无运动，则放弃跟踪(单位：毫秒)
}StuITRACK_Params;

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
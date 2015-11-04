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
	int flag_setting;	//�����Ƿ�����
	int height;			//ͼ��߶�
	int width;			//ͼ����

	TrackPrarms_Point_t stuTrack_vertex[4];		//ѧ�������ĸ�����λ��
	int stuTrack_direct_standard[4];			//�ĸ�����λ����ֱ������ͼ���еĽǶ�
	int stuTrack_stuWidth_standard[4];			//�ĸ�����λ��ѧ����ͼ������ռ�Ŀ��
	int stuTrack_direct_range;					//����ʱ����ĽǶ�ƫ�뷶Χ
	float stuTrack_move_threshold;				//�ж����ƶ�Ŀ���ƫ����ֵ����ֵ��
	int stuTrack_standCount_threshold;			//�ж�Ϊ������֡����ֵ
	int stuTrack_sitdownCount_threshold;		//�ж�Ϊ���µ�֡����ֵ
	int sturTrack_moveDelayed_threshold;		//�ƶ�Ŀ�걣�ָ��ٵ���ʱ���������ʱ�����˶������������(��λ������)
}StuITRACK_Params;


#define STUTRACK_RETURN_NULL		0
#define	STUTRACK_RETURN_STANDUP		1
#define	STUTRACK_RETURN_SITDOWN		2
#define	STUTRACK_RETURN_MOVE		4
#define STUTRACK_RETURN_STOPMOVE	8

typedef struct _StuITRACK_OutParams
{
	int result_flag;
	int count_trackObj_stand;
	int count_trackObj_bigMove;
	TrackPrarms_Point_t stand_position;
	TrackPrarms_Point_t move_position;
	int standObj_size[2];
	int moveObj_size[2];
}StuITRACK_OutParams_t;

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
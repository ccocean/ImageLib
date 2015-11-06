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

typedef struct _SIZE
{
	int width;
	int height;
}TrackPrarms_Size_t;

typedef struct 	_StuITRACK_Params
{
	int flag_setting;	//�����Ƿ�����
	int height;			//ͼ��߶�
	int width;			//ͼ����

	TrackPrarms_Point_t stuTrack_vertex[4];		//ѧ�������ĸ�����λ��
	int stuTrack_direct_standard[4];			//�ĸ�����λ����ֱ������ͼ���еĽǶ�
	int stuTrack_stuWidth_standard[4];			//�ĸ�����λ��ѧ����ͼ������ռ�Ŀ��
	unsigned int stuTrack_direct_range;					//����ʱ����ĽǶ�ƫ�뷶Χ
	float stuTrack_move_threshold;				//�ж����ƶ�Ŀ���ƫ����ֵ����ֵ��
	unsigned int stuTrack_standCount_threshold;			//�ж�Ϊ������֡����ֵ
	unsigned int stuTrack_sitdownCount_threshold;		//�ж�Ϊ���µ�֡����ֵ
	unsigned int stuTrack_moveDelayed_threshold;		//�ƶ�Ŀ�걣�ָ��ٵ���ʱ���������ʱ�����˶������������(��λ������)
}StuITRACK_Params;

//
#define RESULT_STUTRACK_NEWCHANGE_FLAG		(1<<30)
#define RESULT_STUTRACK_IF_NEWCHANGE(n)	((n & RESULT_STUTRACK_NEWCHANGE_FLAG)== RESULT_STUTRACK_NEWCHANGE_FLAG)	//�ж��Ƿ��б仯

//�仯״̬��
#define RESULT_STUTRACK_NULL_FLAG		0
#define	RESULT_STUTRACK_STANDUP_FLAG	1
#define	RESULT_STUTRACK_SITDOWN_FLAG	2
#define	RESULT_STUTRACK_MOVE_FLAG		4
#define RESULT_STUTRACK_STOPMOVE_FLAG	8

#define RESULT_STUTRACK_IF_STANDUP(n)		((n & RESULT_STUTRACK_STANDUP_FLAG)== RESULT_STUTRACK_STANDUP_FLAG)	//�ж��Ƿ�������
#define RESULT_STUTRACK_IF_SITDOWN(n)		((n & RESULT_STUTRACK_SITDOWN_FLAG)== RESULT_STUTRACK_SITDOWN_FLAG)	//�ж��Ƿ�������
#define RESULT_STUTRACK_IF_MOVE(n)			((n & RESULT_STUTRACK_MOVE_FLAG)== RESULT_STUTRACK_MOVE_FLAG)			//�ж��Ƿ����ƶ�Ŀ��
#define RESULT_STUTRACK_IF_STOPMOVE(n)		((n & RESULT_STUTRACK_STOPMOVE_FLAG)== RESULT_STUTRACK_STOPMOVE_FLAG)	//�ж��Ƿ����ƶ�Ŀ��ֹͣ�˶�

typedef struct _StuITRACK_OutParams
{
	int result_flag;							//��ǰ֡�仯״̬
	unsigned int count_trackObj_stand;			//����Ŀ�����
	unsigned int count_trackObj_bigMove;		//�ƶ�Ŀ�����
	TrackPrarms_Point_t stand_position;			//����Ŀ��λ��
	TrackPrarms_Point_t move_position;			//�ƶ�Ŀ��λ��
	TrackPrarms_Size_t standObj_size;			//����Ŀ���С
	TrackPrarms_Size_t moveObj_size;			//�ƶ�Ŀ���С
}StuITRACK_OutParams_t;

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
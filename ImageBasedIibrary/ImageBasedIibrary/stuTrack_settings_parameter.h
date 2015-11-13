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
	int flag_setting;	//�����Ƿ�����
	int height;			//���õ�ͼ��߶�
	int width;			//���õ�ͼ����
	int stuTrack_debugMsg_flag;					//������Ϣ����ȼ�
	int stuTrack_Draw_flag;						//�Ƿ���ƽ��
	int stuTrack_direct_standard[4];			//�ĸ�����λ����ֱ������ͼ���еĽǶ�
	int stuTrack_stuWidth_standard[4];			//�ĸ�����λ��ѧ����ͼ������ռ�Ŀ��
	int stuTrack_direct_range;					//����ʱ����ĽǶ�ƫ�뷶Χ
	int stuTrack_standCount_threshold;			//�ж�Ϊ������֡����ֵ
	int stuTrack_sitdownCount_threshold;		//�ж�Ϊ���µ�֡����ֵ
	int stuTrack_moveDelayed_threshold;			//�ƶ�Ŀ�걣�ָ��ٵ���ʱ���������ʱ�����˶������������(��λ������)
	double stuTrack_move_threshold;				//�ж����ƶ�Ŀ���ƫ����ֵ����ֵ��
	TrackPrarms_Point_t stuTrack_vertex[4];		//ѧ�������ĸ�����λ��
}StuITRACK_ClientParams_t;

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
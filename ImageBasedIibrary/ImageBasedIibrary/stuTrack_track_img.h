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

#define STUTRACK_IMG_HEIGHT 264
#define STUTRACK_IMG_WIDTH	480

#define MALLOC_ELEMENT_COUNT 100

#define _PRINTF ((callbackmsg)(interior_params_p->callbackmsg_func))

typedef struct StuTrack_Stand_t
{
	int direction;
	int count_teack;	//
	int count_up;		//
	int count_down;		//
	int flag_Stand;		//������־
	int flag_matching;	//ƥ���־
	Track_Point_t centre;
	Track_Rect_t roi;
	clock_t start_tClock;
	clock_t current_tClock;
}StuTrack_Stand_t;

typedef struct StuTrack_BigMoveObj_t
{
	int count_track;
	int flag_bigMove;		//��־
	int dis_threshold;		//��Ϊ���ƶ�Ŀ�����ֵ
	Track_Rect_t roi;
	clock_t start_tClock;
	clock_t current_tClock;
	Track_Point_t origin_position;
	Track_Point_t current_position;
}StuTrack_BigMoveObj_t;

typedef struct _StuITRACK_InteriorParams
{
	int _count;			//ͳ��֡��
	int img_size;		//ͼ���Сw*h
	int count_trackObj_stand;		//�����������
	StuTrack_Stand_t* stuTrack_stand;

	int count_trackObj_bigMove;		//�ƶ�Ŀ�����
	StuTrack_BigMoveObj_t* stuTrack_bigMOveObj;

	int count_stuTrack_rect;		//�˶��������
	Track_Rect_t *stuTrack_rect_arr;

	Track_MemStorage_t* stuTrack_storage;

	int *stuTrack_size_threshold;				//�˶�Ŀ���С������ֵ������λ�ò�ͬ��ֵ��ͬ��
	int *stuTrack_direct_threshold;				//�����ı�׼�Ƕ�,��СΪwidth
	Itc_Mat_t *tempMat;
	Itc_Mat_t *currMat;
	Itc_Mat_t *lastMat;
	Itc_Mat_t *mhiMat;
	Itc_Mat_t *maskMat;

	callbackmsg callbackmsg_func;						//������Ϣ����ĺ���ָ��
}StuITRACK_InteriorParams;

void stuTrack_filtrate_contours(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, Track_Contour_t** pContour);		//����ɸѡ

int stuTrack_matchingSatnd_ROI(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, Track_Rect_t roi);					//ƥ��roi

void stuTrack_analyze_ROI(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p);
int stuTrack_judgeStand_ROI(StuITRACK_Params *inst, StuTrack_Stand_t teack_stand);			//�ж��Ƿ�����

void stuTrack_proStandDown_ROI(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p);

void stuTrack_initializeTrack(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p);
void stuTrack_process(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p, char* imageData);
void stuTrack_stopTrack(StuITRACK_Params *inst, StuITRACK_InteriorParams* interior_params_p);

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
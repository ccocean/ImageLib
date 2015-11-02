#ifndef stuTrack_track_img_h__
#define stuTrack_track_img_h__
#include "itctype.h"
#include "itcerror.h"
#include "itcdatastructs.h"
#include "itcCore.h"

#ifdef  __cplusplus
extern "C" {
#endif

#define STUTRACK_IMG_HEIGHT 264
#define STUTRACK_IMG_WIDTH	480

#define MALLOC_ELEMENT_COUNT 100

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
	clock_t start_tClock;
	clock_t current_tClock;
}StuTrack_Stand_t;

typedef struct StuTrack_BigMoveObj_t
{
	int count_track;
	int flag_bigMove;		//标志
	int dis_threshold;		//认为是移动目标的阈值
	Track_Rect_t roi;
	clock_t start_tClock;
	clock_t current_tClock;
	Track_Point_t origin_position;
	Track_Point_t current_position;
}StuTrack_BigMoveObj_t;

int stuTrack_filtrate_contours(Track_Contour_t** pContour);			//轮廓筛选

int stuTrack_matchingSatnd_ROI(Itc_Mat_t* mhi, Track_Rect_t roi);	//匹配roi

void stuTrack_analyze_ROI(Itc_Mat_t* mhi);
int stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t teack_stand);								//判断是否起立

void stuTrack_proStandDown_ROI(Itc_Mat_t* mhi);

void stuTrack_initializeTrack(int height, int width);
void stuTrack_main(char* imageData);
void stuTrack_stopTrack();

#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
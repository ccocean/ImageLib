#ifndef stuTrack_track_img_h__
#define stuTrack_track_img_h__
#include "itctype.h"
#include "itcerror.h"
#include "itcdatastructs.h"
#include "itcCore.h"

#ifdef  __cplusplus
extern "C" {
#endif

#define STUTRACK_IMG_HEIGHT 270
#define STUTRACK_IMG_WIDTH	480

#define MALLOC_ELEMENT_COUNT 100

typedef struct StuTrack_Stand_t
{
	int direction;
	Track_Point_t centre;
	Track_Rect_t roi;
	int count_teack;	//
	int count_up;		//
	int count_down;		//
	int flag_Stand;		//起立标志
	int flag_matching;	//匹配标志
	//int startVertex_Y;
	//int max_height;
}StuTrack_Stand_t;

typedef struct StuTrack_BigMoveObj_t
{
	clock_t start_tClock;
	clock_t current_tClock;
	int count_track;
	Track_Rect_t roi;
	Track_Point_t origin_position;
	Track_Point_t current_position;
	int flag__bigMove;		//标志
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
#ifndef stuTrack_track_img_h__
#define stuTrack_track_img_h__
#include "itctype.h"
#include "itcerror.h"
#include "itcdatastructs.h"
#include "itcCore.h"

typedef struct StuTrack_Stand_t
{
	int direction;
	ItcPoint centre;
	ItcRect roi;
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
	int count_track;
	int count_utrack;
	ItcPoint origin_position;
	ItcPoint current_position;
}StuTrack_BigMoveObj_t;

int stuTrack_filtrate_contours(ItcContour** pContour, int size_Threshold[], ItcRect *rect_arr);			//轮廓筛选
bool stuTrack_matchingSatnd_ROI(Itc_Mat_t* mhi, ItcRect roi, StuTrack_Stand_t teack_stand[], int &count_trackObj, int direct_threshold[], int direct_range);	//匹配roi
void stuTrack_analyze_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t teack_stand[], int &count_trackObj, int direct_threshold[], int direct_range);
int stuTrack_judgeStand_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t teack_stand);								//判断是否起立

void stuTrack_proStandDown_ROI(Itc_Mat_t* mhi, StuTrack_Stand_t teack_stand[], int &count_trackObj, ItcRect rect_arr[], int &count_rect, int direct_threshold[], int direct_range);

void stuTrack_initializeTrack(int height, int width);
void stuTrack_main(char* imageData);
void stuTrack_stopTrack();
#endif
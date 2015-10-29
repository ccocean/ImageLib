#ifndef _TCH_TRACK_
#define _TCH_TRACK_

#include "itcCore.h"
#include<time.h>

//定义老师跟踪状态
#define RETURN_TRACK_TCH_STAND 1
#define RETURN_TRACK_TCH_MOVEINVIEW 2
#define RETURN_TRACK_TCH_MOVEOUTVIEW 3
#define RETURN_TRACK_TCH_OUTSIDE 4
#define RETURN_TRACK_TCH_BLACKBOARD 5
#define RETURN_TRACK_TCH_MULITY 6

int tch_lastStatus;

//站定时间的阈值
//#define TRACK_STAND_THRESHOLD 2
//#define TRACK_TARGETAREA_THRESHOLD 12000
//#define TRACK_TCHOUTSIDE_THRESHOLD 130

//阈值
int track_standThreshold;
int track_targetAreaThreshold;
int track_tchOutsideThreshold;

Track_Size_t g_frameSize;
Track_Rect_t g_tchWin;
Track_Rect_t g_blkWin;

//double g_time = 0;
int g_posIndex;
int g_prevPosIndex;
int g_flag;
int g_rectCnt;

int *tch_pos;

//计时器
typedef struct TrackTimer
{
	clock_t start;
	clock_t finish;
	/*double timeLast;
	double timeNow;*/
	double deltaTime;
}Track_Timer_t;

//预置位块
typedef struct CamPosition
{
	int index;
	//pixels
	int left_pixel;
	int right_pixel;
}Track_CamPosition_t;

//预置位滑块
typedef struct CamPositionSlide
{
	//index
	int center;
	int left;
	int right;
	int width;
}Track_CamPosSlide_t;
//预置位相应宏参数
#define TRACK_SLIDE_WIDTH 5
#define TRACK_NUMOF_POSITION 10
int track_pos_width;
//#define TRACK_POS_WIDTH (g_frameSize.width/TRACK_NUMOF_POSITION)

//判定旗帜
int g_isMulti;
int g_isOnStage;
int g_count;

Track_Point_t center;
Track_Point_t lastCenter;

Itc_Mat_t *srcMat;
Itc_Mat_t *tempMatTch;
Itc_Mat_t *tempMatBlk;
Itc_Mat_t *prevMatTch;
Itc_Mat_t *prevMatBlk;
Itc_Mat_t *currMatTch;
Itc_Mat_t *mhiMatTch;
Itc_Mat_t *maskMatTch;
Itc_Mat_t *currMatBlk;
Itc_Mat_t *mhiMatBlk;
Itc_Mat_t *maskMatBlk;

Track_MemStorage_t *storage;
Track_MemStorage_t *storageTch;
Track_MemStorage_t *storageBlk;


//计时器定义
Track_Timer_t slideTimer;

//预置位滑块定义
Track_CamPosSlide_t pos_slide;

//初始化预置位块
Track_CamPosition_t cam_pos[TRACK_NUMOF_POSITION];

char* g_videoPath;

//教师跟踪的函数

void tch_trackDestroy();

int tch_trackInit();

int tch_track(Itc_Mat_t *src);

int tch_calculateDirect_TCH(Itc_Mat_t* src, Track_Rect_t roi);

int tch_setArg_blkWin(int x, int y, int width, int height);

int tch_setArg_tchWin(int x, int y, int width, int height);

int tch_setArg_framSize(int width, int height);

int tch_setArg_threshold(int stand, int targetArea, int outside);

#endif
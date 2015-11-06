#ifndef itcTrack_draw_img_h__
#define itcTrack_draw_img_h__
#include "itctype.h"

#ifdef  __cplusplus
extern "C" {
#endif
typedef struct _colour
{
	uchar val[3];
}Trcak_Colour_t;

Trcak_Colour_t colour(uchar y, uchar u, uchar v);
Trcak_Colour_t colour_RGB2YUV(int R, int G, int B);

void track_draw_point(uchar *buffer, Track_Size_t *img_size, Track_Point_t* start_point, Trcak_Colour_t *yuv_value);
void track_draw_line(uchar *buffer, Track_Size_t* img_size, Track_Point_t* start_point, Track_Point_t* end_point, Trcak_Colour_t *yuv_value);
void track_draw_rectangle(uchar *buffer, Track_Size_t* img_size, Track_Rect_t* rect, Trcak_Colour_t *yuv_value);
#ifdef  __cplusplus  
}
#endif  /* end of __cplusplus */ 

#endif
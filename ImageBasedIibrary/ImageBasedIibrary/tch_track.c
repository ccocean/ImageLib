#include "tch_track.h"

//判定旗帜
//int g_isMulti;
//int g_isOnStage;
//int g_count;
////预置位
//int track_pos_width;
//
//Track_Point_t center;
//Track_Point_t lastCenter;
//
//Itc_Mat_t *srcMat;
//Itc_Mat_t *tempMatTch;
//Itc_Mat_t *tempMatBlk;
//Itc_Mat_t *prevMatTch;
//Itc_Mat_t *prevMatBlk;
//Itc_Mat_t *currMatTch;
//Itc_Mat_t *mhiMatTch;
//Itc_Mat_t *maskMatTch;
//Itc_Mat_t *currMatBlk;
//Itc_Mat_t *mhiMatBlk;
//Itc_Mat_t *maskMatBlk;
//
//Track_MemStorage_t *storage;
//Track_MemStorage_t *storageTch;
//Track_MemStorage_t *storageBlk;


//计时器定义
//Tch_Timer_t slideTimer;

//预置位滑块定义
//Tch_CamPosSlide_t pos_slide;

//初始化预置位块
//Tch_CamPosition_t cam_pos[TRACK_NUMOF_POSITION];
//
////阈值
//int track_standThreshold;
//int track_targetAreaThreshold;
//int track_tchOutsideThreshold;
//
//Track_Size_t g_frameSize;
//Track_Rect_t g_tchWin;
//Track_Rect_t g_blkWin;

//double g_time = 0;
//int g_posIndex;
//int g_prevPosIndex;
//int g_flag;
//int g_rectCnt;
//int tch_lastStatus;
//int *tch_pos;



int tch_trackInit(Tch_Data_t *data)
{
	/*track_standThreshold = 2;
	track_targetAreaThreshold = 12000;
	track_tchOutsideThreshold = 130;*/
	int i=0;
	/*if (g_frameSize.width <= 0)
	{
		return -1;
	}*/
	data->track_pos_width = data->g_frameSize.width / TRACK_NUMOF_POSITION;

	data->tch_lastStatus = 0;

	
	data->g_posIndex = -1;
	data->g_prevPosIndex = -1;
	data->g_flag = 0;
	//tch_pos = &g_posIndex;

	data->g_isMulti = 0;
	data->g_isOnStage = 0;
	

	data->slideTimer.start = 0;
	data->slideTimer.finish = 0;
	data->slideTimer.deltaTime = 0;

	data->pos_slide.width = (TRACK_SLIDE_WIDTH - 1) / 2;
	data->pos_slide.center = -1;
	data->pos_slide.left = -1;
	data->pos_slide.right = -1;

	for (i = 0; i < data->g_frameSize.width; i += data->track_pos_width)
	{
		data->cam_pos[i / data->track_pos_width].index = i / data->track_pos_width;
		data->cam_pos[i / data->track_pos_width].left_pixel = i;
		data->cam_pos[i / data->track_pos_width].right_pixel = i + data->track_pos_width;
	}

	data->srcMat = itc_create_mat(data->g_frameSize.height, data->g_frameSize.width, ITC_8UC1);

	data->prevMatTch = itc_create_mat(data->g_tchWin.height, data->g_tchWin.width, ITC_8UC1);
	data->prevMatBlk = itc_create_mat(data->g_blkWin.height, data->g_blkWin.width, ITC_8UC1);
	data->tempMatTch = NULL;
	data->tempMatBlk = NULL;

	data->currMatTch = itc_create_mat(data->g_tchWin.height, data->g_tchWin.width, ITC_8UC1);
	data->mhiMatTch = itc_create_mat(data->g_tchWin.height, data->g_tchWin.width, ITC_8UC1);
	data->maskMatTch = itc_create_mat(data->g_tchWin.height, data->g_tchWin.width, ITC_8UC1);
	data->currMatBlk = itc_create_mat(data->g_blkWin.height, data->g_blkWin.width, ITC_8UC1);
	data->mhiMatBlk = itc_create_mat(data->g_blkWin.height, data->g_blkWin.width, ITC_8UC1);
	data->maskMatBlk = itc_create_mat(data->g_blkWin.height, data->g_blkWin.width, ITC_8UC1);

	data->storage = itcCreateMemStorage(0);
	data->storageTch = itcCreateChildMemStorage(data->storage);
	data->storageBlk = itcCreateChildMemStorage(data->storage);

	return 0;
}

int tch_track(char *src, TeaITRACK_Params *params, Tch_Data_t *data, Tch_Result_t *res)
{
	int err = 0;
	err = tch_setParams(params, data);
	if (err<0)
	{
		return err;
	}
	else
	{
		err = tch_trackInit(data);
		if (err<0)
		{
			return err;
		}
	}
	
	//Tch_Result_t res;
	int i = 0, j = 0;
	memcpy(data->srcMat->data.ptr, src, params->frame.width*params->frame.height);
	if (data->g_count>0)
	{
		ITC_SWAP(data->currMatTch, data->prevMatTch, data->tempMatTch);
		ITC_SWAP(data->currMatBlk, data->prevMatBlk, data->tempMatBlk);
	}
	track_copyImage_ROI(data->srcMat, data->currMatTch, data->g_tchWin);
	track_copyImage_ROI(data->srcMat, data->currMatBlk, data->g_blkWin);

	int s_contourRectTch = 0;//老师的轮廓数目
	int s_contourRectBlk = 0;//板书的轮廓数目
	Track_Rect_t s_rectsTch[100];
	Track_Rect_t s_rectsBlk[100];
	Track_Rect_t s_bigRects[100];//筛选出来的大面积运动物体
	int s_maxdist = -1;//比较多个面积
	int s_rectCnt = 0;
	

	if (data->g_count>0)
	{
		
		//g_time = (double)cvGetTickCount();
		track_update_MHI(data->currMatTch, data->prevMatTch, data->mhiMatTch, 10, data->maskMatTch, 235);
		Track_Contour_t *contoursTch = NULL;
		itcClearMemStorage(data->storageTch);
		track_find_contours(data->maskMatTch, &contoursTch, data->storageTch);
		s_contourRectTch = track_filtrate_contours(&contoursTch, 20, s_rectsTch);

		track_update_MHI(data->currMatBlk, data->prevMatBlk, data->mhiMatBlk, 10, data->maskMatBlk, 235);
		Track_Contour_t *contoursBlk = NULL;
		itcClearMemStorage(data->storageBlk);
		track_find_contours(data->maskMatBlk, &contoursBlk, data->storageBlk);
		s_contourRectBlk = track_filtrate_contours(&contoursBlk, 20, s_rectsBlk);

		if (s_contourRectBlk > 0)
		{
			/*res.status = RETURN_TRACK_TCH_BLACKBOARD;
			res.pos = -1;
			return res;*/
			res->status = RETURN_TRACK_TCH_BLACKBOARD;
			res->pos = data->g_prevPosIndex;
			return RETURN_TRACK_TCH_BLACKBOARD;
		}

		//比较多个Rect之间x坐标的距离
		for (i = 0; i < s_contourRectTch; i++)
		{
			if (params->threshold.targetArea<s_rectsTch[i].width*s_rectsTch[i].height)
			{
				s_bigRects[s_rectCnt] = s_rectsTch[i];
				s_rectCnt++;
			}
		}
		if (0 == s_rectCnt)
		{
			if (data->g_prevPosIndex >= 0)
			{
				if (data->pos_slide.left <= data->g_prevPosIndex&&data->g_prevPosIndex <= data->pos_slide.right)
				{
					data->slideTimer.finish = clock();
					data->slideTimer.deltaTime = data->slideTimer.finish - data->slideTimer.start;
					if ((data->slideTimer.deltaTime / CLOCKS_PER_SEC) > params->threshold.stand)
					{
						if (data->g_isOnStage == 0)
						{
							res->status = RETURN_TRACK_TCH_OUTSIDE;
							res->pos = -1;
							return RETURN_TRACK_TCH_OUTSIDE;
						}
						else
						{
							res->status = data->tch_lastStatus;
							res->pos = data->g_prevPosIndex;
							return data->tch_lastStatus;//返回特写镜头命令
						}
					}
				}
				else
				{
					data->slideTimer.finish = 0;
					data->slideTimer.deltaTime = 0;
					data->slideTimer.start = clock();
				}
			}
		}
		else if (s_rectCnt>1)
		{
			s_maxdist = -1;
			for (i = 0; i < s_rectCnt; i++)
			{
				for (j = i + 1; j < s_rectCnt; j++)
				{
					if (abs(s_bigRects[i].x - s_bigRects[j].x)>s_maxdist)
					{
						s_maxdist = abs(s_bigRects[i].x - s_bigRects[j].x);
					}
					else
						continue;
				}
			}
			if (s_maxdist>data->track_pos_width)
			{
				data->g_isMulti = 1;
				data->tch_lastStatus = RETURN_TRACK_TCH_MULITY;
				res->status = RETURN_TRACK_TCH_MULITY;
				res->pos = -1;
				return RETURN_TRACK_TCH_MULITY;
			}
		}
		else
		{
			for (i = 0; i < s_rectCnt; i++)
			{
				data->g_isMulti = 0;
				int direct = -1;
				direct = tch_calculateDirect_TCH(data->mhiMatTch, s_rectsTch[i]);
				if (direct>-1)
				{
					data->center = itcPoint(s_bigRects[i].x + (s_bigRects[i].width >> 1), s_bigRects[i].y + (s_bigRects[i].height >> 1));
					data->lastCenter = data->center;
					for (j = 0; j < TRACK_NUMOF_POSITION; j++)
					{
						if (data->cam_pos[j].left_pixel <= data->center.x&&data->center.x <= data->cam_pos[j].right_pixel)
						{
							if (!data->g_prevPosIndex)
							{
								data->g_prevPosIndex = data->cam_pos[j].index;
							}
							else
							{
								//判断获得的位置和上次位置之间的距离，如果过大的话认为是多目标
								data->g_posIndex = data->cam_pos[j].index;
								if (abs(data->g_prevPosIndex - data->g_posIndex)>data->pos_slide.width + 1)
								{
									if (data->g_isOnStage == 1)
									{
										//tch_pos = &g_prevPosIndex;
										data->g_posIndex = data->g_prevPosIndex;
										data->g_isMulti = 1;
										data->tch_lastStatus = RETURN_TRACK_TCH_MULITY;
										res->status = RETURN_TRACK_TCH_MULITY;
										res->pos = data->g_prevPosIndex;
										return RETURN_TRACK_TCH_MULITY;
									}
									else
									{
										//tch_pos = &g_posIndex;
										res->pos = data->g_posIndex;
										data->g_prevPosIndex = data->g_posIndex;
									}
								}
								else
								{
									//tch_pos = &g_posIndex;
									res->pos = data->g_posIndex;
									data->g_prevPosIndex = data->g_posIndex;
									//g_posIndex = cam_pos[j].index;
								}
							}

							if (data->pos_slide.center<0)//当预置位滑框未被使用时
							{
								//初始化计时器
								data->slideTimer.start = clock();
								if (data->g_posIndex<data->pos_slide.width)
								{
									data->pos_slide.center = data->pos_slide.width;
									data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
									data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
								}
								else if (data->g_posIndex>TRACK_NUMOF_POSITION - 1 - data->pos_slide.width)
								{
									data->pos_slide.center = TRACK_NUMOF_POSITION - 1 - data->pos_slide.width;
									data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
									data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
								}
								else
								{
									data->pos_slide.center = data->g_posIndex;
									data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
									data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
								}
								break;
							}
							else
							{
								if (data->g_posIndex == data->pos_slide.right || data->g_posIndex == data->pos_slide.left)//当人物走到滑框的边缘，就更新滑框，相当于调全景移动云台相机
								{
									//更新滑框时，计时器置0，重新记时
									//当目标在最左端和最右端时不要重置计时器
									if (data->g_posIndex<data->pos_slide.width)
									{
										if (data->g_posIndex == 0)
										{
											data->slideTimer.finish = clock();
											data->slideTimer.deltaTime = data->slideTimer.finish - data->slideTimer.start;
											if ((data->slideTimer.deltaTime / CLOCKS_PER_SEC) > params->threshold.stand)
											{
												if (data->g_isMulti == 0)//不是多目标
												{
													if (data->center.y > params->threshold.outside)//lastCenter
													{
														data->g_isOnStage = 0;
														data->tch_lastStatus = RETURN_TRACK_TCH_OUTSIDE;
														res->status = RETURN_TRACK_TCH_OUTSIDE;
														return RETURN_TRACK_TCH_OUTSIDE;
													}
													else
													{
														data->g_isOnStage = 1;
														data->tch_lastStatus = RETURN_TRACK_TCH_MOVEINVIEW;
														res->status = RETURN_TRACK_TCH_MOVEINVIEW;
														return RETURN_TRACK_TCH_MOVEINVIEW;//返回全景镜头命令
													}
												}
												else
												{
													data->tch_lastStatus = RETURN_TRACK_TCH_MULITY;
													res->status = RETURN_TRACK_TCH_MULITY;
													return RETURN_TRACK_TCH_MULITY;
												}
											}
											else
											{
												data->tch_lastStatus = RETURN_TRACK_TCH_MOVEOUTVIEW;
												res->status = RETURN_TRACK_TCH_MOVEOUTVIEW;
												return RETURN_TRACK_TCH_MOVEOUTVIEW;//返回全景镜头命令
											}
										}
										else
										{
											data->slideTimer.finish = 0;
											data->slideTimer.deltaTime = 0;
											data->slideTimer.start = clock();
										}
										data->pos_slide.center = data->pos_slide.width;
										data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
										data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
									}
									else if (data->g_posIndex>TRACK_NUMOF_POSITION - 1 - data->pos_slide.width)
									{
										if (data->g_posIndex == TRACK_NUMOF_POSITION - 1)
										{
											data->slideTimer.finish = clock();
											data->slideTimer.deltaTime = data->slideTimer.finish - data->slideTimer.start;
											if ((data->slideTimer.deltaTime / CLOCKS_PER_SEC) > params->threshold.stand)
											{
												if (data->g_isMulti == 0)//不是多目标
												{
													if (data->lastCenter.y > params->threshold.outside)
													{
														data->g_isOnStage = 0;
														data->tch_lastStatus = RETURN_TRACK_TCH_OUTSIDE;
														res->status = RETURN_TRACK_TCH_OUTSIDE;
														return RETURN_TRACK_TCH_OUTSIDE;
													}
													else
													{
														data->g_isOnStage = 1;
														data->tch_lastStatus = RETURN_TRACK_TCH_MOVEINVIEW;
														res->status = RETURN_TRACK_TCH_MOVEINVIEW;
														return RETURN_TRACK_TCH_MOVEINVIEW;//返回全景镜头命令
														
													}
												}
												else
												{
													data->tch_lastStatus = RETURN_TRACK_TCH_MULITY;
													res->status = RETURN_TRACK_TCH_MULITY;
													res->pos = -1;
													return RETURN_TRACK_TCH_MULITY;
												}
											}
											else
											{
												data->tch_lastStatus = RETURN_TRACK_TCH_MOVEOUTVIEW;
												res->status = RETURN_TRACK_TCH_MOVEOUTVIEW;
												return RETURN_TRACK_TCH_MOVEOUTVIEW;//返回全景镜头命令
											}
										}
										else
										{
											data->slideTimer.finish = 0;
											data->slideTimer.deltaTime = 0;
											data->slideTimer.start = clock();
										}
										data->pos_slide.center = TRACK_NUMOF_POSITION - 1 - data->pos_slide.width;
										data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
										data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
									}
									else
									{
										data->pos_slide.center = data->g_posIndex;
										data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
										data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;

										data->slideTimer.finish = 0;
										data->slideTimer.deltaTime = 0;
										data->slideTimer.start = clock();
									}

									break;
								}
								else if (data->pos_slide.left>data->g_posIndex || data->g_posIndex>data->pos_slide.right)//当目标已经出了滑框
								{
									//当追踪到的目标已经在上一帧滑框的外面则立刻更新滑框
									if (data->g_posIndex < data->pos_slide.width)
									{

										data->slideTimer.finish = 0;
										data->slideTimer.deltaTime = 0;
										data->slideTimer.start = clock();

										data->pos_slide.center = data->pos_slide.width;
										data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
										data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
									}
									else if (data->g_posIndex > TRACK_NUMOF_POSITION - 1 - data->pos_slide.width)
									{

										data->slideTimer.finish = 0;
										data->slideTimer.deltaTime = 0;
										data->slideTimer.start = clock();

										data->pos_slide.center = TRACK_NUMOF_POSITION - 1 - data->pos_slide.width;
										data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
										data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;
									}
									else
									{
										data->pos_slide.center = data->g_posIndex;
										data->pos_slide.left = data->pos_slide.center - data->pos_slide.width;
										data->pos_slide.right = data->pos_slide.center + data->pos_slide.width;

										data->slideTimer.finish = 0;
										data->slideTimer.deltaTime = 0;
										data->slideTimer.start = clock();
									}
									break;
								}
								//没有移动到边缘就不更新滑框
								else
								{
									//在这里加入时间计算，到了阈值之后告诉Control该调特写镜头了。
									data->slideTimer.finish = clock();
									data->slideTimer.deltaTime = data->slideTimer.finish - data->slideTimer.start;
									if ((data->slideTimer.deltaTime / CLOCKS_PER_SEC) > params->threshold.stand)
									{
										if (data->g_isMulti == 0)//不是多目标
										{
											if (data->lastCenter.y>params->threshold.outside)
											{
												data->g_isOnStage = 0;
												data->tch_lastStatus = RETURN_TRACK_TCH_OUTSIDE;
												res->status = RETURN_TRACK_TCH_OUTSIDE;
												return RETURN_TRACK_TCH_OUTSIDE;
											}
											else
											{
												data->g_isOnStage = 1;
												data->tch_lastStatus = RETURN_TRACK_TCH_MOVEINVIEW;
												res->status = RETURN_TRACK_TCH_MOVEINVIEW;
												return RETURN_TRACK_TCH_MOVEINVIEW;//返回全景镜头命令
											}
										}
										else
										{
											data->tch_lastStatus = RETURN_TRACK_TCH_MULITY;
											res->status = RETURN_TRACK_TCH_MULITY;
											res->pos = -1;
											return RETURN_TRACK_TCH_MULITY;
										}
									}
									else
									{
										data->tch_lastStatus = RETURN_TRACK_TCH_MOVEOUTVIEW;
										res->status = RETURN_TRACK_TCH_MOVEOUTVIEW;
										return RETURN_TRACK_TCH_MOVEOUTVIEW;//返回全景镜头命令
									}
									break;
								}
							}
						}
					}
				}
			}
		}
		//tch_pos = &g_prevPosIndex;
		res->status = data->tch_lastStatus;
		res->pos = data->g_prevPosIndex;
		return data->tch_lastStatus;
	}
	else
	{
		if (!data->currMatTch->data.ptr||!data->currMatBlk->data.ptr)
		{
			return -2;
		}
		return 0;
	}
	return 0;
}

int tch_calculateDirect_TCH(Itc_Mat_t* src, Track_Rect_t roi)
{
	int i = 0;
	int j = 0;
	int direct = 0;;
	int sum_gradientV = 0;		//垂直方向梯度
	int sum_gradientH = 0;		//水平方向

	int x1 = roi.x;
	int y1 = roi.y;
	int x2 = roi.x + roi.width;
	int y2 = roi.y + roi.height;

	int step = src->step;
	char *img0 = (char*)(src->data.ptr + step*y1);
	char *img = (char*)(src->data.ptr + step*(y1 + 1));

	for (i = y1; i < y2 - 1; i++)
	{
		for (j = x1; j < x2 - 1; j++)
		{
			if (img0[j] != 0)
			{
				if (img[j] != 0)
					sum_gradientV += img0[j] - img[j];
				if (img0[j + 1] != 0)
					sum_gradientH += img0[j] - img0[j + 1];
			}
		}
		img0 = img;
		img += step;
	}

	int threshold = roi.width*roi.height / 10;
	if (abs(sum_gradientV) > abs(sum_gradientH))
	{
		if (sum_gradientV > threshold)
		{
			direct = 2;
		}
		else if (sum_gradientV < -threshold)
		{
			direct = 1;
		}
	}
	else
	{
		if (sum_gradientH > threshold)
		{
			direct = 3;
		}
		else if (sum_gradientH < -threshold)
		{
			direct = 4;
		}
	}
	return direct;
	//printf("位置：%d,%d,大小：%d,%d 垂直梯度：%d,水平梯度：%d\n", x1, y1, roi.width, roi.height,sum_gradientV, sum_gradientH);
}

void tch_trackDestroy(Tch_Data_t *data)
{
	itcReleaseMemStorage(&data->storageTch);
	itcReleaseMemStorage(&data->storageBlk);
	itcReleaseMemStorage(&data->storage);

	itc_release_mat(&data->currMatTch);
	itc_release_mat(&data->prevMatTch);
	itc_release_mat(&data->mhiMatTch);
	itc_release_mat(&data->maskMatTch);

	itc_release_mat(&data->currMatBlk);
	itc_release_mat(&data->prevMatBlk);
	itc_release_mat(&data->mhiMatBlk);
	itc_release_mat(&data->maskMatBlk);
}

int tch_setParams(TeaITRACK_Params *params, Tch_Data_t *data)
{
	if (params->isSetParams==0)
	{
		params->frame.width = 640;
		params->frame.height = 360;

		params->tch.x = 0;
		params->tch.y = 100;
		params->tch.width = 640;
		params->tch.height = 200;

		params->blk.x = 0;
		params->blk.y = 35;
		params->blk.width = 640;
		params->blk.height = 50;

		params->threshold.stand = 2;
		params->threshold.outside = 130;
		params->threshold.targetArea = 12000;

		return 0;
	}
	//初始化帧的大小
	if (params->frame.width <= 0 || params->frame.height <= 0)
	{
		return -1;
	}
	data->g_frameSize.width = params->frame.width;
	data->g_frameSize.height = params->frame.height;
	//初始化教师框
	if (params->tch.width<0 || params->tch.height<0)
	{
		return -1;
	}
	else if (params->tch.width>params->frame.width || params->tch.height>params->frame.height)
	{
		return -1;
	}
	else if (params->tch.x<0 || params->tch.x>params->frame.width || params->tch.y<0 || params->tch.y>params->frame.height)
	{
		return -1;
	}
	else
	{
		data->g_tchWin.x = params->tch.x;
		data->g_tchWin.y = params->tch.y;
		data->g_tchWin.width = params->tch.width;
		data->g_tchWin.height = params->tch.height;
	}
	//初始化板书框体
	if (params->blk.width < 0 || params->blk.height < 0)
	{
		return -1;
	}
	else if (params->blk.width > params->frame.width || params->blk.height > params->frame.height)
	{
		return -1;
	}
	else if (params->blk.x < 0 || params->blk.x > params->frame.width || params->blk.y < 0 || params->blk.y > params->frame.height)
	{
		return -1;
	}
	else
	{
		data->g_blkWin.x = params->blk.x;
		data->g_blkWin.y = params->blk.y;
		data->g_blkWin.width = params->blk.width;
		data->g_blkWin.height = params->blk.height;
	}
	//初始化阈值
	if (params->threshold.stand <= 0 || params->threshold.targetArea <= 0 || params->threshold.outside <= 0)
	{
		return -1;
	}
	/*else
	{
	track_standThreshold = params->threshold.stand;
	track_targetAreaThreshold = params->threshold.targetArea;
	track_tchOutsideThreshold = params->threshold.outside;
	}*/
	
	return 0;
}
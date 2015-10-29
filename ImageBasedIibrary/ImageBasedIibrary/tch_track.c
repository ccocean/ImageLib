#include "tch_track.h"



int tch_trackInit()
{
	/*track_standThreshold = 2;
	track_targetAreaThreshold = 12000;
	track_tchOutsideThreshold = 130;*/

	if (g_frameSize.width <= 0)
	{
		return -1;
	}
	track_pos_width = g_frameSize.width / TRACK_NUMOF_POSITION;

	tch_lastStatus = 0;

	
	g_posIndex = -1;
	g_prevPosIndex = -1;
	g_flag = 0;
	g_rectCnt = 0;
	tch_pos = &g_posIndex;

	g_isMulti = 0;
	g_isOnStage = 0;
	g_count = 0;

	slideTimer.start = 0;
	slideTimer.finish = 0;
	slideTimer.deltaTime = 0;

	pos_slide.width = (TRACK_SLIDE_WIDTH - 1) / 2;
	pos_slide.center = -1;
	pos_slide.left = -1;
	pos_slide.right = -1;

	for (int i = 0; i < g_frameSize.width; i += track_pos_width)
	{
		cam_pos[i / track_pos_width].index = i / track_pos_width;
		cam_pos[i / track_pos_width].left_pixel = i;
		cam_pos[i / track_pos_width].right_pixel = i + track_pos_width;
	}

	srcMat = itc_create_mat(g_frameSize.height, g_frameSize.width, ITC_8UC1);

	prevMatTch = itc_create_mat(g_tchWin.height, g_tchWin.width, ITC_8UC1);
	prevMatBlk = itc_create_mat(g_blkWin.height, g_blkWin.width, ITC_8UC1);
	tempMatTch = NULL;
	tempMatBlk = NULL;

	currMatTch = itc_create_mat(g_tchWin.height, g_tchWin.width, ITC_8UC1);
	mhiMatTch = itc_create_mat(g_tchWin.height, g_tchWin.width, ITC_8UC1);
	maskMatTch = itc_create_mat(g_tchWin.height, g_tchWin.width, ITC_8UC1);
	currMatBlk = itc_create_mat(g_blkWin.height, g_blkWin.width, ITC_8UC1);
	mhiMatBlk = itc_create_mat(g_blkWin.height, g_blkWin.width, ITC_8UC1);
	maskMatBlk = itc_create_mat(g_blkWin.height, g_blkWin.width, ITC_8UC1);

	storage = itcCreateMemStorage(0);
	storageTch = itcCreateChildMemStorage(storage);
	storageBlk = itcCreateChildMemStorage(storage);

	g_videoPath =  "video/teacher.mp4";
	return 0;
}

int tch_setArg_framSize(int width,int height)
{
	if (width<=0||height<=0)
	{
		return -1;
	}
	g_frameSize.width = width;
	g_frameSize.height = height;
	return 0;
}

int tch_setArg_tchWin(int x,int y,int width, int height)
{
	if (width<0||height<0)
	{
		return -1;
	}
	else if (width>g_frameSize.width||height>g_frameSize.height)
	{
		return -1;
	}
	else if (x<0||x>g_frameSize.width||y<0||y>g_frameSize.height)
	{
		return -1;
	}
	else
	{
		g_tchWin.x = x;
		g_tchWin.y = y;
		g_tchWin.width = width;
		g_tchWin.height = height;
		return 0;
	}
}

int tch_setArg_blkWin(int x, int y, int width, int height)
{
	if (width < 0 || height < 0)
	{
		return -1;
	}
	else if (width > g_frameSize.width || height > g_frameSize.height)
	{
		return -1;
	}
	else if (x < 0 || x > g_frameSize.width || y < 0 || y > g_frameSize.height)
	{
		return -1;
	}
	else
	{
		g_blkWin.x = x;
		g_blkWin.y = y;
		g_blkWin.width = width;
		g_blkWin.height = height;
		return 0;
	}
}

int tch_setArg_threshold(int stand, int targetArea, int outside)
{
	if (stand <= 0 || targetArea <= 0 || outside <= 0)
	{
		return -1;
	}
	track_standThreshold = stand;
	track_targetAreaThreshold = targetArea;
	track_tchOutsideThreshold = outside;
	return 0;
}

int tch_track(Itc_Mat_t *src)
{
	if (g_count>0)
	{
		ITC_SWAP(currMatTch, prevMatTch, tempMatTch);
		ITC_SWAP(currMatBlk, prevMatBlk, tempMatBlk);
	}
	track_copyImage_ROI(src, currMatTch, g_tchWin);
	track_copyImage_ROI(src, currMatBlk, g_blkWin);

	int s_contourRectTch = 0;//��ʦ��������Ŀ
	int s_contourRectBlk = 0;//�����������Ŀ
	Track_Rect_t s_rectsTch[100];
	Track_Rect_t s_rectsBlk[100];
	Track_Rect_t s_bigRects[100];//ɸѡ�����Ĵ�����˶�����
	int s_maxdist = -1;//�Ƚ϶�����
	g_rectCnt = 0;
	

	if (g_count>0)
	{
		
		//g_time = (double)cvGetTickCount();
		track_update_MHI(currMatTch, prevMatTch, mhiMatTch, 10, maskMatTch, 235);
		Track_Contour_t *contoursTch = NULL;
		itcClearMemStorage(storageTch);
		track_find_contours(maskMatTch, &contoursTch, storageTch);
		s_contourRectTch = track_filtrate_contours(&contoursTch, 20, s_rectsTch);

		track_update_MHI(currMatBlk, prevMatBlk, mhiMatBlk, 10, maskMatBlk, 235);
		Track_Contour_t *contoursBlk = NULL;
		itcClearMemStorage(storageBlk);
		track_find_contours(maskMatBlk, &contoursBlk, storageBlk);
		s_contourRectBlk = track_filtrate_contours(&contoursBlk, 20, s_rectsBlk);

		if (s_contourRectBlk > 0)
		{
			return RETURN_TRACK_TCH_BLACKBOARD;
		}

		//�Ƚ϶��Rect֮��x����ľ���
		for (int i = 0; i < s_contourRectTch; i++)
		{
			if (track_targetAreaThreshold<s_rectsTch[i].width*s_rectsTch[i].height)
			{
				s_bigRects[g_rectCnt] = s_rectsTch[i];
				g_rectCnt++;
			}
		}
		if (0 == g_rectCnt)
		{
			if (g_prevPosIndex >= 0)
			{
				if (pos_slide.left <= g_prevPosIndex&&g_prevPosIndex <= pos_slide.right)
				{
					slideTimer.finish = clock();
					slideTimer.deltaTime = slideTimer.finish - slideTimer.start;
					if ((slideTimer.deltaTime / CLOCKS_PER_SEC) > track_standThreshold)
					{
						if (g_isOnStage == 0)
						{
							return RETURN_TRACK_TCH_OUTSIDE;
						}
						else
						{
							return tch_lastStatus;//������д��ͷ����
						}
					}
				}
				else
				{
					slideTimer.finish = 0;
					slideTimer.deltaTime = 0;
					slideTimer.start = clock();
				}
			}
		}
		else if (g_rectCnt>1)
		{
			s_maxdist = -1;
			for (int i = 0; i < g_rectCnt; i++)
			{
				for (int j = i + 1; j < g_rectCnt; j++)
				{
					if (abs(s_bigRects[i].x - s_bigRects[j].x)>s_maxdist)
					{
						s_maxdist = abs(s_bigRects[i].x - s_bigRects[j].x);
					}
					else
						continue;
				}
			}
			if (s_maxdist>track_pos_width)
			{
				g_isMulti = 1;
				tch_lastStatus = RETURN_TRACK_TCH_MULITY;
				return RETURN_TRACK_TCH_MULITY;
			}
		}
		else
		{
			for (int i = 0; i < g_rectCnt; i++)
			{
				g_isMulti = 0;
				int direct = -1;
				direct = tch_calculateDirect_TCH(mhiMatTch, s_rectsTch[i]);
				if (direct>-1)
				{
					center = itcPoint(s_bigRects[i].x + 0.5*s_bigRects[i].width, s_bigRects[i].y + 0.5*s_bigRects[i].height);
					lastCenter = center;
					for (int j = 0; j < TRACK_NUMOF_POSITION; j++)
					{
						if (cam_pos[j].left_pixel <= center.x&&center.x <= cam_pos[j].right_pixel)
						{
							if (!g_prevPosIndex)
							{
								g_prevPosIndex = cam_pos[j].index;
							}
							else
							{
								//�жϻ�õ�λ�ú��ϴ�λ��֮��ľ��룬�������Ļ���Ϊ�Ƕ�Ŀ��
								g_posIndex = cam_pos[j].index;
								if (abs(g_prevPosIndex - g_posIndex)>pos_slide.width + 1)
								{
									if (g_isOnStage == 1)
									{
										tch_pos = &g_prevPosIndex;
										g_posIndex = g_prevPosIndex;
										g_isMulti = 1;
										tch_lastStatus = RETURN_TRACK_TCH_MULITY;
										return RETURN_TRACK_TCH_MULITY;
									}
									else
									{
										tch_pos = &g_posIndex;
										g_prevPosIndex = g_posIndex;
									}
								}
								else
								{
									tch_pos = &g_posIndex;
									g_prevPosIndex = g_posIndex;
									//g_posIndex = cam_pos[j].index;
								}
							}

							if (pos_slide.center<0)//��Ԥ��λ����δ��ʹ��ʱ
							{
								//��ʼ����ʱ��
								slideTimer.start = clock();
								if (g_posIndex<pos_slide.width)
								{
									pos_slide.center = pos_slide.width;
									pos_slide.left = pos_slide.center - pos_slide.width;
									pos_slide.right = pos_slide.center + pos_slide.width;
								}
								else if (g_posIndex>TRACK_NUMOF_POSITION - 1 - pos_slide.width)
								{
									pos_slide.center = TRACK_NUMOF_POSITION - 1 - pos_slide.width;
									pos_slide.left = pos_slide.center - pos_slide.width;
									pos_slide.right = pos_slide.center + pos_slide.width;
								}
								else
								{
									pos_slide.center = g_posIndex;
									pos_slide.left = pos_slide.center - pos_slide.width;
									pos_slide.right = pos_slide.center + pos_slide.width;
								}
								break;
							}
							else
							{
								if (g_posIndex == pos_slide.right || g_posIndex == pos_slide.left)//�������ߵ�����ı�Ե���͸��»����൱�ڵ�ȫ���ƶ���̨���
								{
									//���»���ʱ����ʱ����0�����¼�ʱ
									//��Ŀ��������˺����Ҷ�ʱ��Ҫ���ü�ʱ��
									if (g_posIndex<pos_slide.width)
									{
										if (g_posIndex == 0)
										{
											slideTimer.finish = clock();
											slideTimer.deltaTime = slideTimer.finish - slideTimer.start;
											if ((slideTimer.deltaTime / CLOCKS_PER_SEC) > track_standThreshold)
											{
												if (g_isMulti == 0)//���Ƕ�Ŀ��
												{
													if (center.y > track_tchOutsideThreshold)//lastCenter
													{
														g_isOnStage = 0;
														tch_lastStatus = RETURN_TRACK_TCH_OUTSIDE;
														return RETURN_TRACK_TCH_OUTSIDE;
													}
													else
													{
														g_isOnStage = 1;
														tch_lastStatus = RETURN_TRACK_TCH_MOVEINVIEW;
														return RETURN_TRACK_TCH_MOVEINVIEW;//����ȫ����ͷ����
													}
												}
												else
												{
													tch_lastStatus = RETURN_TRACK_TCH_MULITY;
													return RETURN_TRACK_TCH_MULITY;
												}
											}
											else
											{
												tch_lastStatus = RETURN_TRACK_TCH_MOVEOUTVIEW;
												return RETURN_TRACK_TCH_MOVEOUTVIEW;//����ȫ����ͷ����
											}
										}
										else
										{
											slideTimer.finish = 0;
											slideTimer.deltaTime = 0;
											slideTimer.start = clock();
										}
										pos_slide.center = pos_slide.width;
										pos_slide.left = pos_slide.center - pos_slide.width;
										pos_slide.right = pos_slide.center + pos_slide.width;
									}
									else if (g_posIndex>TRACK_NUMOF_POSITION - 1 - pos_slide.width)
									{
										if (g_posIndex == TRACK_NUMOF_POSITION - 1)
										{
											slideTimer.finish = clock();
											slideTimer.deltaTime = slideTimer.finish - slideTimer.start;
											if ((slideTimer.deltaTime / CLOCKS_PER_SEC) > track_standThreshold)
											{
												if (g_isMulti == 0)//���Ƕ�Ŀ��
												{
													if (lastCenter.y > track_tchOutsideThreshold)
													{
														g_isOnStage = 0;
														tch_lastStatus = RETURN_TRACK_TCH_OUTSIDE;
														return RETURN_TRACK_TCH_OUTSIDE;
													}
													else
													{
														g_isOnStage = 1;
														tch_lastStatus = RETURN_TRACK_TCH_MOVEINVIEW;
														return RETURN_TRACK_TCH_MOVEINVIEW;//����ȫ����ͷ����
														
													}
												}
												else
												{
													tch_lastStatus = RETURN_TRACK_TCH_MULITY;
													return RETURN_TRACK_TCH_MULITY;
												}
											}
											else
											{
												tch_lastStatus = RETURN_TRACK_TCH_MOVEOUTVIEW;
												return RETURN_TRACK_TCH_MOVEOUTVIEW;//����ȫ����ͷ����
											}
										}
										else
										{
											slideTimer.finish = 0;
											slideTimer.deltaTime = 0;
											slideTimer.start = clock();
										}
										pos_slide.center = TRACK_NUMOF_POSITION - 1 - pos_slide.width;
										pos_slide.left = pos_slide.center - pos_slide.width;
										pos_slide.right = pos_slide.center + pos_slide.width;
									}
									else
									{
										pos_slide.center = g_posIndex;
										pos_slide.left = pos_slide.center - pos_slide.width;
										pos_slide.right = pos_slide.center + pos_slide.width;

										slideTimer.finish = 0;
										slideTimer.deltaTime = 0;
										slideTimer.start = clock();
									}

									break;
								}
								else if (pos_slide.left>g_posIndex || g_posIndex>pos_slide.right)//��Ŀ���Ѿ����˻���
								{
									//��׷�ٵ���Ŀ���Ѿ�����һ֡��������������̸��»���
									if (g_posIndex < pos_slide.width)
									{

										slideTimer.finish = 0;
										slideTimer.deltaTime = 0;
										slideTimer.start = clock();

										pos_slide.center = pos_slide.width;
										pos_slide.left = pos_slide.center - pos_slide.width;
										pos_slide.right = pos_slide.center + pos_slide.width;

									}
									else if (g_posIndex > TRACK_NUMOF_POSITION - 1 - pos_slide.width)
									{

										slideTimer.finish = 0;
										slideTimer.deltaTime = 0;
										slideTimer.start = clock();

										pos_slide.center = TRACK_NUMOF_POSITION - 1 - pos_slide.width;
										pos_slide.left = pos_slide.center - pos_slide.width;
										pos_slide.right = pos_slide.center + pos_slide.width;
									}
									else
									{
										pos_slide.center = g_posIndex;
										pos_slide.left = pos_slide.center - pos_slide.width;
										pos_slide.right = pos_slide.center + pos_slide.width;

										slideTimer.finish = 0;
										slideTimer.deltaTime = 0;
										slideTimer.start = clock();
									}
									break;
								}
								//û���ƶ�����Ե�Ͳ����»���
								else
								{
									//���������ʱ����㣬������ֵ֮�����Control�õ���д��ͷ�ˡ�
									slideTimer.finish = clock();
									slideTimer.deltaTime = slideTimer.finish - slideTimer.start;
									if ((slideTimer.deltaTime / CLOCKS_PER_SEC) > track_standThreshold)
									{
										if (g_isMulti == 0)//���Ƕ�Ŀ��
										{
											if (lastCenter.y>track_tchOutsideThreshold)
											{
												g_isOnStage = 0;
												tch_lastStatus = RETURN_TRACK_TCH_OUTSIDE;
												return RETURN_TRACK_TCH_OUTSIDE;
											}
											else
											{
												g_isOnStage = 1;
												tch_lastStatus = RETURN_TRACK_TCH_MOVEINVIEW;
												return RETURN_TRACK_TCH_MOVEINVIEW;//����ȫ����ͷ����
											}
										}
										else
										{
											tch_lastStatus = RETURN_TRACK_TCH_MULITY;
											return RETURN_TRACK_TCH_MULITY;
										}
									}
									else
									{
										tch_lastStatus = RETURN_TRACK_TCH_MOVEOUTVIEW;
										return RETURN_TRACK_TCH_MOVEOUTVIEW;//����ȫ����ͷ����
									}
									break;
								}
							}
						}
					}
				}
			}
		}
		tch_pos = &g_prevPosIndex;
		return tch_lastStatus;
	}
	else
	{
		if (!currMatTch->data.ptr||!currMatBlk->data.ptr)
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
	int sum_gradientV = 0;		//��ֱ�����ݶ�
	int sum_gradientH = 0;		//ˮƽ����

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
	//printf("λ�ã�%d,%d,��С��%d,%d ��ֱ�ݶȣ�%d,ˮƽ�ݶȣ�%d\n", x1, y1, roi.width, roi.height,sum_gradientV, sum_gradientH);
}

void tch_trackDestroy()
{
	itcReleaseMemStorage(&storageTch);
	itcReleaseMemStorage(&storageBlk);
	itcReleaseMemStorage(&storage);

	itc_release_mat(&currMatTch);
	itc_release_mat(&prevMatTch);
	itc_release_mat(&mhiMatTch);
	itc_release_mat(&maskMatTch);

	itc_release_mat(&currMatBlk);
	itc_release_mat(&prevMatBlk);
	itc_release_mat(&mhiMatBlk);
	itc_release_mat(&maskMatBlk);
}
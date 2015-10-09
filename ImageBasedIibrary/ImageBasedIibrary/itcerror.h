/************************************************************************** 
    *  @Copyright (c) 2015, XueYB, All rights reserved. 
 
    *  @file     : itcerror.h 
    *  @version  : ver 1.0 
 
    *  @author   : XueYB 
    *  @date     : 2015/10/09 11:45 
    *  @brief    : 一些错误的处理 
**************************************************************************/
#ifndef itcerror_h__
#define itcerror_h__

#define ITC_ERROR_(errors) printf("error:%s\n",errors);
#define ITC_ERROR_DETAIL(errors,info) printf("code:%s,err:%s",errors,info);

#endif // itcerror_h__
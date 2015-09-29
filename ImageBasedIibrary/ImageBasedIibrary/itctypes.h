/****************************************************************************************\
*                             Common macros and inline functions                         *
\****************************************************************************************/

inline int itcRound(double a)
{
	return (int)(a+0.5);
}

/*************************************** ItcRect *****************************************/

typedef struct ItcRect
{
	int x;
	int y;
	int width;
	int height;
}ItcRect;

inline ItcRect  itcRect(int x, int y, int width, int height)
{
	ItcRect r;

	r.x=x;
	r.y=y;
	r.width=width;
	r.height=height;

	return r;
}

/*************************************** ItcPoint *****************************************/

typedef struct ItcPoint
{
	int x;
	int y;
}ItcPoint;

inline ItcPoint itcPoint(int x, int y)
{
	ItcPoint p;
	p.x=x;
	p.y=y;

	return p;
}

typedef struct ItcPoint2D32f
{
	float x;
	float y;
}ItcPoint2D32f;


inline  ItcPoint2D32f  itcPoint2D32f( double x, double y )
{
	ItcPoint2D32f p;

	p.x = (float)x;
	p.y = (float)y;

	return p;
}


inline  ItcPoint2D32f  itcPointTo32f( ItcPoint point )
{
	return itcPoint2D32f( (float)point.x, (float)point.y );
}


inline  ItcPoint  itcPointFrom32f( ItcPoint2D32f point )
{
	ItcPoint ipt;
	ipt.x = itcRound(point.x);
	ipt.y = itcRound(point.y);

	return ipt;
}


typedef struct ItcPoint3D32f
{
	float x;
	float y;
	float z;
}ItcPoint3D32f;


inline  ItcPoint3D32f  itcPoint3D32f( double x, double y, double z )
{
	ItcPoint3D32f p;

	p.x = (float)x;
	p.y = (float)y;
	p.z = (float)z;

	return p;
}


typedef struct ItcPoint2D64f
{
	double x;
	double y;
}ItcPoint2D64f;


inline  ItcPoint2D64f  itcPoint2D64f( double x, double y )
{
	ItcPoint2D64f p;

	p.x = x;
	p.y = y;

	return p;
}


typedef struct ItcPoint3D64f
{
	double x;
	double y;
	double z;
}
ItcPoint3D64f;


inline  ItcPoint3D64f  itcPoint3D64f( double x, double y, double z )
{
	ItcPoint3D64f p;

	p.x = x;
	p.y = y;
	p.z = z;

	return p;
}

/******************************** CvSize's & CvBox **************************************/

typedef struct ItcSize
{
	int width;
	int height;
}ItcSize;

inline ItcSize itcSize(int width, int height)
{
	ItcSize s;
	s.width=width;
	s.height=height;
}


/****************************************************************************************\
*                                   Dynamic Data structures                              *
\****************************************************************************************/

/******************************** Memory storage ****************************************/

typedef struct ItcMemBlock
{
	struct ItcMemBlock*  prev;
	struct ItcMemBlock*  next;
}
ItcMemBlock;

typedef struct ItcMemStorage
{
	int signature;
	ItcMemBlock* bottom;/* first allocated block */
	ItcMemBlock* top;   /* current memory block - top of the stack */
	struct  ItcMemStorage* parent; /* borrows new blocks from */
	int block_size;  /* block size */
	int free_space;  /* free space in the current block */
}ItcMemStorage;
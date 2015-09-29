/****************************************************************************************\
*                             Common macros and inline functions                         *
\****************************************************************************************/


#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)
#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)
#define CV_MAT_TYPE_MASK        (CV_DEPTH_MAX*CV_CN_MAX - 1)
#define CV_MAT_TYPE(flags)      ((flags) & CV_MAT_TYPE_MASK)
#define CV_MAT_CONT_FLAG_SHIFT  14
#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
#define CV_IS_MAT_CONT(flags)   ((flags) & CV_MAT_CONT_FLAG)
#define CV_IS_CONT_MAT          CV_IS_MAT_CONT
#define CV_MAT_TEMP_FLAG_SHIFT  15
#define CV_MAT_TEMP_FLAG        (1 << CV_MAT_TEMP_FLAG_SHIFT)
#define CV_IS_TEMP_MAT(flags)   ((flags) & CV_MAT_TEMP_FLAG)

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

#define ITC_MAGIC_MASK       0xFFFF0000
#define ITC_MAT_MAGIC_VAL    0x42420000

typedef struct ItcMemBlock
{
	struct ItcMemBlock*  prev;
	struct ItcMemBlock*  next;
}
ItcMemBlock;

#define ITC_STORAGE_MAGIC_VAL    0x42890000

typedef struct ItcMemStorage
{
	int signature;
	ItcMemBlock* bottom;/* first allocated block */
	ItcMemBlock* top;   /* current memory block - top of the stack */
	struct  ItcMemStorage* parent; /* borrows new blocks from */
	int block_size;  /* block size */
	int free_space;  /* free space in the current block */
}ItcMemStorage;

#define CV_IS_STORAGE(storage)  \
	((storage) != NULL &&       \
	(((ItcMemStorage*)(storage))->signature & CV_MAGIC_MASK) == CV_STORAGE_MAGIC_VAL)

typedef struct ItcMemStoragePos
{
	ItcMemBlock* top;
	int free_space;
}ItcMemStoragePos;

/*********************************** Sequence *******************************************/
#define ITC_SEQ_MAGIC_VAL             0x42990000
typedef struct ItcSeqBlock
{
    struct ItcSeqBlock*  prev; /* previous sequence block */
    struct ItcSeqBlock*  next; /* next sequence block */
    int    start_index;       /* index of the first element in the block +
                                 sequence->first->start_index */
    int    count;             /* number of elements in the block */
    char*  data;              /* pointer to the first element of the block */
}ItcSeqBlock;

#define CV_TREE_NODE_FIELDS(node_type)                          \
	int       flags;         /* micsellaneous flags */          \
	int       header_size;   /* size of sequence header */      \
struct    node_type* h_prev; /* previous sequence */        \
struct    node_type* h_next; /* next sequence */            \
struct    node_type* v_prev; /* 2nd previous sequence */    \
struct    node_type* v_next  /* 2nd next sequence */

/*
   Read/Write sequence.
   Elements can be dynamically inserted to or deleted from the sequence.
*/
#define CV_SEQUENCE_FIELDS()                                            \
    CV_TREE_NODE_FIELDS(ItcSeq);                                         \
    int       total;          /* total number of elements */            \
    int       elem_size;      /* size of sequence element in bytes */   \
    char*     block_max;      /* maximal bound of the last block */     \
    char*     ptr;            /* current write pointer */               \
    int       delta_elems;    /* how many elements allocated when the seq grows */  \
    ItcMemStorage* storage;    /* where the seq is stored */             \
    ItcSeqBlock* free_blocks;  /* free blocks list */                    \
    ItcSeqBlock* first; /* pointer to the first sequence block */

typedef struct ItcSeq
{
    CV_SEQUENCE_FIELDS()
}
ItcSeq;

#define CV_TYPE_NAME_SEQ             "opencv-sequence"
#define CV_TYPE_NAME_SEQ_TREE        "opencv-sequence-tree"
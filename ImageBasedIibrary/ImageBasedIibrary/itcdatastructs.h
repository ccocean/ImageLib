#ifndef _ITCTYPE_H_
#define _ITCTYPE_H_


#pragma once
/****************************************************************************************\
*                             Common macros and inline functions                         *
\****************************************************************************************/


#define ITC_CN_MAX     64
#define ITC_CN_SHIFT   3
#define ITC_DEPTH_MAX  (1 << ITC_CN_SHIFT)

#define ITC_8U   0
#define ITC_8S   1
#define ITC_16U  2
#define ITC_16S  3
#define ITC_32S  4
#define ITC_32F  5
#define ITC_64F  6
#define ITC_USRTYPE1 7

#define ITC_MAKETYPE(depth,cn) ((depth) + (((cn)-1) << ITC_CN_SHIFT))
#define ITC_MAKE_TYPE ITC_MAKETYPE

#define ITC_8UC1 ITC_MAKETYPE(ITC_8U,1)
#define ITC_8UC2 ITC_MAKETYPE(ITC_8U,2)
#define ITC_8UC3 ITC_MAKETYPE(ITC_8U,3)
#define ITC_8UC4 ITC_MAKETYPE(ITC_8U,4)
#define ITC_8UC(n) ITC_MAKETYPE(ITC_8U,(n))

#define ITC_8SC1 ITC_MAKETYPE(ITC_8S,1)
#define ITC_8SC2 ITC_MAKETYPE(ITC_8S,2)
#define ITC_8SC3 ITC_MAKETYPE(ITC_8S,3)
#define ITC_8SC4 ITC_MAKETYPE(ITC_8S,4)
#define ITC_8SC(n) ITC_MAKETYPE(ITC_8S,(n))

#define ITC_16UC1 ITC_MAKETYPE(ITC_16U,1)
#define ITC_16UC2 ITC_MAKETYPE(ITC_16U,2)
#define ITC_16UC3 ITC_MAKETYPE(ITC_16U,3)
#define ITC_16UC4 ITC_MAKETYPE(ITC_16U,4)
#define ITC_16UC(n) ITC_MAKETYPE(ITC_16U,(n))

#define ITC_16SC1 ITC_MAKETYPE(ITC_16S,1)
#define ITC_16SC2 ITC_MAKETYPE(ITC_16S,2)
#define ITC_16SC3 ITC_MAKETYPE(ITC_16S,3)
#define ITC_16SC4 ITC_MAKETYPE(ITC_16S,4)
#define ITC_16SC(n) ITC_MAKETYPE(ITC_16S,(n))

#define ITC_32SC1 ITC_MAKETYPE(ITC_32S,1)
#define ITC_32SC2 ITC_MAKETYPE(ITC_32S,2)
#define ITC_32SC3 ITC_MAKETYPE(ITC_32S,3)
#define ITC_32SC4 ITC_MAKETYPE(ITC_32S,4)
#define ITC_32SC(n) ITC_MAKETYPE(ITC_32S,(n))

#define ITC_32FC1 ITC_MAKETYPE(ITC_32F,1)
#define ITC_32FC2 ITC_MAKETYPE(ITC_32F,2)
#define ITC_32FC3 ITC_MAKETYPE(ITC_32F,3)
#define ITC_32FC4 ITC_MAKETYPE(ITC_32F,4)
#define ITC_32FC(n) ITC_MAKETYPE(ITC_32F,(n))

#define ITC_64FC1 ITC_MAKETYPE(ITC_64F,1)
#define ITC_64FC2 ITC_MAKETYPE(ITC_64F,2)
#define ITC_64FC3 ITC_MAKETYPE(ITC_64F,3)
#define ITC_64FC4 ITC_MAKETYPE(ITC_64F,4)
#define ITC_64FC(n) ITC_MAKETYPE(ITC_64F,(n))

#define ITC_AUTO_STEP  0x7fffffff
#define ITC_WHOLE_ARR  itcSlice( 0, 0x3fffffff )

#define ITC_MAT_CN_MASK          ((ITC_CN_MAX - 1) << ITC_CN_SHIFT)
#define ITC_MAT_CN(flags)        ((((flags) & ITC_MAT_CN_MASK) >> ITC_CN_SHIFT) + 1)
#define ITC_MAT_DEPTH_MASK       (ITC_DEPTH_MAX - 1)
#define ITC_MAT_DEPTH(flags)     ((flags) & ITC_MAT_DEPTH_MASK)
#define ITC_MAT_TYPE_MASK        (ITC_DEPTH_MAX*ITC_CN_MAX - 1)
#define ITC_MAT_TYPE(flags)      ((flags) & ITC_MAT_TYPE_MASK)
#define ITC_MAT_CONT_FLAG_SHIFT  14
#define ITC_MAT_CONT_FLAG        (1 << ITC_MAT_CONT_FLAG_SHIFT)
#define ITC_IS_MAT_CONT(flags)   ((flags) & ITC_MAT_CONT_FLAG)
#define ITC_IS_CONT_MAT          ITC_IS_MAT_CONT
#define ITC_MAT_TEMP_FLAG_SHIFT  15
#define ITC_MAT_TEMP_FLAG        (1 << ITC_MAT_TEMP_FLAG_SHIFT)
#define ITC_IS_TEMP_MAT(flags)   ((flags) & ITC_MAT_TEMP_FLAG)


/****************************************************************************************\
*                                    Sequence types                                      *
\****************************************************************************************/
#define CV_SEQ_ELTYPE_GENERIC        0

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
#define ITC_SEQ_MAGIC_VAL             0x42990000  //稠密序列  队列，栈，向量
#define ITC_SET_MAGIC_VAL             0x42980000  //稀疏序列  图，点集，哈希表
typedef struct ItcSeqBlock
{
    struct ItcSeqBlock*  prev; /* previous sequence block */
    struct ItcSeqBlock*  next; /* next sequence block */
    int    start_index;       /* index of the first element in the block +
                                 sequence->first->start_index */
    int    count;             /* number of elements in the block */
    char*  data;              /* pointer to the first element of the block */
}ItcSeqBlock;

#define ITC_TREE_NODE_FIELDS(node_type)                          \
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
#define ITC_SEQUENCE_FIELDS()                                            \
    ITC_TREE_NODE_FIELDS(ItcSeq);                                         \
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
    ITC_SEQUENCE_FIELDS()
}
ItcSeq;

#define ITC_TYPE_NAME_SEQ             "opencv-sequence"
#define ITC_TYPE_NAME_SEQ_TREE        "opencv-sequence-tree"

/*********************************** Chain/Countour *************************************/

typedef struct ItcChain
{
	ITC_SEQUENCE_FIELDS()
		ItcPoint  origin;
}ItcChain;

#define ITC_CONTOUR_FIELDS()  \
	ITC_SEQUENCE_FIELDS()     \
	ItcRect rect;             \
	int color;               \
	int reserved[3];

typedef struct ItcContour
{
	ITC_CONTOUR_FIELDS()
}
ItcContour;

typedef ItcContour ItcPoint2DSeq;


// declaration
inline int  itcAlign(int size, int align);
inline void* itcAlignPtr(const void* ptr, int align);
inline int itcAlignLeft(int size, int align);
static void* itcDefaultAlloc(size_t size, void*);
static int itcDefaultFree(void* ptr, void*);
void*  itcAlloc(size_t size);
void  itcFree_(void* ptr);
static void itcInitMemStorage(ItcMemStorage* storage, int block_size);
ItcMemStorage* itcCreateMemStorage(int block_size);
ItcMemStorage* itcCreateChildMemStorage(ItcMemStorage * parent);
static void itcDestroyMemStorage(ItcMemStorage* storage);
void itcReleaseMemStorage(ItcMemStorage** storage);
void itcClearMemStorage(ItcMemStorage * storage);
static void itcGoNextMemBlock(ItcMemStorage * storage);
void itcSaveMemStoragePos(const ItcMemStorage * storage, ItcMemStoragePos * pos);
void itcRestoreMemStoragePos(ItcMemStorage * storage, ItcMemStoragePos * pos);
void* itcMemStorageAlloc(ItcMemStorage* storage, size_t size);
ItcSeq *itcCreateSeq(int seq_flags, int header_size, int elem_size, ItcMemStorage * storage);
void itcSetSeqBlockSize(ItcSeq *seq, int delta_elements);
char* itcGetSeqElem(const ItcSeq *seq, int index);
int itcSeqElemIdx(const ItcSeq* seq, const void* _element, ItcSeqBlock** _block);
static void itcGrowSeq(ItcSeq *seq, int in_front_of);
char* itcSeqPush(ItcSeq *seq, void *element);
void itcSeqPop(ItcSeq *seq, void *element);
static void itcFreeSeqBlock(ItcSeq *seq, int in_front_of);
char* itcSeqPushFront(ItcSeq *seq, void *element);
void itcSeqPopFront(ItcSeq *seq, void *element);
char* itcSeqInsert(ItcSeq *seq, int before_index, void *element);
void itcSeqRemove(ItcSeq *seq, int index);

#endif // !1

#ifndef _ITCTYPE_H_
#define _ITCTYPE_H_

#include "itcerror.h"
#include "limits.h"
#include "itctype.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>


#pragma once

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

#define ITC_IS_STORAGE(storage)  \
	((storage) != NULL &&       \
	(((ItcMemStorage*)(storage))->signature & ITC_MAGIC_MASK) == ITC_STORAGE_MAGIC_VAL)

typedef struct ItcMemStoragePos
{
	ItcMemBlock* top;
	int free_space;
}ItcMemStoragePos;

/*********************************** Sequence *******************************************/
#define ITC_SEQ_MAGIC_VAL             0x42990000  //稠密序列  队列，栈，向量

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

/*************************************** Set ********************************************/
/*
Set.
Order is not preserved. There can be gaps between sequence elements.
After the element has been inserted it stays in the same place all the time.
The MSB(most-significant or sign bit) of the first field (flags) is 0 iff the element exists.
*/
#define ITC_SET_MAGIC_VAL             0x42980000  //稀疏序列  图，点集，哈希表

#define ITC_SET_ELEM_FIELDS(elem_type)   \
	int  flags;                         \
struct elem_type* next_free;

typedef struct ItcSetElem
{
	ITC_SET_ELEM_FIELDS(ItcSetElem)
}
ItcSetElem;

#define ITC_SET_FIELDS()      \
	ITC_SEQUENCE_FIELDS()     \
	ItcSetElem* free_elems;   \
	int active_count;

typedef struct ItcSet
{
	ITC_SET_FIELDS()
}
ItcSet;

#define ITC_SET_ELEM_IDX_MASK   ((1 << 26) - 1)
#define ITC_SET_ELEM_FREE_FLAG  (1 << (sizeof(int)*8-1))

/* Checks whether the element pointed by ptr belongs to a set or not */
#define ITC_IS_SET_ELEM( ptr )  (((ItcSetElem*)(ptr))->flags >= 0)

/****************************************************************************************\
*                                    Sequence types                                      *
\****************************************************************************************/

//#define CV_SEQ_MAGIC_VAL             0x42990000

#define ITC_IS_SEQ(seq) \
	((seq) != NULL && (((ItcSeq*)(seq))->flags & ITC_MAGIC_MASK) == ITC_SEQ_MAGIC_VAL)

//#define CV_SET_MAGIC_VAL             0x42980000
#define ITC_IS_SET(set) \
	((set) != NULL && (((ItcSet*)(set))->flags & ITC_MAGIC_MASK) == ITC_SET_MAGIC_VAL)

#define ITC_SEQ_ELTYPE_BITS           9
#define ITC_SEQ_ELTYPE_MASK           ((1 << ITC_SEQ_ELTYPE_BITS) - 1)

#define ITC_SEQ_ELTYPE_POINT          ITC_32SC2  /* (x,y) */
#define ITC_SEQ_ELTYPE_CODE           ITC_8UC1   /* freeman code: 0..7 */
#define ITC_SEQ_ELTYPE_GENERIC        0
#define ITC_SEQ_ELTYPE_PTR            ITC_USRTYPE1
#define ITC_SEQ_ELTYPE_PPOINT         ITC_SEQ_ELTYPE_PTR  /* &(x,y) */
#define ITC_SEQ_ELTYPE_INDEX          ITC_32SC1  /* #(x,y) */
#define ITC_SEQ_ELTYPE_GRAPH_EDGE     0  /* &next_o, &next_d, &vtx_o, &vtx_d */
#define ITC_SEQ_ELTYPE_GRAPH_VERTEX   0  /* first_edge, &(x,y) */
#define ITC_SEQ_ELTYPE_TRIAN_ATR      0  /* vertex of the binary tree   */
#define ITC_SEQ_ELTYPE_CONNECTED_COMP 0  /* connected component  */
#define ITC_SEQ_ELTYPE_POINT3D        ITC_32FC3  /* (x,y,z)  */

#define ITC_SEQ_KIND_BITS        3
#define ITC_SEQ_KIND_MASK        (((1 << ITC_SEQ_KIND_BITS) - 1)<<ITC_SEQ_ELTYPE_BITS)

/* types of sequences */
#define ITC_SEQ_KIND_GENERIC     (0 << ITC_SEQ_ELTYPE_BITS)
#define ITC_SEQ_KIND_CURVE       (1 << ITC_SEQ_ELTYPE_BITS)
#define ITC_SEQ_KIND_BIN_TREE    (2 << ITC_SEQ_ELTYPE_BITS)

/* types of sparse sequences (sets) */
#define ITC_SEQ_KIND_GRAPH       (3 << ITC_SEQ_ELTYPE_BITS)
#define ITC_SEQ_KIND_SUBDIV2D    (4 << ITC_SEQ_ELTYPE_BITS)

#define ITC_SEQ_FLAG_SHIFT       (ITC_SEQ_KIND_BITS + ITC_SEQ_ELTYPE_BITS)

/* flags for curves */
#define ITC_SEQ_FLAG_CLOSED     (1 << ITC_SEQ_FLAG_SHIFT)
#define ITC_SEQ_FLAG_SIMPLE     (2 << ITC_SEQ_FLAG_SHIFT)
#define ITC_SEQ_FLAG_CONVEX     (4 << ITC_SEQ_FLAG_SHIFT)
#define ITC_SEQ_FLAG_HOLE       (8 << ITC_SEQ_FLAG_SHIFT)

/* flags for graphs */
#define ITC_GRAPH_FLAG_ORIENTED (1 << ITC_SEQ_FLAG_SHIFT)

#define ITC_GRAPH               ITC_SEQ_KIND_GRAPH
#define ITC_ORIENTED_GRAPH      (ITC_SEQ_KIND_GRAPH|ITC_GRAPH_FLAG_ORIENTED)

/* point sets */
#define ITC_SEQ_POINT_SET       (ITC_SEQ_KIND_GENERIC| ITC_SEQ_ELTYPE_POINT)
#define ITC_SEQ_POINT3D_SET     (ITC_SEQ_KIND_GENERIC| ITC_SEQ_ELTYPE_POINT3D)
#define ITC_SEQ_POLYLINE        (ITC_SEQ_KIND_CURVE  | ITC_SEQ_ELTYPE_POINT)
#define ITC_SEQ_POLYGON         (ITC_SEQ_FLAG_CLOSED | ITC_SEQ_POLYLINE )
#define ITC_SEQ_CONTOUR         ITC_SEQ_POLYGON
#define ITC_SEQ_SIMPLE_POLYGON  (ITC_SEQ_FLAG_SIMPLE | ITC_SEQ_POLYGON  )

/* chain-coded curves */
#define ITC_SEQ_CHAIN           (ITC_SEQ_KIND_CURVE  | ITC_SEQ_ELTYPE_CODE)
#define ITC_SEQ_CHAIN_CONTOUR   (ITC_SEQ_FLAG_CLOSED | ITC_SEQ_CHAIN)

/* binary tree for the contour */
#define ITC_SEQ_POLYGON_TREE    (ITC_SEQ_KIND_BIN_TREE  | ITC_SEQ_ELTYPE_TRIAN_ATR)

/* sequence of the connected components */
#define ITC_SEQ_CONNECTED_COMP  (ITC_SEQ_KIND_GENERIC  | ITC_SEQ_ELTYPE_CONNECTED_COMP)

/* sequence of the integer numbers */
#define ITC_SEQ_INDEX           (ITC_SEQ_KIND_GENERIC  | ITC_SEQ_ELTYPE_INDEX)

#define ITC_SEQ_ELTYPE( seq )   ((seq)->flags & ITC_SEQ_ELTYPE_MASK)
#define ITC_SEQ_KIND( seq )     ((seq)->flags & ITC_SEQ_KIND_MASK )

/* flag checking */
#define ITC_IS_SEQ_INDEX( seq )      ((ITC_SEQ_ELTYPE(seq) == ITC_SEQ_ELTYPE_INDEX) && \
	(ITC_SEQ_KIND(seq) == ITC_SEQ_KIND_GENERIC))

#define ITC_IS_SEQ_CURVE( seq )      (ITC_SEQ_KIND(seq) == ITC_SEQ_KIND_CURVE)
#define ITC_IS_SEQ_CLOSED( seq )     (((seq)->flags & ITC_SEQ_FLAG_CLOSED) != 0)
#define ITC_IS_SEQ_CONVEX( seq )     (((seq)->flags & ITC_SEQ_FLAG_CONVEX) != 0)
#define ITC_IS_SEQ_HOLE( seq )       (((seq)->flags & ITC_SEQ_FLAG_HOLE) != 0)
#define ITC_IS_SEQ_SIMPLE( seq )     ((((seq)->flags & ITC_SEQ_FLAG_SIMPLE) != 0) || \
	ITC_IS_SEQ_CONVEX(seq))

/* type checking macros */
#define ITC_IS_SEQ_POINT_SET( seq ) \
	((ITC_SEQ_ELTYPE(seq) == ITC_32SC2 || ITC_SEQ_ELTYPE(seq) == ITC_32FC2))

#define ITC_IS_SEQ_POINT_SUBSET( seq ) \
	(ITC_IS_SEQ_INDEX(seq) || ITC_SEQ_ELTYPE(seq) == ITC_SEQ_ELTYPE_PPOINT)

#define ITC_IS_SEQ_POLYLINE( seq )   \
	(ITC_SEQ_KIND(seq) == ITC_SEQ_KIND_CURVE && ITC_IS_SEQ_POINT_SET(seq))

#define ITC_IS_SEQ_POLYGON( seq )   \
	(ITC_IS_SEQ_POLYLINE(seq) && ITC_IS_SEQ_CLOSED(seq))

#define ITC_IS_SEQ_CHAIN( seq )   \
	(ITC_SEQ_KIND(seq) == ITC_SEQ_KIND_CURVE && (seq)->elem_size == 1)

#define ITC_IS_SEQ_CONTOUR( seq )   \
	(ITC_IS_SEQ_CLOSED(seq) && (ITC_IS_SEQ_POLYLINE(seq) || ITC_IS_SEQ_CHAIN(seq)))

#define ITC_IS_SEQ_CHAIN_CONTOUR( seq ) \
	(ITC_IS_SEQ_CHAIN(seq) && ITC_IS_SEQ_CLOSED(seq))

#define ITC_IS_SEQ_POLYGON_TREE( seq ) \
	(ITC_SEQ_ELTYPE(seq) == ITC_SEQ_ELTYPE_TRIAN_ATR &&    \
	ITC_SEQ_KIND(seq) == ITC_SEQ_KIND_BIN_TREE)

#define ITC_IS_GRAPH( seq )    \
	(ITC_IS_SET(seq) && ITC_SEQ_KIND((ItcSet*)(seq)) == ITC_SEQ_KIND_GRAPH)

#define ITC_IS_GRAPH_ORIENTED( seq )   \
	(((seq)->flags & ITC_GRAPH_FLAG_ORIENTED) != 0)

#define ITC_IS_SUBDIV2D( seq )  \
	(ITC_IS_SET(seq) && ITC_SEQ_KIND((ItcSet*)(seq)) == ITC_SEQ_KIND_SUBDIV2D)

/****************************************************************************************/
/*                            Sequence writer & reader                                  */
/****************************************************************************************/

#define ITC_SEQ_WRITER_FIELDS()                                     \
	int          header_size;                                      \
	ItcSeq*       seq;        /* the sequence written */            \
	ItcSeqBlock*  block;      /* current block */                   \
	char*        ptr;        /* pointer to free space */           \
	char*        block_min;  /* pointer to the beginning of block*/\
	char*        block_max;  /* pointer to the end of block */

typedef struct ItcSeqWriter
{
	ITC_SEQ_WRITER_FIELDS()
}
ItcSeqWriter;

#define ITC_SEQ_READER_FIELDS()                                      \
	int          header_size;                                       \
	ItcSeq*       seq;        /* sequence, beign read */             \
	ItcSeqBlock*  block;      /* current block */                    \
	char*        ptr;        /* pointer to element be read next */  \
	char*        block_min;  /* pointer to the beginning of block */\
	char*        block_max;  /* pointer to the end of block */      \
	int          delta_index;/* = seq->first->start_index   */      \
	char*        prev_elem;  /* pointer to previous element */


typedef struct ItcSeqReader
{
	ITC_SEQ_READER_FIELDS()
}
ItcSeqReader;

/****************************************************************************************/
/*                                Operations on sequences                               */
/****************************************************************************************/

char* itcGetSeqElem(const ItcSeq *seq, int index);
void itcCreateSeqBlock(ItcSeqWriter * writer);
void itcChangeSeqBlock(void* _reader, int direction);

#define ITC_FRONT 0 //在序列头部添加元素
#define ITC_BACK 1 //在序列尾部添加元素

#define  ITC_SEQ_ELEM( seq, elem_type, index )                    \
	/* assert gives some guarantee that <seq> parameter is valid */  \
	(assert(sizeof((seq)->first[0]) == sizeof(ItcSeqBlock) && \
	(seq)->elem_size == sizeof(elem_type)), \
	(elem_type*)((seq)->first && (unsigned)index < \
	(unsigned)((seq)->first->count) ? \
	(seq)->first->data + (index)* sizeof(elem_type) : \
	itcGetSeqElem((ItcSeq*)(seq), (index))))
#define ITC_GET_SEQ_ELEM( elem_type, seq, index ) ITC_SEQ_ELEM( (seq), elem_type, (index) )

/* macro that adds element to sequence */
#define ITC_WRITE_SEQ_ELEM_VAR( elem_ptr, writer )     \
{\
	if ((writer).ptr >= (writer).block_max)          \
	{                                                 \
		itcCreateSeqBlock(&writer);                   \
	}                                                 \
	memcpy((writer).ptr, elem_ptr, (writer).seq->elem_size); \
	(writer).ptr += (writer).seq->elem_size;          \
}

#define ITC_WRITE_SEQ_ELEM( elem, writer )             \
{                                                     \
	assert((writer).seq->elem_size == sizeof(elem)); \
	if ((writer).ptr >= (writer).block_max)          \
	{                                                 \
		itcCreateSeqBlock(&writer);                   \
	}                                                 \
	assert((writer).ptr <= (writer).block_max - sizeof(elem)); \
	memcpy((writer).ptr, &(elem), sizeof(elem));      \
	(writer).ptr += sizeof(elem);                     \
}

/* move reader position forward */
#define ITC_NEXT_SEQ_ELEM( elem_size, reader )                 \
{                                                             \
	if (((reader).ptr += (elem_size)) >= (reader).block_max) \
	{                                                         \
		itcChangeSeqBlock(&(reader), 1);                     \
	}                                                         \
}

/* move reader position backward */
#define ITC_PREV_SEQ_ELEM( elem_size, reader )                \
{                                                            \
if (((reader).ptr -= (elem_size)) < (reader).block_min) \
{                                                        \
	itcChangeSeqBlock(&(reader), -1);                   \
}                                                        \
}

/* read element and move read position forward */
#define ITC_READ_SEQ_ELEM( elem, reader )                       \
{                                                              \
	assert((reader).seq->elem_size == sizeof(elem));          \
	memcpy(&(elem), (reader).ptr, sizeof((elem)));            \
	ITC_NEXT_SEQ_ELEM(sizeof(elem), reader)                   \
}

/* read element and move read position backward */
#define ITC_REV_READ_SEQ_ELEM( elem, reader )                     \
{                                                                \
	assert((reader).seq->elem_size == sizeof(elem));            \
	memcpy(&(elem), (reader).ptr, sizeof((elem)));               \
	ITC_PREV_SEQ_ELEM(sizeof(elem), reader)                     \
}

#define ITC_READ_CHAIN_POINT( _pt, reader )                              \
{                                                                       \
	(_pt) = (reader).pt;                                                \
	if ((reader).ptr)                                                  \
	{                                                                   \
		ITC_READ_SEQ_ELEM((reader).code, (reader));                     \
		assert(((reader).code & ~7) == 0);                            \
		(reader).pt.x += (reader).deltas[(int)(reader).code][0];        \
		(reader).pt.y += (reader).deltas[(int)(reader).code][1];        \
	}                                                                   \
}

#define ITC_CURRENT_POINT( reader )  (*((ItcPoint*)((reader).ptr)))
#define ITC_PREV_POINT( reader )     (*((ItcPoint*)((reader).prev_elem)))

#define ITC_READ_EDGE( pt1, pt2, reader )               \
{                                                      \
	assert(sizeof(pt1) == sizeof(ItcPoint) && \
	sizeof(pt2) == sizeof(ItcPoint) && \
	reader.seq->elem_size == sizeof(ItcPoint)); \
	(pt1) = ITC_PREV_POINT(reader);                   \
	(pt2) = ITC_CURRENT_POINT(reader);                \
	(reader).prev_elem = (reader).ptr;                 \
	ITC_NEXT_SEQ_ELEM(sizeof(ItcPoint), (reader));      \
}

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


typedef  struct ItcLinkedRunPoint
{
	struct ItcLinkedRunPoint* link;
	struct ItcLinkedRunPoint* next;
	ItcPoint pt;
}
ItcLinkedRunPoint;

typedef ItcContour ItcPoint2DSeq;


/****************************************************************************************/
/*                                   declaration                                        */
/****************************************************************************************/
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
int itcSeqElemIdx(const ItcSeq* seq, const void* _element, ItcSeqBlock** _block);
static void itcGrowSeq(ItcSeq *seq, int in_front_of);
char* itcSeqPush(ItcSeq *seq, void *element);
void itcSeqPop(ItcSeq *seq, void *element);
static void itcFreeSeqBlock(ItcSeq *seq, int in_front_of);
char* itcSeqPushFront(ItcSeq *seq, void *element);
void itcSeqPopFront(ItcSeq *seq, void *element);
char* itcSeqInsert(ItcSeq *seq, int before_index, void *element);
void itcSeqRemove(ItcSeq *seq, int index);
void itcStartAppendToSeq(ItcSeq *seq, ItcSeqWriter * writer);
void itcStartWriteSeq(int seq_flags, int header_size,int elem_size, ItcMemStorage * storage, ItcSeqWriter * writer);
void itcFlushSeqWriter(ItcSeqWriter * writer);
ItcSeq * itcEndWriteSeq(ItcSeqWriter * writer);
void itcStartReadSeq(const ItcSeq *seq, ItcSeqReader * reader, int reverse);
void itcSeqPushMulti(ItcSeq *seq, void *_elements, int count, int front);
void itcSeqPopMulti(ItcSeq *seq, void *_elements, int count, int front);
void itcClearSeq(ItcSeq *seq);
int itcGetSeqReaderPos(ItcSeqReader* reader);
void itcSetSeqReaderPos(ItcSeqReader* reader, int index, int is_relative);

#endif // !1
#include "itctypes.h"
#include "limits.h"
#include "itcCore.h"
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <malloc.h>


#define cvFree(ptr) (cvFree_(*(ptr)), *(ptr)=0)

/* maximum size of dynamic memory buffer.
   cvAlloc reports an error if a larger block is requested. */
#define  ITC_MAX_ALLOC_SIZE    (((size_t)1 << (sizeof(size_t)*8-2)))
/* the alignment of all the allocated buffers */
#define  ITC_MALLOC_ALIGN    32
/* default storage block size */
#define  ITC_STORAGE_BLOCK_SIZE   ((1<<16) - 128)
/* default alignment for dynamic data strucutures, resided in storages. */
#define  ITC_STRUCT_ALIGN    ((int)sizeof(double))

#define ICV_FREE_PTR(storage)  \
	((char*)(storage)->top + (storage)->block_size - (storage)->free_space)
/* 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define ITC_ELEM_SIZE(type) \
	(ITC_MAT_CN(type) << ((((sizeof(size_t)/4+1)*16384|0x3a50) >> ITC_MAT_DEPTH(type)*2) & 3))
#define ITC_SHIFT_TAB_MAX 32
static const char itcPower2ShiftTab[] =
{
	0, 1, -1, 2, -1, -1, -1, 3, -1, -1, -1, -1, -1, -1, -1, 4,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5
};


//error code
#define  ITC_OK 0
#define  ITC_StsBadSize    -201 /* the input/output structure size is incorrect  */
#define  ITC_StsOutOfRange -211  /* some of parameters are out of range */
#define  ITC_StsNoMem -4	/* insufficient memory */
#define  ITC_StsNullPtr  -27 /* null pointer */
#define  ITC_BADARG_ERR   -49  //ipp comp

/* the alignment of all the allocated buffers */
#define  ITC_MALLOC_ALIGN    32
// pointers to allocation functions, initially set to default
static void* p_cvAllocUserData = 0;

// declaration
inline int  itcAlign(int size, int align);
inline void* itcAlignPtr( const void* ptr, int align );
inline int itcAlignLeft( int size, int align );
static void* itcDefaultAlloc( size_t size, void* );
static int itcDefaultFree( void* ptr, void* );
void*  itcAlloc( size_t size );
void  itcFree_( void* ptr );
static void itcInitMemStorage( ItcMemStorage* storage, int block_size );
ItcMemStorage* itcCreateMemStorage(int block_size);
ItcMemStorage* itcCreateChildMemStorage( ItcMemStorage * parent );
static void itcDestroyMemStorage( ItcMemStorage* storage );
void itcReleaseMemStorage( ItcMemStorage** storage );
void itcClearMemStorage( ItcMemStorage * storage );
static void itcGoNextMemBlock( ItcMemStorage * storage );
void itcSaveMemStoragePos( const ItcMemStorage * storage, ItcMemStoragePos * pos );
void itcRestoreMemStoragePos( ItcMemStorage * storage, ItcMemStoragePos * pos );
void* itcMemStorageAlloc( ItcMemStorage* storage, size_t size );
ItcSeq *itcCreateSeq( int seq_flags, int header_size, int elem_size, ItcMemStorage * storage );
void itcSetSeqBlockSize( ItcSeq *seq, int delta_elements );
char* itcGetSeqElem( const ItcSeq *seq, int index );
int itcSeqElemIdx( const ItcSeq* seq, const void* _element, ItcSeqBlock** _block );

inline int  itcAlign(int size, int align)
{
	assert((align&(align-1))==0 && size<INT_MAX);
	return (size + align - 1) & -align;
}

inline void* itcAlignPtr( const void* ptr, int align=32 )
{
	assert( (align & (align-1)) == 0 );
	return (void*)( ((size_t)ptr + align - 1) & ~(size_t)(align-1) );
}

inline int
	itcAlignLeft( int size, int align )
{
	return size & -align;
}



// default <malloc>
static void*
	itcDefaultAlloc( size_t size, void* )
{
	char *ptr, *ptr0 = (char*)malloc(
		(size_t)(size + ITC_MALLOC_ALIGN*((size >= 4096) + 1) + sizeof(char*)));

	if( !ptr0 )
		return 0;

	// align the pointer
	ptr = (char*)itcAlignPtr(ptr0 + sizeof(char*) + 1, ITC_MALLOC_ALIGN);
	*(char**)(ptr - sizeof(char*)) = ptr0;

	return ptr;
}

// default <free>
static int
	itcDefaultFree( void* ptr, void* )
{
	// Pointer must be aligned by CV_MALLOC_ALIGN
	if( ((size_t)ptr & (ITC_MALLOC_ALIGN-1)) != 0 )
		return ITC_BADARG_ERR;
	free( *((char**)ptr - 1) );

	return ITC_OK;
}

// pointers to allocation functions, initially set to default
//static CvAllocFunc p_cvAlloc = icvDefaultAlloc;

void*  itcAlloc( size_t size )
{
	void* ptr = 0;

	//CV_FUNCNAME( "cvAlloc" );

	//__BEGIN__;

	if( (size_t)size > ITC_MAX_ALLOC_SIZE )
		std::cout<<"err: "<<ITC_StsOutOfRange<<std::endl;

	ptr = itcDefaultAlloc( size, p_cvAllocUserData );
	if( !ptr )
		std::cout<<"err: "<<ITC_StsNoMem<<std::endl;

	//__END__;

	return ptr;
}

void  itcFree_( void* ptr )
{
	//CV_FUNCNAME( "cvFree_" );

	//__BEGIN__;

	if( ptr )
	{
		int status = itcDefaultFree( ptr, p_cvAllocUserData );
		if( status < 0 )
			std::cout<<"Deallocation error"<<std::endl;
	}

	//__END__;
}

/****************************************************************************************\
*            Functions for manipulating memory storage - list of memory blocks           *
\****************************************************************************************/

/* initializes allocated storage */
static void itcInitMemStorage( ItcMemStorage* storage, int block_size )
{
	//CV_FUNCNAME( "icvInitMemStorage " );

	//__BEGIN__;

	if( !storage )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	if( block_size <= 0 )
		block_size = ITC_STORAGE_BLOCK_SIZE;

	block_size = itcAlign( block_size, ITC_STRUCT_ALIGN );
	assert( sizeof(ItcMemBlock) % ITC_STRUCT_ALIGN == 0 );

	memset( storage, 0, sizeof( *storage ));
	storage->signature = ITC_STORAGE_MAGIC_VAL;
	storage->block_size = block_size;

	//__END__;
}

/* creates root memory storage */
ItcMemStorage* itcCreateMemStorage(int block_size)
{
	ItcMemStorage *storage = 0;

	storage=(ItcMemStorage *) itcAlloc(sizeof(ItcMemStorage));

	itcInitMemStorage( storage, block_size );

	return storage;
}

/* creates child memory storage */
ItcMemStorage* itcCreateChildMemStorage( ItcMemStorage * parent )
{
	ItcMemStorage *storage = 0;
	//CV_FUNCNAME( "cvCreateChildMemStorage" );

	//__BEGIN__;

	if( !parent )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	//CV_CALL( storage = cvCreateMemStorage(parent->block_size));
	storage = itcCreateMemStorage(parent->block_size);
	storage->parent = parent;

	//__END__;

	//if( cvGetErrStatus() < 0 )
		//cvFree( &storage );

	return storage;
}

/* releases all blocks of the storage (or returns them to parent if any) */
static void itcDestroyMemStorage( ItcMemStorage* storage )
{
	//CV_FUNCNAME( "icvDestroyMemStorage" );

	//__BEGIN__;

	int k = 0;

	ItcMemBlock *block;
	ItcMemBlock *dst_top = 0;

	if( !storage )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	if( storage->parent )
		dst_top = storage->parent->top;

	for( block = storage->bottom; block != 0; k++ )
	{
		ItcMemBlock *temp = block;

		block = block->next;
		if( storage->parent )
		{
			if( dst_top )
			{
				temp->prev = dst_top;
				temp->next = dst_top->next;
				if( temp->next )
					temp->next->prev = temp;
				dst_top = dst_top->next = temp;
			}
			else
			{
				dst_top = storage->parent->bottom = storage->parent->top = temp;
				temp->prev = temp->next = 0;
				storage->free_space = storage->block_size - sizeof( *temp );
			}
		}
		else
		{
			//cvFree( &temp );
			itcFree_(&temp);
		}
	}

	storage->top = storage->bottom = 0;
	storage->free_space = 0;

	//__END__;
}

/* releases memory storage */
void
	itcReleaseMemStorage( ItcMemStorage** storage )
{
	ItcMemStorage *st;
	//CV_FUNCNAME( "cvReleaseMemStorage" );

	//__BEGIN__;

	if( !storage )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	st = *storage;
	*storage = 0;

	if( st )
	{
		itcDestroyMemStorage(st);
		itcFree_(&st);
		//CV_CALL( icvDestroyMemStorage( st ));
		//cvFree( &st );
	}

	//__END__;
}

/* clears memory storage (returns blocks to the parent if any) */
void
	itcClearMemStorage( ItcMemStorage * storage )
{
	//CV_FUNCNAME( "cvClearMemStorage" );

	//__BEGIN__;

	if( !storage )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	if( storage->parent )
	{
		itcDestroyMemStorage( storage );
	}
	else
	{
		storage->top = storage->bottom;
		storage->free_space = storage->bottom ? storage->block_size - sizeof(ItcMemBlock) : 0;
	}

	//__END__;
}

/* moves stack pointer to next block.
   If no blocks, allocate new one and link it to the storage */
static void
itcGoNextMemBlock( ItcMemStorage * storage )
{
    //CV_FUNCNAME( "icvGoNextMemBlock" );
    
    //__BEGIN__;
    
    if( !storage )
        std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

    if( !storage->top || !storage->top->next )
    {
        ItcMemBlock *block;

        if( !(storage->parent) )
        {
            //CV_CALL( block = (CvMemBlock *)cvAlloc( storage->block_size ));
			block = (ItcMemBlock *)itcAlloc( storage->block_size );
        }
        else
        {
            ItcMemStorage *parent = storage->parent;
            ItcMemStoragePos parent_pos;

            itcSaveMemStoragePos( parent, &parent_pos );
			itcGoNextMemBlock(parent);
            //CV_CALL( icvGoNextMemBlock( parent ));

            block = parent->top;
            itcRestoreMemStoragePos( parent, &parent_pos );

            if( block == parent->top )  /* the single allocated block */
            {
                assert( parent->bottom == block );
                parent->top = parent->bottom = 0;
                parent->free_space = 0;
            }
            else
            {
                /* cut the block from the parent's list of blocks */
                parent->top->next = block->next;
                if( block->next )
                    block->next->prev = parent->top;
            }
        }

        /* link block */
        block->next = 0;
        block->prev = storage->top;

        if( storage->top )
            storage->top->next = block;
        else
            storage->top = storage->bottom = block;
    }

    if( storage->top->next )
        storage->top = storage->top->next;
    storage->free_space = storage->block_size - sizeof(ItcMemBlock);
    assert( storage->free_space % ITC_STRUCT_ALIGN == 0 );

    //__END__;
}

/* remembers memory storage position */
void
	itcSaveMemStoragePos( const ItcMemStorage * storage, ItcMemStoragePos * pos )
{
	//CV_FUNCNAME( "cvSaveMemStoragePos" );

	//__BEGIN__;

	if( !storage || !pos )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	pos->top = storage->top;
	pos->free_space = storage->free_space;

	//__END__;
}

/* restores memory storage position */
void
itcRestoreMemStoragePos( ItcMemStorage * storage, ItcMemStoragePos * pos )
{
   // CV_FUNCNAME( "cvRestoreMemStoragePos" );

    //__BEGIN__;

    if( !storage || !pos )
        std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;
    if( pos->free_space > storage->block_size )
		std::cout<<"err: "<<ITC_StsBadSize<<std::endl;
        //CV_ERROR( CV_StsBadSize, "" );

    /*
    // this breaks icvGoNextMemBlock, so comment it off for now
    if( storage->parent && (!pos->top || pos->top->next) )
    {
        CvMemBlock* save_bottom;
        if( !pos->top )
            save_bottom = 0;
        else
        {
            save_bottom = storage->bottom;
            storage->bottom = pos->top->next;
            pos->top->next = 0;
            storage->bottom->prev = 0;
        }
        icvDestroyMemStorage( storage );
        storage->bottom = save_bottom;
    }*/

    storage->top = pos->top;
    storage->free_space = pos->free_space;

    if( !storage->top )
    {
        storage->top = storage->bottom;
        storage->free_space = storage->top ? storage->block_size - sizeof(ItcMemBlock) : 0;
    }

    //__END__;
}

/* Allocates continuous buffer of the specified size in the storage */
void*
	itcMemStorageAlloc( ItcMemStorage* storage, size_t size )
{
	char *ptr = 0;

	//CV_FUNCNAME( "cvMemStorageAlloc" );

	//__BEGIN__;

	if( !storage )
		//CV_ERROR( CV_StsNullPtr, "NULL storage pointer" );
		std::cout<<"err: "<<ITC_StsNullPtr<<" Null storage pointer"<<std::endl;

	if( size > INT_MAX )
		//CV_ERROR( CV_StsOutOfRange, "Too large memory block is requested" );
		std::cout<<"err: "<<ITC_StsOutOfRange<<" Too large memory block is requested"<<std::endl;

	assert( storage->free_space % ITC_STRUCT_ALIGN == 0 );

	if( (size_t)storage->free_space < size )
	{
		size_t max_free_space = itcAlignLeft(storage->block_size - sizeof(ItcMemBlock), ITC_STRUCT_ALIGN);
		if( max_free_space < size )
			//CV_ERROR( CV_StsOutOfRange, "requested size is negative or too big" );
			std::cout<<"err: "<<ITC_StsOutOfRange<<" Too large memory block is requested"<<std::endl;

		//CV_CALL( icvGoNextMemBlock( storage ));
		itcGoNextMemBlock(storage);
	}

	ptr = ICV_FREE_PTR(storage);
	assert( (size_t)ptr % ITC_STRUCT_ALIGN == 0 );
	storage->free_space = itcAlignLeft(storage->free_space - (int)size, ITC_STRUCT_ALIGN );

	//__END__;

	return ptr;
}

/****************************************************************************************\
*                               Sequence implementation                                  *
\****************************************************************************************/

/* creates empty sequence */
ItcSeq *
	itcCreateSeq( int seq_flags, int header_size, int elem_size, ItcMemStorage * storage )
{
	ItcSeq *seq = 0;

	//CV_FUNCNAME( "cvCreateSeq" );

	//__BEGIN__;

	if( !storage )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;
	if( header_size < (int)sizeof( ItcSeq ) || elem_size <= 0 )
		std::cout<<"err: "<<ITC_StsBadSize<<std::endl;

	/* allocate sequence header */
	//CV_CALL( seq = (CvSeq*)cvMemStorageAlloc( storage, header_size ));
	seq=(ItcSeq*)itcMemStorageAlloc(storage,header_size);
	memset( seq, 0, header_size );

	seq->header_size = header_size;
	seq->flags = (seq_flags & ~ITC_MAGIC_MASK) | ITC_SEQ_MAGIC_VAL;
	{
		int elemtype = ITC_MAT_TYPE(seq_flags);
		int typesize = ITC_ELEM_SIZE(elemtype);

		if( elemtype != CV_SEQ_ELTYPE_GENERIC &&
			typesize != 0 && typesize != elem_size )
			std::cout<<"err: "<<ITC_StsBadSize<<"Specified element size doesn't match to the size of the specified element type (try to use 0 for element type)"<<std::endl;
			//CV_ERROR( CV_StsBadSize,
			//"Specified element size doesn't match to the size of the specified element type "
			//"(try to use 0 for element type)" );
	}
	seq->elem_size = elem_size;
	seq->storage = storage;

	//CV_CALL( cvSetSeqBlockSize( seq, (1 << 10)/elem_size ));
	itcSetSeqBlockSize(seq, (1 << 10)/elem_size );

	//__END__;

	return seq;
}

/* adjusts <delta_elems> field of sequence. It determines how much the sequence
   grows if there are no free space inside the sequence buffers */
void
itcSetSeqBlockSize( ItcSeq *seq, int delta_elements )
{
    int elem_size;
    int useful_block_size;

    //CV_FUNCNAME( "cvSetSeqBlockSize" );

    //__BEGIN__;
	
    if( !seq || !seq->storage )
        std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;
    if( delta_elements < 0 )
        std::cout<<"err: "<<ITC_StsBadSize<<std::endl;

    useful_block_size = itcAlignLeft(seq->storage->block_size - sizeof(ItcMemBlock) -
                                    sizeof(ItcSeqBlock), ITC_STRUCT_ALIGN);
    elem_size = seq->elem_size;

    if( delta_elements == 0 )
    {
        delta_elements = (1 << 10) / elem_size;
        delta_elements = ITC_MAX( delta_elements, 1 );
    }
    if( delta_elements * elem_size > useful_block_size )
    {
        delta_elements = useful_block_size / elem_size;
        if( delta_elements == 0 )
			std::cout<<"err: "<<ITC_StsBadSize<<"Storage block size is too small to fit the sequence elements"<<std::endl;
            //CV_ERROR( CV_StsOutOfRange, "Storage block size is too small "
                                       // "to fit the sequence elements" );
    }

    seq->delta_elems = delta_elements;

    //__END__;
}

/* finds sequence element by its index */
char*
	itcGetSeqElem( const ItcSeq *seq, int index )
{
	ItcSeqBlock *block;
	int count, total = seq->total;

	if( (unsigned)index >= (unsigned)total )
	{
		index += index < 0 ? total : 0;
		index -= index >= total ? total : 0;
		if( (unsigned)index >= (unsigned)total )
			return 0;
	}

	block = seq->first;
	if( index + index <= total )
	{
		while( index >= (count = block->count) )
		{
			block = block->next;
			index -= count;
		}
	}
	else
	{
		do
		{
			block = block->prev;
			total -= block->count;
		}
		while( index < total );
		index -= total;
	}

	return block->data + index * seq->elem_size;
}

/* calculates index of sequence element */
int
	itcSeqElemIdx( const ItcSeq* seq, const void* _element, ItcSeqBlock** _block )
{
	const char *element = (const char *)_element;
	int elem_size;
	int id = -1;
	ItcSeqBlock *first_block;
	ItcSeqBlock *block;

	//CV_FUNCNAME( "cvSeqElemIdx" );

	//__BEGIN__;

	if( !seq || !element )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;

	block = first_block = seq->first;
	elem_size = seq->elem_size;

	for( ;; )
	{
		if( (unsigned)(element - block->data) < (unsigned) (block->count * elem_size) )
		{
			if( _block )
				*_block = block;
			if( elem_size <= ITC_SHIFT_TAB_MAX && (id = itcPower2ShiftTab[elem_size - 1]) >= 0 )
				id = (int)((size_t)(element - block->data) >> id);
			else
				id = (int)((size_t)(element - block->data) / elem_size);
			id += block->start_index - seq->first->start_index;
			break;
		}
		block = block->next;
		if( block == first_block )
			break;
	}

	//__END__;

	return id;
}

/* pushes element to the sequence */
char*
	cvSeqPush( ItcSeq *seq, void *element )
{
	char *ptr = 0;
	size_t elem_size;

	//CV_FUNCNAME( "cvSeqPush" );

	//__BEGIN__;

	if( !seq )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;
		//CV_ERROR( CV_StsNullPtr, "" );

	elem_size = seq->elem_size;
	ptr = seq->ptr;

	if( ptr >= seq->block_max )
	{
		//CV_CALL( icvGrowSeq( seq, 0 ));
		itcGrowSeq( seq, 0 );
		ptr = seq->ptr;
		assert( ptr + elem_size <= seq->block_max /*&& ptr == seq->block_min */  );
	}

	if( element )
		CV_MEMCPY_AUTO( ptr, element, elem_size );
	seq->first->prev->count++;
	seq->total++;
	seq->ptr = ptr + elem_size;

	__END__;

	return ptr;
}


/* the function allocates space for at least one more sequence element.
   if there are free sequence blocks (seq->free_blocks != 0),
   they are reused, otherwise the space is allocated in the storage */
static void
itcGrowSeq( ItcSeq *seq, int in_front_of )
{
    //CV_FUNCNAME( "icvGrowSeq" );

    //__BEGIN__;

    ItcSeqBlock *block;

    if( !seq )
		std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;
        //CV_ERROR( CV_StsNullPtr, "" );
    block = seq->free_blocks;

    if( !block )
    {
        int elem_size = seq->elem_size;
        int delta_elems = seq->delta_elems;
        ItcMemStorage *storage = seq->storage;

        if( seq->total >= delta_elems*4 )
            itcSetSeqBlockSize( seq, delta_elems*2 );

        if( !storage )
			std::cout<<"err: "<<ITC_StsNullPtr<<std::endl;
            //CV_ERROR( CV_StsNullPtr, "The sequence has NULL storage pointer" );

        /* if there is a free space just after last allocated block
           and it's big enough then enlarge the last block
           (this can happen only if the new block is added to the end of sequence */
        if( (unsigned)(ICV_FREE_PTR(storage) - seq->block_max) < ITC_STRUCT_ALIGN &&
            storage->free_space >= seq->elem_size && !in_front_of )
        {
            int delta = storage->free_space / elem_size;

            delta = ITC_MIN( delta, delta_elems ) * elem_size;
            seq->block_max += delta;
            storage->free_space = itcAlignLeft((int)(((char*)storage->top + storage->block_size) -
                                              seq->block_max), ITC_STRUCT_ALIGN );
            //EXIT;
        }
        else
        {
            int delta = elem_size * delta_elems + (int)itcAlign(sizeof(ItcSeqBlock), ITC_STRUCT_ALIGN);

            /* try to allocate <delta_elements> elements */
            if( storage->free_space < delta )
            {
                int small_block_size = ITC_MAX(1, delta_elems/3)*elem_size +
                                       (int)itcAlign(sizeof(ItcSeqBlock), ITC_STRUCT_ALIGN);
                /* try to allocate smaller part */
                if( storage->free_space >= small_block_size + ITC_STRUCT_ALIGN )
                {
                    delta = (storage->free_space - (int)itcAlign(sizeof(ItcSeqBlock), ITC_STRUCT_ALIGN))/seq->elem_size;
                    delta = delta*seq->elem_size + (int)itcAlign(sizeof(ItcSeqBlock), ITC_STRUCT_ALIGN);
                }
                else
                {
                    //CV_CALL( icvGoNextMemBlock( storage ));
					itcGoNextMemBlock(storage);
                    assert( storage->free_space >= delta );
                }
            }

            //CV_CALL( block = (CvSeqBlock*)cvMemStorageAlloc( storage, delta ));
			block = (ItcSeqBlock*)itcMemStorageAlloc( storage, delta );
            block->data = (char*)itcAlignPtr( block + 1, ITC_STRUCT_ALIGN );
            block->count = delta - (int)itcAlign(sizeof(ItcSeqBlock), ITC_STRUCT_ALIGN);
            block->prev = block->next = 0;
        }
    }
    else
    {
        seq->free_blocks = block->next;
    }

    if( !(seq->first) )
    {
        seq->first = block;
        block->prev = block->next = block;
    }
    else
    {
        block->prev = seq->first->prev;
        block->next = seq->first;
        block->prev->next = block->next->prev = block;
    }

    /* for free blocks the <count> field means total number of bytes in the block.
       And for used blocks it means a current number of sequence
       elements in the block */
    assert( block->count % seq->elem_size == 0 && block->count > 0 );

    if( !in_front_of )
    {
        seq->ptr = block->data;
        seq->block_max = block->data + block->count;
        block->start_index = block == block->prev ? 0 :
            block->prev->start_index + block->prev->count;
    }
    else
    {
        int delta = block->count / seq->elem_size;
        block->data += block->count;

        if( block != block->prev )
        {
            assert( seq->first->start_index == 0 );
            seq->first = block;
        }
        else
        {
            seq->block_max = seq->ptr = block->data;
        }

        block->start_index = 0;

        for( ;; )
        {
            block->start_index += delta;
            block = block->next;
            if( block == seq->first )
                break;
        }
    }

    block->count = 0;

    //__END__;
}
//int
//itcSliceLength( CvSlice slice, const CvSeq* seq )
//{
//    int total = seq->total;
//    int length = slice.end_index - slice.start_index;
//    
//    if( length != 0 )
//    {
//        if( slice.start_index < 0 )
//            slice.start_index += total;
//        if( slice.end_index <= 0 )
//            slice.end_index += total;
//
//        length = slice.end_index - slice.start_index;
//    }
//
//    if( length < 0 )
//    {
//        length += total;
//        /*if( length < 0 )
//            length += total;*/
//    }
//    else if( length > total )
//        length = total;
//
//    return length;
//}
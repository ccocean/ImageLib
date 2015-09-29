#include "itctypes.h"
<<<<<<< HEAD
#include "limits.h"
#include <assert.h>
#include <iostream>


/* maximum size of dynamic memory buffer.
   cvAlloc reports an error if a larger block is requested. */
#define  ITC_MAX_ALLOC_SIZE    (((size_t)1 << (sizeof(size_t)*8-2)))
/* default storage block size */
#define  ITC_STORAGE_BLOCK_SIZE   ((1<<16) - 128)
/* default alignment for dynamic data strucutures, resided in storages. */
#define  ITC_STRUCT_ALIGN    ((int)sizeof(double))

//error code
#define  ITC_StsOutOfRange -211  /* some of parameters are out of range */
#define  ITC_StsNoMem -4	/* insufficient memory */
#define  ITC_StsNullPtr  -27 /* null pointer */

/* the alignment of all the allocated buffers */
#define  ITC_MALLOC_ALIGN    32

// pointers to allocation functions, initially set to default
static void* p_cvAllocUserData = 0;

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

// default <malloc>
static void*
	icvDefaultAlloc( size_t size, void* )
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

// pointers to allocation functions, initially set to default
//static CvAllocFunc p_cvAlloc = icvDefaultAlloc;

void*  itcAlloc( size_t size )
{
	void* ptr = 0;

	//CV_FUNCNAME( "cvAlloc" );

	//__BEGIN__;

	if( (size_t)size > ITC_MAX_ALLOC_SIZE )
		std::cout<<"err: "<<ITC_StsOutOfRange<<std::endl;

	ptr = icvDefaultAlloc( size, p_cvAllocUserData );
	if( !ptr )
		std::cout<<"err: "<<ITC_StsNoMem<<std::endl;

	//__END__;

	return ptr;
}

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
=======

inline int  itcAlign(int size, int align)
{
	assert((align&(align-1))==0 && size<INT_MAX)
>>>>>>> 7d1bf42daf64f82f67817f5f1502b34738b2ab03
}
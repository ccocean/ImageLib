#include "itctypes.h"

inline int  itcAlign(int size, int align)
{
	assert((align&(align-1))==0 && size<INT_MAX)
}
#ifndef _zf_common_function_h_
#define _zf_common_function_h_

#include "zf_common_typedef.h"

#define func_abs(x)             ((x) >= 0 ? (x): -(x))
#define func_limit(x, y)        ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
#define func_limit_ab(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

#endif

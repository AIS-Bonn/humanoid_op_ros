// likely/unlikely macros for optimization
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_UTIL_LIKELY_H
#define DYNALIB_UTIL_LIKELY_H

#define unlikely(x) __builtin_expect(!!(x), 0)
#define likely(x) __builtin_expect(!!(x), 1)

#endif

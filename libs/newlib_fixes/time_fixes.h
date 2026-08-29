#ifndef TIME_FIXES_H_
#define TIME_FIXES_H_

#include "time.h"

int __secs_to_tm(long long t, struct tm* tm);

/* int __secs_to_tm(long long t, struct tm* tm); */
#define gmtime_ex __secs_to_tm

#endif

#ifndef __orogen
#include <stdint.h>
#endif

namespace Test {
    struct Simple {
	int a;
	char b[20];
    };

    struct Timestamp {
	uint32_t sec;
	uint32_t usec;
    };
}


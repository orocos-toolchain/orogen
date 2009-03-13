#ifndef __orogen
#include <stdint.h>
#endif

namespace Test {
    enum SIMPLE_ENUM {
        VALUE_0,
        VALUE_1,
        VALUE_100 = 100
    };

    struct Simple {
	int          i;
        SIMPLE_ENUM  e;
	char         a[20];

#ifndef __orogen
        bool operator == (Simple const& other) const
        { 
            if (other.i != i)
                return false;
            if (other.e != e)
                return false;

            for (int it = 0; it < 20; ++it)
            {
                if (other.a[i] != a[i])
                    return false;
            }
            return true;
        }
#endif
    };

    struct Timestamp {
	uint32_t sec;
	uint32_t usec;
    };
}


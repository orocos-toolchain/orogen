#ifndef __orogen
#include <stdint.h>
#endif

namespace Test {
    struct Simple {
	int a;
	char b[20];

#ifndef __orogen
        bool operator == (Simple const& other) const
        { 
            if (other.a != a)
                return false;

            for (int i = 0; i < 20; ++i)
            {
                if (other.b[i] != b[i])
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


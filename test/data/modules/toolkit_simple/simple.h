#ifndef __orogen
#include <stdint.h>
#endif

namespace Test {
    enum BASIC_ENUM {
        VALUE_0,
        VALUE_1,
	VALUE_20  = 20,
        VALUE_100 = 100
    };

    struct BaseTypes {
        bool               v0;
        char               v1;
        unsigned char      v2;
        short              v3;
        unsigned short     v4;
        int                v5;
        unsigned int       v6;
        long               v7;
        unsigned long      v8;
        long long          v9;
        unsigned long long v10;

        BASIC_ENUM        e;
	char               a[20];

#ifndef __orogen
        bool operator == (BaseTypes const& other) const
        { 
	    if (other.v0 != v0) return false;
	    if (other.v1 != v1) return false;
	    if (other.v2 != v2) return false;
	    if (other.v3 != v3) return false;
	    if (other.v4 != v4) return false;
	    if (other.v5 != v5) return false;
	    if (other.v6 != v6) return false;
	    if (other.v7 != v7) return false;
	    if (other.v8 != v8) return false;
	    if (other.v9 != v9) return false;
	    if (other.v10 != v10) return false;

	    if (other.e != e) return false;

            for (int it = 0; it < 20; ++it)
            {
                if (other.a[it] != a[it])
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

    struct Image {
        std::vector<uint8_t> data;
    };
}


#ifndef TYPEKIT_SIMPLE_SIMPLE_H
#define TYPEKIT_SIMPLE_SIMPLE_H

#include <stdint.h>
#include <vector>

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

        BASIC_ENUM        e;
	char               a[20];

        bool operator == (BaseTypes const& other) const
        { 
	    if (other.v0 != v0) return false;
	    if (other.v1 != v1) return false;
	    if (other.v2 != v2) return false;
	    if (other.v3 != v3) return false;
	    if (other.v4 != v4) return false;
	    if (other.v5 != v5) return false;
	    if (other.v6 != v6) return false;

	    if (other.e != e) return false;

            for (int it = 0; it < 20; ++it)
            {
                if (other.a[it] != a[it])
                    return false;
            }
            return true;
        }
    };

    struct TestArrayOfDifferentSizes
    {
        int a[10];
        int b[20];
    };

    struct Test64BitHandling
    {
        BaseTypes base;
        long long ll;
        unsigned long long ull;

        bool operator == (Test64BitHandling const& other) const
        { 
	    if (!(other.base == base)) return false;
	    if (other.ll != ll) return false;
	    if (other.ull != ull) return false;
            return true;
        }

    };

    struct SimpleVector {
        uint32_t field;
        std::vector<uint8_t> data;
        bool operator == (SimpleVector const& other) const
        { return field == other.field && data == other.data; }
        bool operator != (SimpleVector const& other) const
        { return !(*this == other); }
    };

    struct ComplexVector
    {
        uint32_t field;
        std::vector<SimpleVector> data;
        bool operator == (ComplexVector const& other) const
        {
            return field == other.field && data == other.data;
        }
    };

    struct ComplexArray
    {
        uint32_t field;
        SimpleVector data[10];
        bool operator == (ComplexArray const& other) const
        {
            if (field != other.field) return false;
            for (int i = 0; i < 10; ++i)
                if (!(data[i] == other.data[i])) return false;
            return true;
        }
    };
}

#endif

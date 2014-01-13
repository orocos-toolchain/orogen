#include <vector>

struct StructWithVectorBool
{
    std::vector<bool> value;
};

typedef std::vector< std::vector<bool> > VectorWithVectorBool;

struct __gccxml_bla {
    VectorWithVectorBool field;
};


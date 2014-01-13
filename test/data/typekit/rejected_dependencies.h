#include <vector>
struct CompoundTest
{
    double* reject;
};
struct ArrayTest
{
    CompoundTest reject[10];
};
struct VectorTest
{
    std::vector<CompoundTest> reject;
};

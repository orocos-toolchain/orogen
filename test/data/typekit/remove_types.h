#ifndef OROGEN_TESTS_REMOVE_TYPES_H
#define OROGEN_TESTS_REMOVE_TYPES_H

#include <vector>

struct Base {
    int field;
};

struct Field {
    int field;
};

struct Derived : Base {
    std::vector<Field> field;
};

typedef Base BaseTypedef;
typedef Derived DerivedTypedef;

#endif

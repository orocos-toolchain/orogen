#include "Test.hpp"

using namespace numerics;

Test::Test(std::string const& name)
    : TestBase(name) {}


int Test::m1(short a, unsigned short b, char c, unsigned char d) {
    return 0;
}

void Test::m2(int a, unsigned int b, long c, unsigned long d) {
}

void Test::m3(long long a, unsigned long long b, float c, double d) {
}






/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Test.hpp for more detailed
// documentation about them.

// bool Test::configureHook() { return true; }
bool Test::startHook()
{
    scripting()->loadPrograms("test.script");
    return false;
}
// void Test::updateHook() {}
// void Test::errorHook() {}
// void Test::stopHook() {}
// void Test::cleanupHook() {}


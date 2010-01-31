#include "Test.hpp"

using namespace numerics;

Test::Test(std::string const& name, TaskCore::TaskState initial_state)
    : TestBase(name, initial_state)
{
}


void Test::m2(int a, unsigned int b)
{
}

void Test::m3(double d)
{
}

void Test::m4(numerics::Bla const& e)
{
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


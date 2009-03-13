#include "Producer.hpp"
#include <iostream>

using namespace std;
using namespace data;

Producer::Producer(std::string const& name)
    : ProducerBase(name) {}







/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Producer.hpp for more detailed
// documentation about them.

// bool Producer::configureHook() { return true; }
// bool Producer::startHook() { return true; }

void Producer::updateHook()
{
    static int idx = 0;
    ++idx;

    if (idx == 11) // finish on a non-triggering write to have a more deterministic test
        exit(0);

    if (idx % 2 == 0)
        _output.write(idx);
    else _output2.write(idx);
}

// void Producer::errorHook() {}
// void Producer::stopHook() {}
// void Producer::cleanupHook() {}


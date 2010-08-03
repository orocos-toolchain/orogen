#include "Consumer.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using namespace data;

Consumer::Consumer(std::string const& name, TaskCore::TaskState initial_state)
    : ConsumerBase(name, initial_state) {}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Consumer.hpp for more detailed
// documentation about them.

// bool Consumer::configureHook() { return true; }
bool Consumer::startHook() {
    outfile = new ofstream("data_trigger.txt");
    return true;
}

void Consumer::updateHook()
{
    double value;
    bool got_data = false;
    while (_input.read(value) == RTT::NewData)
    {
        got_data = true;
        *outfile << value << " " << flush;
    }

    if (!got_data)
        *outfile << "U " << flush;
}

// void Consumer::errorHook() {}
void Consumer::stopHook()
{
    delete outfile;
}
// void Consumer::cleanupHook() {}


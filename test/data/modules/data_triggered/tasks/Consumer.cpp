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

void Consumer::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    if (find(updated_ports.begin(), updated_ports.end(), &_input) != updated_ports.end())
    {
        double value;
        while (_input.read(value))
            *outfile << value << " " << flush;
    }
    else *outfile << "U " << flush;
}

// void Consumer::errorHook() {}
void Consumer::stopHook()
{
    delete outfile;
}
// void Consumer::cleanupHook() {}


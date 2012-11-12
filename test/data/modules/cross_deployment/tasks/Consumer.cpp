#include "Consumer.hpp"

using namespace cross_deployment;
using namespace std;

Consumer::Consumer(std::string const& name, TaskCore::TaskState initial_state)
    : ConsumerBase(name, initial_state) {}
Consumer::Consumer(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ConsumerBase(name, engine, initial_state) {}







/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Consumer.hpp for more detailed
// documentation about them.

// bool Consumer::configureHook() { return true; }
// bool Consumer::startHook() { return true; }

void Consumer::updateHook()
{
    NotOrogenCompatible::Point2D point;
    if (_point.read(point))
        *io << "[" << point.x() << " " << point.y() << "] " << flush;
    else
        *io << "[U] " << flush;
}

// void Consumer::errorHook() {}
// void Consumer::stopHook() {}
// void Consumer::cleanupHook() {}


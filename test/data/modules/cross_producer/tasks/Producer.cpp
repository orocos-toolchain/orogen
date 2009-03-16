#include "Producer.hpp"

using namespace cross_producer;

Producer::Producer(std::string const& name, TaskCore::TaskState initial_state)
    : ProducerBase(name, initial_state) {}







/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Producer.hpp for more detailed
// documentation about them.

// bool Producer::configureHook() { return true; }
// bool Producer::startHook() { return true; }

void Producer::updateHook()
{
    static int i = 0;
    NotOrogenCompatible::Point2D point;
    point.x() = ++i;
    point.y() = ++i;
    _point.write(point);
    if (i == 20)
        exit(0);
}

// void Producer::errorHook() {}
// void Producer::stopHook() {}
// void Producer::cleanupHook() {}


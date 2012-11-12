#include "Consumer.hpp"
#include <fstream>

using namespace cross_consumer;

Consumer::Consumer(std::string const& name, TaskCore::TaskState initial_state)
    : ConsumerBase(name, initial_state)
{
    io = new std::ofstream("cross_dependencies.txt");
}

Consumer::Consumer(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ConsumerBase(name, engine, initial_state)
{
    io = new std::ofstream("cross_dependencies.txt");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Consumer.hpp for more detailed
// documentation about them.

// bool Consumer::configureHook() { return true; }
// bool Consumer::startHook() { return true; }
// void Consumer::errorHook() {}
// void Consumer::stopHook() {}
// void Consumer::cleanupHook() {}


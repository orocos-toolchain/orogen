#include "Configurable.hpp"

using namespace with_configuration;

Configurable::Configurable(std::string const& name)
    : ConfigurableBase(name) {}







/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Configurable.hpp for more detailed
// documentation about them.

// bool Configurable::configureHook() { return true; }
// bool Configurable::startHook() { return true; }

// void Configurable::updateHook() {}

// void Configurable::errorHook() {}
// void Configurable::stopHook() {}
// void Configurable::cleanupHook() {}


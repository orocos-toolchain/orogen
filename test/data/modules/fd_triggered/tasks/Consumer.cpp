#include "Consumer.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <rtt/FileDescriptorActivity.hpp>

using namespace std;
using namespace fd;

RTT::FileDescriptorActivity* Consumer::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }

Consumer::Consumer(std::string const& name)
    : ConsumerBase(name)
{
    // The test suite must create a pipe and give us the FD for the read side
    // through the FD_DRIVEN_TEST_FILE environment variable.
    m_fd = boost::lexical_cast<int>(getenv("FD_DRIVEN_TEST_FILE"));
}

Consumer::~Consumer()
{
    close(m_fd);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Consumer.hpp for more detailed
// documentation about them.

bool Consumer::configureHook() {
    getFileDescriptorActivity()->watch(m_fd);
    return TaskContext::configureHook();
}

void Consumer::updateHook()
{
    static int idx = 'A';

    // We should have one byte on stdin
    char byte;
    if (read(m_fd, &byte, 1) != 1)
    {
        cerr << "ERROR: error reading input" << endl;
        exit(1);
    }
    if (byte != idx)
    {
        cerr << "ERROR: expected " << (char)idx << ", got " << byte << endl;
        exit(1);
    }

    if (++idx == 'F')
        exit(0);
}

// void Consumer::errorHook() {}
// void Consumer::stopHook() {}
// void Consumer::cleanupHook() {}


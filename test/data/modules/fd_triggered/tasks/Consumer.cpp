#include "Consumer.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>

using namespace std;
using namespace fd;

Consumer::Consumer(std::string const& name)
    : ConsumerBase(name)
    , m_fd(-1)
{
    // The test suite must create a pipe and give us the FD for the read side
    // through the FD_DRIVEN_TEST_FILE environment variable.
    char const* fd = getenv("FD_DRIVEN_TEST_FILE");
    if (fd)
        m_fd = boost::lexical_cast<int>(fd);
    else
        std::cerr << "FD_DRIVEN_TEST_FILE is not set" << std::endl;
}

Consumer::~Consumer()
{
    close(m_fd);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Consumer.hpp for more detailed
// documentation about them.

bool Consumer::configureHook() {
    if (m_fd != -1)
        getActivity<RTT::extras::FileDescriptorActivity>()->watch(m_fd);
    return TaskContext::configureHook();
}

void Consumer::updateHook()
{
    static int idx = 'A';

    // We should have one byte on stdin
    char byte;
    int ret = read(m_fd, &byte, 1);
    if (ret == -1)
    {
        cerr << "ERROR: error reading input" << endl;
        exit(1);
    }
    else if (ret == 0)
    {
        return;
    }
    else if (byte != idx)
    {
        cerr << "ERROR: expected " << (char)idx << ", got " << byte << endl;
        exit(1);
    }
    ++idx;
}

// void Consumer::errorHook() {}
// void Consumer::stopHook() {}
// void Consumer::cleanupHook() {}


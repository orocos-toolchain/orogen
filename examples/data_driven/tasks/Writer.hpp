#ifndef DATADRIVEN_WRITER_TASK_HPP
#define DATADRIVEN_WRITER_TASK_HPP

#include "datadriven/WriterBase.hpp"

namespace DataDriven {
    class Writer : public WriterBase
    {
	friend class WriterBase;
    protected:
    
    

    public:
        Writer(std::string const& name = "Writer");
        void updateHook();
    };
}

#endif


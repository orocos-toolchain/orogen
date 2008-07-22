#ifndef DATADRIVEN_BUFFERREADER_TASK_HPP
#define DATADRIVEN_BUFFERREADER_TASK_HPP

#include "datadriven/BufferReaderBase.hpp"

namespace DataDriven {
    class BufferReader : public BufferReaderBase
    {
	friend class BufferReaderBase;
    protected:
    
    

    public:
        BufferReader(std::string const& name = "BufferReader");
        void updateHook();
    };
}

#endif


#ifndef DATADRIVEN_DATAREADER_TASK_HPP
#define DATADRIVEN_DATAREADER_TASK_HPP

#include "datadriven/DataReaderBase.hpp"

namespace DataDriven {
    class DataReader : public DataReaderBase
    {
	friend class DataReaderBase;
    protected:
    
    

    public:
        DataReader(std::string const& name = "DataReader");
        void updateHook();
    };
}

#endif


#include "DataReader.hpp"

using namespace DataDriven;

DataReader::DataReader(std::string const& name)
    : DataReaderBase(name) {}

void DataReader::updateHook()
{
    std::cout << "Data: " << _input.Get() << std::endl;
}







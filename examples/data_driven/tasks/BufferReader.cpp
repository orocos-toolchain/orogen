#include "BufferReader.hpp"

using namespace DataDriven;

BufferReader::BufferReader(std::string const& name)
    : BufferReaderBase(name) {}


void BufferReader::updateHook()
{
    int value;
    if (!_input.Pop(value))
        std::cout << "Empty buffer ?" << std::endl;
    else
        std::cout << "Buffer: " << value << std::endl;
}






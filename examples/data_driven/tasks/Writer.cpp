#include "Writer.hpp"

using namespace DataDriven;

Writer::Writer(std::string const& name)
    : WriterBase(name) {}


void Writer::updateHook()
{
    static int value = 0;
    _data.Set(++value);
    _buffer.Push(value);
}






#include "TestToolkitTypes.hpp"
#include "TestToolkit.hpp"

#include <rtt/os/main.h>
#include <rtt/Types.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/DataSources.hpp>
#include <rtt/marsh/XMLMarshaller.hpp>
#include <rtt/marsh/PropertyMarshaller.hpp>
#include <rtt/marsh/PropertyDemarshaller.hpp>
#include <iostream>
#include <fstream>

using namespace RTT;
using std::cerr;
using std::endl;

int ORO_main(int argc, char** argv)
{
    RTT::Toolkit::Import( Test::Toolkit );

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();

    TypeInfo* type = ti->type("Simple");
    if (! type)
    {
	cerr << "cannot find the type in the type info repository" << endl;
	return 1;
    }

    // Create a structure of type Test::Simple, inject it into Orocos and check
    // that it is able to generate a XML representation of it
    Test::Simple value = { 10, "" };
    for (int i = 0; i < 20; ++i)
	value.b[i] = 'a' + i;

    ConstantDataSource<Test::Simple>* source = new ConstantDataSource<Test::Simple>(value);

    PropertyBag bag;
    type->decomposeType(source, bag);

    // First, save it into XML. The Ruby test case will compare that to an
    // expected XML document
    std::ofstream xml_file("test_toolkit.xml");
    XMLMarshaller<std::ostream> xml_output(xml_file);
    xml_output.serialize(bag);

    // Now, marshal it to the standard Orocos format, reload it and compare
    // the result
    PropertyMarshaller cpf_output("test_toolkit.cpf");
    cpf_output.serialize(bag);
    cpf_output.flush();

    PropertyBag input_bag;
    PropertyDemarshaller cpf_input("test_toolkit.cpf");
    cpf_input.deserialize(input_bag);

    int input_a = Property<int>(input_bag.getItem(0)).value();
    if (input_a != 10)
    {
	cerr << "input.a == " << input_a << ", 10 expected" << endl;
	return 1;
    }

    return 0;
}


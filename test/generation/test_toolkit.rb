require 'orocos/generation/test'
require 'orocos/generation/toolkit'

class TC_GenerationToolkit < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'generation', 'data' )

    def test_orocos_type_equivalence
	registry = Typelib::Registry.new

	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('int32_t')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('uint32_t')))
	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('int16_t')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('uint16_t')))
	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('short')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('unsigned short')))

	assert_raises(TypeError) { registry.orocos_equivalent(registry.get('int64_t')) }
    end

    def test_toolkit_generation(with_corba = true)
	component = Generation.new do
	    toolkit('Test') do
		load File.join(TEST_DATA_DIR, 'type_info_generation.h')	
		disable_corba unless with_corba
	    end
	end

	copy_in_wc File.join(TEST_DATA_DIR, 'test_toolkit.cpp')
	in_wc do
	    component.generate
	    if with_corba
		assert(File.file?( File.join('.orocos', 'toolkit', 'TestToolkitCorba.hpp')))
		assert(File.file?( File.join('.orocos', 'toolkit', 'TestToolkit.idl')))
	    else
		assert(!File.file?( File.join('.orocos', 'toolkit', 'TestToolkitCorba.hpp')))
		assert(!File.file?( File.join('.orocos', 'toolkit', 'TestToolkit.idl')))
	    end
	    assert(File.file?( File.join('.orocos', 'toolkit', 'TestToolkit.hpp')))
	    assert(File.file?( File.join('.orocos', 'toolkit', 'TestToolkit.cpp')))
	end

	compile_wc(component) do
	    File.open('CMakeLists.txt', 'a') do |io|
		io << "\nADD_EXECUTABLE(test_toolkit test_toolkit.cpp)"
		io << "\nTARGET_LINK_LIBRARIES(test_toolkit Test-toolkit-${OROCOS_TARGET})"
		io << "\nTARGET_LINK_LIBRARIES(test_toolkit ${OROCOS_COMPONENT_LIBRARIES})"
		io << "\n"
	    end
	end

	in_wc do
	    output = nil
	    Dir.chdir("build") do
		assert(system("./test_toolkit"))
		output = File.read('test_toolkit.xml')
	    end

	    expected = File.read(File.join(TEST_DATA_DIR, 'simple_value.xml'))
	    assert_equal(expected, output)
	end
    end
    def test_disable_corba; test_toolkit_generation(false) end
end


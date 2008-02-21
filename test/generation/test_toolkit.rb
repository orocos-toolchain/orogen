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

    def test_to_orocos_toolkit
	registry = Typelib::Registry.new
	registry.import File.join(TEST_DATA_DIR, 'type_info_generation.h')

	header, source = nil
	assert_nothing_raised { header, source = registry.to_orocos_toolkit("Test") }
	assert(! header.empty?)
	assert(! source.empty?)
	assert( source =~ /SimpleTypeInfo/ )
    end

    def test_toolkit_generation
	copy_in_wc File.join(TEST_DATA_DIR, 'type_info_generation.h')
	in_wc do
	    Generation.toolkit('Test') do
		load 'type_info_generation.h'
	    end

	    assert(File.file?( File.join('.orocos', 'toolkit', 'TestToolkit.hpp')))
	    assert(File.file?( File.join('.orocos', 'toolkit', 'TestToolkit.cpp')))
	end
	compile_wc('Test')
    end
end


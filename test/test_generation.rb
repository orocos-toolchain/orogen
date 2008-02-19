$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'test/unit'
require 'orocos/generation/toolkit'

class TC_Generation < Test::Unit::TestCase
    TEST_DATA_DIR = File.join( File.dirname(__FILE__), 'data' )

    def test_load_template
	erb = nil
	assert_nothing_raised { erb = Generation.load_template('toolkit/type_info.cpp') }
	assert_kind_of(ERB, erb)

	assert_same(erb, Generation.load_template('toolkit/type_info.cpp'))
    end

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

	STDERR.puts header
	STDERR.puts source
    end
end

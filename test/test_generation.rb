$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'test/unit'
require 'orocos/generation'

class TC_Generation < Test::Unit::TestCase
    TEST_DATA_DIR = File.join( File.dirname(__FILE__), 'data' )
    def test_load_template
	erb = nil
	assert_nothing_raised { erb = Generation.load_template('type_info.cpp') }
	assert_kind_of(ERB, erb)

	assert_same(erb, Generation.load_template('type_info.cpp'))
    end

    def test_orocos_type_equivalence
	registry = Typelib::Registry.new
	registry.build_orocos_type_equivalence

	assert_equal(registry.get('int'), registry.orocos_type_equivalence[registry.get('int32_t')])
	assert_equal(registry.get('unsigned int'), registry.orocos_type_equivalence[registry.get('uint32_t')])
	assert_equal(registry.get('int'), registry.orocos_type_equivalence[registry.get('int16_t')])
	assert_equal(registry.get('unsigned int'), registry.orocos_type_equivalence[registry.get('uint16_t')])
	assert_equal(registry.get('int'), registry.orocos_type_equivalence[registry.get('short')])
	assert_equal(registry.get('unsigned int'), registry.orocos_type_equivalence[registry.get('unsigned short')])

	assert_raises(TypeError) { registry.orocos_equivalent(registry.get('int64_t')) }
    end


    def test_to_orocos_toolkit
	registry = Typelib::Registry.new
	registry.import File.join(TEST_DATA_DIR, 'type_info_generation.h')

	result = ""
	assert_nothing_raised { registry.to_orocos_toolkit(result) }
	assert(! result.empty?)
	assert( result =~ /SimpleTypeInfo/ )
	STDERR.puts result
    end
end

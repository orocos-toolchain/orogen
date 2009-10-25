$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationBase < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    def test_load_template
	assert_raises(ArgumentError) { Generation.load_template('does', 'not', 'exist') }

	erb = nil
	assert_nothing_raised { erb = Generation.load_template('toolkit', 'Toolkit.cpp') }
	assert_kind_of(ERB, erb)

	assert_same(erb, Generation.load_template('toolkit', 'Toolkit.cpp'))

	other_erb = nil
	assert_nothing_raised { other_erb = Generation.load_template('CMakeLists.txt') }
	assert_not_same(other_erb, erb)
    end

    def test_save_user
        create_wc "base"
	assert_raises(ArgumentError) { Orocos::Generation.save_user }

	in_wc do
	    Generation.save_user 'test.cpp', "blabla"
	end

	target_file = File.join(working_directory, 'test.cpp')
	assert_equal("blabla", File.read(target_file))

	in_wc do
	    Generation.save_user 'subdir', 'test.cpp', "bloblo"
	end

	target_file = File.join(working_directory, 'subdir', 'test.cpp')
	assert_equal("bloblo", File.read(target_file))
    end

    def test_save_automatic
        create_wc "base"
	assert_raises(ArgumentError) { Orocos::Generation.save_automatic }

	in_wc do
	    Generation.save_automatic 'test.cpp', "blabla"
	end

	target_file = File.join(working_directory, Generation::AUTOMATIC_AREA_NAME, 'test.cpp')
	assert_equal("blabla", File.read(target_file))

	in_wc do
	    Generation.save_automatic 'subdir', 'test.cpp', "bloblo"
	end

	target_file = File.join(working_directory, Generation::AUTOMATIC_AREA_NAME, 'subdir', 'test.cpp')
	assert_equal("bloblo", File.read(target_file))
    end

    def test_adapt_namespace
	assert_equal("    }\n", Generation.adapt_namespace('/A/B', '/A'))
	assert_equal("        }\n    }\n    namespace D {\n", Generation.adapt_namespace('/A/B/C', '/A/D'))
	assert_equal("", Generation.adapt_namespace('/A/B/C', '/A/B/C'))
    end

    def test_dsl_attribute
	cl = Class.new do
            def initialize
                @no_filter = nil
                @filter_integer = nil
                @filter_string  = nil
            end
        end

	cl.dsl_attribute :no_filter
	obj = cl.new
	assert(obj.respond_to?(:no_filter))
	assert_equal(nil, obj.no_filter)
	obj.no_filter 10
	assert_equal(10, obj.no_filter)
        obj.no_filter :bla, :blo
	assert_equal([:bla, :blo], obj.no_filter)

	cl.dsl_attribute :filter_integer do |value|
	    Integer(value)
	end
	cl.dsl_attribute :filter_string do |value|
	    if value.respond_to?(:to_str)
		value.to_str
	    else
		raise
	    end
	end

	2.times do
	    obj = cl.new
	    assert(obj.respond_to?(:no_filter))
	    assert(obj.respond_to?(:filter_integer))
	    assert(obj.respond_to?(:filter_string))

	    assert_equal(nil, obj.filter_integer)
	    assert_equal(obj, obj.filter_integer(10))
	    assert_equal(10, obj.filter_integer)
	    assert_raises(ArgumentError) { obj.filter_integer "v" }
	    assert_raises(ArgumentError) { obj.filter_integer :bla, :blo }

	    assert_equal(nil, obj.filter_string)
	    assert_equal(obj, obj.filter_string("10"))
	    assert_equal("10", obj.filter_string)
	    assert_raises(RuntimeError) { obj.filter_string 10 }
	    cl = Class.new(cl)
	end
    end

    def test_component_generate
        create_wc "base"
	in_wc do
            # No name, no orogen file
	    component = Generation::Component.new
	    assert_raises(ArgumentError) { component.generate }

            # No orogen file
	    component = Generation::Component.new
	    component.name "cmp"
	    assert_raises(ArgumentError) { component.generate } 
            
            # No name
	    component = Generation::Component.new
	    component.instance_variable_set(:@deffile, File.join(TEST_DATA_DIR, "empty_component.orogen"))
	    assert_raises(ArgumentError) { component.generate } 

            # OK
	    component = Generation::Component.new
	    component.load(File.join(TEST_DATA_DIR, "empty_component.orogen"))
	    assert_nothing_raised { component.generate } 
	end
    end
end


$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orocos/generation/test'
require 'orocos/generation/base'

class TC_GenerationBase < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'generation', 'data' )

    def test_load_template
	assert_raises(ArgumentError) { Generation.load_template('does', 'not', 'exist') }

	erb = nil
	assert_nothing_raised { erb = Generation.load_template('toolkit', 'type_info.cpp') }
	assert_kind_of(ERB, erb)

	assert_same(erb, Generation.load_template('toolkit', 'type_info.cpp'))

	other_erb = nil
	assert_nothing_raised { other_erb = Generation.load_template('build', 'CMakeLists.txt') }
	assert_not_same(other_erb, erb)
    end

    def test_save_user
	base_file = File.join(TEST_DATA_DIR, 'type_info_generation.h')
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
	base_file = File.join(TEST_DATA_DIR, 'type_info_generation.h')
	assert_raises(ArgumentError) { Orocos::Generation.save_automatic }

	in_wc do
	    Generation.save_automatic 'test.cpp', "blabla"
	end

	target_file = File.join(working_directory, '.orocos', 'test.cpp')
	assert_equal("blabla", File.read(target_file))

	in_wc do
	    Generation.save_automatic 'subdir', 'test.cpp', "bloblo"
	end

	target_file = File.join(working_directory, '.orocos', 'subdir', 'test.cpp')
	assert_equal("bloblo", File.read(target_file))
    end

    def test_adapt_namespace
	assert_equal("    }\n", Generation.adapt_namespace('/A/B', '/A'))
	assert_equal("        }\n    }\n    namespace D {\n", Generation.adapt_namespace('/A/B/C', '/A/D'))
	assert_equal("", Generation.adapt_namespace('/A/B/C', '/A/B/C'))
    end
end


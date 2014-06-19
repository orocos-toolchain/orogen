require 'orogen/gen/test'

class TC_RTT_CPPBase < Minitest::Test
    def test_load_template
	assert_raises(ArgumentError) { RTT_CPP.load_template('does', 'not', 'exist') }

	erb = nil
	erb = RTT_CPP.load_template('typekit', 'Plugin.cpp')
	assert_kind_of(ERB, erb)

	assert_same(erb, RTT_CPP.load_template('typekit', 'Plugin.cpp'))

	other_erb = nil
	other_erb = RTT_CPP.load_template('CMakeLists.txt')
	refute_same(other_erb, erb)
    end

    def test_save_user
        create_wc "base"
	assert_raises(ArgumentError) { RTT_CPP.save_user }

	in_wc do
	    RTT_CPP.save_user 'test.cpp', "blabla"
	end

	target_file = File.join(working_directory, 'test.cpp')
	assert_equal("blabla", File.read(target_file))

	in_wc do
	    RTT_CPP.save_user 'subdir', 'test.cpp', "bloblo"
	end

	target_file = File.join(working_directory, 'subdir', 'test.cpp')
	assert_equal("bloblo", File.read(target_file))
    end

    def test_save_automatic
        create_wc "base"
	assert_raises(ArgumentError) { RTT_CPP.save_automatic }

	in_wc do
	    RTT_CPP.save_automatic 'test.cpp', "blabla"
	end

	target_file = File.join(working_directory, RTT_CPP::AUTOMATIC_AREA_NAME, 'test.cpp')
	assert_equal("blabla", File.read(target_file))

	in_wc do
	    RTT_CPP.save_automatic 'subdir', 'test.cpp', "bloblo"
	end

	target_file = File.join(working_directory, RTT_CPP::AUTOMATIC_AREA_NAME, 'subdir', 'test.cpp')
	assert_equal("bloblo", File.read(target_file))
    end

    def test_adapt_namespace
	assert_equal("    }\n", RTT_CPP.adapt_namespace('/A/B', '/A'))
	assert_equal("        }\n    }\n    namespace D {\n", RTT_CPP.adapt_namespace('/A/B/C', '/A/D'))
	assert_equal("", RTT_CPP.adapt_namespace('/A/B/C', '/A/B/C'))
    end
end


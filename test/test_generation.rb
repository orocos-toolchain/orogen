$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationErrorCases < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    # Test on a very simple component. This usually catches some buildsystem
    # problems that are hidden by the complexity of full-fledged components
    def test_simple(with_corba = true)
        build_test_component('modules/simple', with_corba)

        # Force install in prefix
        in_prefix do
        end
    end
    def test_simple_without_corba
        test_simple(false)
    end
end



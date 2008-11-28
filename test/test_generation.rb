$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationErrorCases < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    # Test on a very simple component. This usually catches some buildsystem
    # problems that are hidden by the complexity of full-fledged components
    def test_simple(with_corba = true)
        component = Generation::Component.new
        copy_in_wc File.join(TEST_DATA_DIR, "simple_component.orogen")
        in_wc do
            component.load "simple_component.orogen"
            assert_nothing_raised { component.generate } 

            if with_corba then component.enable_corba
            else component.disable_corba
            end
        end


        compile_wc(component)

        # Force install in prefix
        in_prefix do
        end
    end
    def test_simple_without_corba
        test_simple(false)
    end
end



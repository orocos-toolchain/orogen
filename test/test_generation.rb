$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationErrorCases < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    # Test on a very simple component. This usually catches some buildsystem
    # problems that are hidden by the complexity of full-fledged components
    def test_simple_component(with_corba = false)
        component = Generation::Component.new
        component.load(File.join(TEST_DATA_DIR, "simple_component.orogen"))
        assert_nothing_raised { component.generate } 

        if with_corba then component.enable_corba
        else component.disable_corba
        end

        compile_wc(component)
    end
    def test_simple_component_corba
        test_simple_component(true)
    end
end



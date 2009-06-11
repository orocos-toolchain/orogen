$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationErrorCases < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    # Test on a very simple component. This usually catches some buildsystem
    # problems that are hidden by the complexity of full-fledged components
    def test_simple(with_corba = true)
        build_test_component('modules/simple', with_corba)
        install
    end
    def test_simple_without_corba
        test_simple(false)
    end

    def test_generation_requires_name_and_orogen
	component = Component.new

        # Should raise because there is no name
        assert_raises(ArgumentError) { component.generate }

        component.name "test"
        # Should raise because there is no orogen file
        assert_raises(ArgumentError) { component.generate }

        component.instance_variable_set(:@deffile, "bla.orogen")

        create_wc("tasks/generation_validation")
        in_wc do
            component.generate
        end
    end

end



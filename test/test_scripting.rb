$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_Scripting < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    # Test that numeric calls are handled properly for all defined numeric types
    def test_numerics(with_corba = false)
        build_test_component "modules/numerics", with_corba
    end
    def test_numerics_on_corba
        test_numerics(true)
    end
end


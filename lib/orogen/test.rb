# simplecov must be loaded FIRST. Only the files required after it gets loaded
# will be profiled !!!
if ENV['TEST_ENABLE_COVERAGE'] == '1'
    begin
        require 'simplecov'
    rescue LoadError
        require 'orogen'
        OroGen.warn "coverage is disabled because the 'simplecov' gem cannot be loaded"
    rescue Exception => e
        require 'orogen'
        OroGen.warn "coverage is disabled: #{e.message}"
    end
end

require 'orogen'
## Uncomment this to enable flexmock
require 'flexmock/test_unit'
require 'minitest/spec'

if ENV['TEST_ENABLE_PRY'] != '0'
    begin
        require 'pry'
    rescue Exception
        OroGen.warn "debugging is disabled because the 'pry' gem cannot be loaded"
    end
end

module OroGen
    # This module is the common setup for all tests
    #
    # It should be included in the toplevel describe blocks
    #
    # @example
    #   require 'dummyproject/test'
    #   describe OroGen do
    #     include OroGen::SelfTest
    #   end
    #
    module SelfTest
        if defined? FlexMock
            include FlexMock::ArgumentTypes
            include FlexMock::MockContainer
        end

        def setup
            # Setup code for all the tests
        end

        def teardown
            # Teardown code for all the tests
        end
    end
end

# Old testing infrastructure
require 'orogen/gen/test'

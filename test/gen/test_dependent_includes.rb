# frozen_string_literal: true

require "orogen/gen/test"

class TC_DependentIncludes < Minitest::Test
    def build_and_install(component_name)        
        build_test_project("modules/" + component_name, ["corba"])
        install
        ENV["PKG_CONFIG_PATH"] = "#{File.join(prefix_directory, "lib", "pkgconfig")}:#{ENV['PKG_CONFIG_PATH']}"
    end

    def test_dependent_includes
        build_and_install("typekit_opaque")
        build_and_install("cross_consumer")
        build_and_install("dependent_includes")
    end
end
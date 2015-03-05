require 'orogen/gen/test'

class TC_GenerationErrorCases < Minitest::Test
    # Test on a very simple project. This usually catches some buildsystem
    # problems that are hidden by the complexity of full-fledged components
    def test_simple(*transports)
        build_test_project('modules/simple', transports)
        install
    end
    def test_simple_corba; test_simple('corba') end
    def test_simple_typelib; test_simple('typelib') end

    def test_generation_requires_name_and_orogen
	project = Project.new

        # Should raise because there is no name
        assert_raises(ArgumentError) { project.generate }

        project.name "test"
        # Should raise because there is no orogen file
        assert_raises(ArgumentError) { project.generate }

        project.instance_variable_set(:@deffile, "bla.orogen")

        create_wc("tasks/generation_validation")
        in_wc do
            project.generate
        end
    end

end



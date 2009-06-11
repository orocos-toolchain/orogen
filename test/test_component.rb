require 'orogen/test'

class TC_GenerationComponent < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

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

    def test_filter_backtrace
        Tempfile.open('test') do |io|
            io.puts "raise TypeError, \"custom exception\""
            io.sync
            io.flush

            begin
                Component.load(io.path)
                flunk "no exception thrown by load"
            rescue TypeError => e
                assert_equal("#{io.path}:1", e.backtrace[0])
                assert_equal("custom exception", e.message)
            end
        end
    end

    def test_find_type
        c = Component.new
        t = c.find_type "/int"
        assert_same c.registry.get("/int"), t
    end
    def test_find_type_should_build_arrays
        c = Component.new
        t = c.find_type "/int[12]"
        assert_same c.registry.get("/int[12]"), t
        assert !c.toolkit
    end
    def test_find_type_should_create_containers
        c = Component.new
        t = c.find_type "/std/vector</int>"
        assert_same c.registry.get("/std/vector</int>"), t
        # The container should have been defined on the toolkit as well
        assert_equal c.toolkit.registry.get("/std/vector</int>"), t
    end

    def test_imported_toolkits
        # Build the simple toolkit first
        build_test_component('modules/toolkit_simple', false)

        c = Component.new
        # Then try to load it
        in_prefix do
            c.using_toolkit "simple"
            assert_equal(1, c.used_toolkits.size)
            tk = c.used_toolkits[0]
            assert_equal("simple", tk.name)
            assert_equal("simple-toolkit-#{Generation.orocos_target}", tk.pkg.name)

            # Check that imported_type? takes the toolkit into account
            t = c.registry.get "/Test/BaseTypes"
            assert c.imported_type?(t)
        end
    end

    def test_imported_type_looks_at_rtt_registry
        c = Component.new
        t = c.registry.get "/std/vector</double>"
        assert c.imported_type?(t)
    end
end



require 'orogen/gen/test'

class TC_GenerationComponent < Minitest::Test
    def test_component_generate
        create_wc "base"
	in_wc do
            # No name, no orogen file
	    component = RTT_CPP::Component.new
	    assert_raises(ArgumentError) { component.generate }

            # No orogen file
	    component = RTT_CPP::Component.new
	    component.name "cmp"
	    assert_raises(ArgumentError) { component.generate } 
            
            # No name
	    component = RTT_CPP::Component.new
	    component.instance_variable_set(:@deffile, File.join(path_to_data, "empty_component.orogen"))
	    assert_raises(ArgumentError) { component.generate } 

            # OK
	    component = RTT_CPP::Component.new
	    component.load(File.join(path_to_data, "empty_component.orogen"))
	    component.generate
	end
    end

    def test_generation_requires_name_and_orogen
	component = Component.new

        # Should raise because there is no name
        assert_raises(ArgumentError) { component.generate }

        component.name "test"
        # Should raise because there is no orogen file
        assert_raises(ArgumentError) { component.generate }

        component.deffile = "bla.orogen"

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
                Component.load(io.path, false)
                flunk "no exception thrown by load"
            rescue TypeError => e
                assert_equal("#{io.path}:1", e.backtrace[0], "complete backtrace is:\n  #{e.backtrace.join("\n  ")}")
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
        assert !c.typekit
    end
    def test_find_type_should_create_containers
        c = Component.new
        t = c.find_type "/std/vector</int>"
        assert_same c.registry.get("/std/vector</int>"), t
        # The container should have been defined on the typekit as well
        assert_equal c.typekit.registry.get("/std/vector</int>"), t
    end

    def test_imported_typekits
        # Build the simple typekit first
        build_test_component('modules/typekit_simple', [])

        c = Component.new
        # Then try to load it
        in_prefix do
            c.using_typekit "simple"
            assert_equal(2, c.used_typekits.size)
            tk = c.used_typekits.find { |t| t.name == "simple" }
            assert tk
            assert_equal("simple", tk.name)
            assert_equal("simple-typekit-#{OroGen::Gen::RTT_CPP.orocos_target}", tk.pkg.name)

            # Check that imported_type? takes the typekit into account
            t = c.registry.get "/Test/BaseTypes"
            assert c.imported_type?(t)
        end
    end

    def test_imported_type_looks_at_rtt_registry
        c = Component.new
        t = c.registry.get "/uint64_t"
        assert c.imported_type?(t)
    end

    def test_project_names_is_lowercase
        c = Component.new
        create_wc "lowercase_assertion"
        in_wc do
            c.name "Test"
            assert_raises(ConfigError) { c.generate }
        end
    end
end


require 'orogen/gen/test'

class TC_GenerationComponent < Minitest::Test
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

    def test_find_task
        component = Component.new
        component.name "TestFindTask"
        task = component.task_context "Task"

        assert_equal(task, component.find_task_context("Task"))
        assert_equal(task, component.find_task_context("TestFindTask::Task"))
        assert_raises(ArgumentError) { component.find_task_context("Bla") }

        build_test_component("modules/simple", [])
        install
        ENV['PKG_CONFIG_PATH'] = "#{File.join(prefix_directory, "lib", "pkgconfig")}:#{ENV['PKG_CONFIG_PATH']}"
        component.using_task_library "simple"
        assert_raises(ArgumentError) { component.find_task_context("SimpleTask") }
        assert_raises(ArgumentError) { component.find_task_context("TestFindTask::SimpleTask") }
        assert(component.find_task_context("simple::SimpleTask"))
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


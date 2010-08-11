$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationDeployment < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    def test_task_name
	component = Component.new 
	component.name 'test'

	context    = component.task_context "task"
        deployment = component.deployment "test"
        task       = deployment.task "my_name", "task"
        assert_equal "my_name", task.name
    end

    def test_task_fails_if_model_does_not_exist
	component = Component.new 
	component.name 'test'

	context    = component.task_context "task"
        deployment = component.deployment "test"

        assert_raises(ArgumentError) { deployment.task "name", "Bla" }
    end

    def test_connpolicy
        policy = ConnPolicy.from_hash :type => :buffer, :lock_policy => :locked, :size => 10
        assert_equal(:buffer, policy.type)
        assert_equal(:locked, policy.lock_policy)
        assert_equal(false, policy.pull)
        assert_equal(10, policy.size)

        policy = ConnPolicy.from_hash(Hash.new)
        assert_equal(:data, policy.type)
        assert_equal(:lock_free, policy.lock_policy)
        assert_equal(false, policy.pull)
        assert_equal(0, policy.size)

        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :unknown }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :lock_policy => :unknown }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :data, :size => 10 }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :data, :size => 0 }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :buffer }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :buffer, :size => 0 }
    end

    def create_simple_deployment
        return component, deployment, task
    end
    def test_activity_uses_default
	component = Component.new 
	component.name 'test'
	context    = component.task_context "task"
        context.default_activity :periodic, 0.1
        deployment = component.deployment "test"
        task       = deployment.task "my_name", "task"
        assert_equal("PeriodicActivity", task.activity_type)
        assert_equal(0.1, task.period)
    end
    def test_can_change_default_activity
	component = Component.new 
	component.name 'test'
	context    = component.task_context "task"
        context.default_activity :periodic, 0.1
        deployment = component.deployment "test"
        task       = deployment.task "my_name", "task"
        task.triggered
        assert_equal("NonPeriodicActivity", task.activity_type)
    end
    def test_cannot_change_required_activity
	component = Component.new 
	component.name 'test'
	context    = component.task_context "task"
        context.required_activity :periodic, 0.1
        deployment = component.deployment "test"
        task       = deployment.task "my_name", "task"
        assert_raises(ArgumentError) { task.triggered }
    end
    def test_cannot_modify_explicitely_set_activity
	component = Component.new 
	component.name 'test'
	context    = component.task_context "task"
        deployment = component.deployment "test"
        task       = deployment.task "my_name", "task"
        task.triggered
        
        assert_raises(ArgumentError) { task.periodic(0.1) }
    end

    def test_data_driven_deployment(with_corba = false)
        build_test_component "modules/data_triggered", with_corba, "bin/data"

        # Check the resulting file
        in_prefix do
            assert_equal "U 2 4 6 8 10 ", File.read('data_trigger.txt')
        end
    end

    def test_fd_driven_deployment(with_corba = false)
        build_test_component "modules/fd_triggered", with_corba

        # Start the resulting deployment
        in_prefix do
            reader, writer = IO.pipe
            child_pid = fork do
                writer.close
                ENV["FD_DRIVEN_TEST_FILE"] = reader.fileno.to_s
                exec("./bin/fd")
            end

            reader.close
            sleep 0.5
            writer.write("AB")
            sleep 0.5
            writer.write("CDE")
            Process.waitpid(child_pid)
            assert_equal(0, $?.exitstatus)
        end
    end

    def test_cross_dependencies(with_corba = true)
        # Generate and build all the modules that are needed ...
        toolkit_opaque = build_test_component("modules/toolkit_opaque", with_corba)
        install
        ENV['PKG_CONFIG_PATH'] = "#{File.join(prefix_directory, "lib", "pkgconfig")}:#{ENV['PKG_CONFIG_PATH']}"
        puts ENV['PKG_CONFIG_PATH'].inspect

        cross_producer = build_test_component("modules/cross_producer", with_corba)
        install
        producer_pkgconfig = File.join(prefix_directory, "lib", "pkgconfig")
        cross_consumer = build_test_component("modules/cross_consumer", with_corba)
        install

        ENV['PKG_CONFIG_PATH'] = "#{File.join(prefix_directory, "lib", "pkgconfig")}:#{producer_pkgconfig}:#{ENV['PKG_CONFIG_PATH']}"

        # Start by loading the component specfication and check some properties
        # on it. Then, do the generation, build and test
        cross_deployment = Component.load(File.join(TEST_DATA_DIR, "modules", "cross_deployment", "deployment.orogen"))

        deployer = cross_deployment.deployers.find { true }
        assert_equal(["opaque"], cross_deployment.used_toolkits.map(&:name))
        assert_equal(["opaque"], deployer.used_toolkits.map(&:name))

        cross_deployment = build_test_component("modules/cross_deployment", with_corba)
        install

        in_prefix do
            if !system(File.join("bin", "cross_deployment"))
                raise "deployer did not finish correctly"
            end

            assert_equal "[1 2] [3 4] [5 6] [7 8] [9 10] [11 12] [13 14] [15 16] [17 18] [19 20] ",
                File.read('cross_dependencies.txt')
        end
    end
    def test_cross_dependencies_without_corba; test_cross_dependencies(true) end
end



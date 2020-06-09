require 'orogen/test'
require 'orogen/ros'

module ROSLoaderTestSetup
    def path_to_data
        File.join(File.dirname(__FILE__),"models")
    end

    def create_ros_loader
        root = OroGen::Loaders::Aggregate.new
        typekit_loader = OroGen::Loaders::Files.new(root)
        typekit_loader.register_typekit(path_to_data, 'std')
        typekit_loader.register_typekit(path_to_data, 'base')
        ros_loader = OroGen::ROS::DefaultLoader.new(root)
        ros_loader.search_path << path_to_data
        ros_loader.package_paths['manipulator_config'] = path_to_data
        root.typekit_model_from_name('std')
        root
    end

    attr_reader :loader
    def setup
        @loader = create_ros_loader
        super
    end
end

describe OroGen::ROS::Loader do
    include ROSLoaderTestSetup

    it "tells us if a given ROS package is available" do
        assert loader.has_project?('artemis_state_publisher')
    end
    it "tells us if a given ROS package is available" do
        assert !loader.has_project?('bla')
    end
    it "should load existing node descriptions" do
        project = loader.project_model_from_name("artemis_state_publisher")
        node = project.task_model_from_name('artemis_state_publisher_node')
        assert node.has_port?("joint_states_out")
        assert node.has_port?("joint_states_in")
        assert node.find_output_port("joint_states_out")
        assert node.find_input_port("joint_states_in")
    end
    it "should load project models as OroGen::ROS::Spec::Package" do
        package = loader.project_model_from_name('joint_state_publisher')
        assert_kind_of OroGen::ROS::Spec::Package, package
    end
    it "should load task models as OroGen::ROS::Spec::Node" do
        package = loader.project_model_from_name('joint_state_publisher')
        node = package.task_model_from_name('joint_state_publisher')
        assert_kind_of OroGen::ROS::Spec::Node, node
    end
end

describe OroGen::ROS::Spec::Package do
    include ROSLoaderTestSetup
    attr_reader :project

    before do
        @project = OroGen::ROS::Spec::Package.new(loader)
    end

    it "lists the ROS nodes in self_tasks" do
        package = loader.project_model_from_name("artemis_state_publisher")
        node = package.self_tasks['artemis_state_publisher::artemis_state_publisher_node']
        assert_equal 'artemis_state_publisher::artemis_state_publisher_node', node.name
    end

    it "lists the ROS launchers in deployers" do
        package = loader.project_model_from_name("manipulator_config")
        launcher = package.deployers['test']
        assert_equal 'test', launcher.name
    end

    describe "normalize_name" do
        it "should normalize names to their full path" do
            assert_equal "/test", OroGen::ROS.normalize_name("test"), "Node name normalization"
            assert_equal "/test", OroGen::ROS.normalize_name("//test"), "Node name normalization"
            assert_equal "/test", OroGen::ROS.normalize_topic_name("////test"), "Topic name normalization"
            assert_equal "/test", OroGen::ROS.normalize_topic_name("test"), "Topic name normalization"
        end
    end

    describe "equality" do
        it "should return true for nodes with the same name" do
            n1 = OroGen::ROS::Spec::Node.new(project, "test")
            n2 = OroGen::ROS::Spec::Node.new(project, "test")

            assert_equal n1,n1, "Node equality"
            assert_equal n1,n2, "Node equality"
        end
    end

    describe "node superclass" do
    end

end


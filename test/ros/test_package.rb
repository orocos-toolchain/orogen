require 'orogen_ros/test'

require 'orocos'

describe OroGen::ROS::Loader do
    include OroGen::SelfTest

    attr_reader :loader
    before do
        @loader = OroGen::ROS::Loader.new
        loader.search_path << File.join(File.dirname(__FILE__),"orogen")
    end

    it "tells us if a given ROS package is available" do
        assert loader.has_project? 'artemis_state_publisher'
    end
    it "tells us if a given ROS package is available" do
        assert !loader.has_project? 'bla'
    end
end

describe OroGen::ROS::Package do
    include OroGen::SelfTest

    attr_reader :loader
    before do
        @loader = OroGen::ROS::Loader.new
        loader.search_path << File.join(File.dirname(__FILE__),"orogen")
    end

    it "lists the ROS nodes in self_tasks" do
        package = launcher.project_model_from_name("artemis_state_publisher")
        node = package.self_tasks['artemis_state_publisher_node']
        assert_equal 'artemis_state_publisher_node', node.name
    end

    it "lists the ROS launchers in deployers" do
        package = launcher.project_model_from_name("manipulator_config")
        node = package.deployers['test']
        assert_equal 'artemis_state_publisher_node', node.name
    end

    describe "rosnode_findpackage" do
        Orocos::ROS.load

        package = Orocos::ROS.rosnode_findpackage("state_publisher")
        assert package.name == "artemis_state_publisher", "Find package of node #{package}"
    end

    describe "simple functions" do
        assert_equal "/test", Orocos::ROS.rosnode_normalize_name("test"), "Node name normalization"
        assert_equal "/test", Orocos::ROS.rosnode_normalize_name("//test"), "Node name normalization"
        assert_equal "/test", Orocos::ROS.normalize_topic_name("////test"), "Topic name normalization"
        assert_equal "/test", Orocos::ROS.normalize_topic_name("test"), "Topic name normalization"
    end

    describe "equality" do
        n1 = Orocos::ROS::Spec::Node.new(nil, "test")
        n2 = Orocos::ROS::Spec::Node.new(nil, "test")

        assert_equal n1,n1, "Node equality"
        assert_equal n1,n2, "Node equality"
    end

    describe "node superclass" do
        n1 = Orocos::ROS::Spec::Node.new(nil, "test")
        assert_equal n1.superclass.name, "ROS::Node", "Node superclass should be ROS::Node, but was #{n1.superclass.name}"
    end

    describe "node spec" do

        assert Orocos::ROS.node_spec_available?("artemis_motionPlanner"), "Node spec available"

        spec = Orocos::ROS.node_spec_by_node_name("artemis_motionPlanner")
        assert spec.has_port?("status")
        assert spec.has_port?("target_trajectory")

        assert spec.find_output_port("status")
    end
end


require 'orogen_ros/test'

require 'orocos'

describe Orocos::ROS::Generation::Project do
    include Orocos::ROS::SelfTest

    Orocos::ROS.spec_search_directories << File.join(File.dirname(__FILE__),"orogen")

    describe "loading ROS files" do
        Orocos::ROS.load(File.join(File.dirname(__FILE__),"orogen"))

        launchers = Orocos::ROS.available_launchers
        assert !launchers.empty?, "Launchers empty"
        puts "Available launchers"
        launchers.each do |l|
            puts "Launcher: #{l} #{l.class}"
        end
    end

    describe "loading orogen specs with ros definition" do
        Orocos::ROS.spec_search_directories.each do |dir|
            specs = Dir.glob(File.join(dir,"*.orogen"))
            specs.each do |file|
                p = Orocos::ROS::Generation::Project.load(file)

                assert !p.orogen_project?

                p.self_tasks.each do |t|
                    puts "Task: #{t} --> #{t.ros_name}"
                end

                if !p.ros_launchers.empty?
                    launcher = p.ros_launchers[0]
                    assert_equal "test", launcher.name
                end
            end
        end # Orocos::ROS.spec_search_directories
    end # describe

    describe "loading launcher definition" do
        specs = Dir.glob(File.join(File.dirname(__FILE__),"orogen","manipulator_config.orogen"))
        specs.each do |file|
            p = Orocos::ROS::Generation::Project.load(file)

            assert !p.orogen_project?

            p.self_tasks.each do |t|
                puts "Task: #{t} --> #{t.ros_name}"
            end

            assert !p.ros_launchers.empty?
            launcher = p.ros_launchers[0]
            assert_equal "test", launcher.name
            puts launcher
            assert !launcher.nodes.empty?, "Launcher did not contain nodes"
            launcher.nodes.each do |n|
                puts "Node #{n} #{n.task_model}"
            end
        end
    end # describe 

    describe "simple functions" do
        assert_equal "/test", Orocos::ROS.rosnode_normalize_name("test"), "Node name normalization"
    end
end


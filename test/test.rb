require 'orogen_ros/test'

describe Orocos::ROS::Generation::Project do
    include Orocos::ROS::SelfTest

    Orocos::ROS.spec_search_directories << File.join(File.dirname(__FILE__),"orogen")

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
end


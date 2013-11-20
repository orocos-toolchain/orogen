module Orocos
    module ROS
        class Package < OroGen::Spec::Project
            extend Logger::Hierarchy

            # @return [Array] array of ros nodes defined in this project
            attr_reader :ros_nodes

            # @return [Array] array of ros launch descriptions defined in this project
            attr_reader :ros_launchers

            def initialize(loader, pkg = nil)
                super 

                @ros_nodes = []
                @ros_launchers = []
            end

            def default_node_superclass
                find_task_context OROGEN_ROS_NODE_NAME
            end

            # Declares a ros node that exists within the package
            # define by the project name
            #
            # name "ros_package"
            # ros_node "ros_node" do
            #   .. node specification ..
            # end
            #
            # defines a <tt> ros_package::ros_node</tt> class.
            #
            # Nodes are represented as instances of RosNode. See
            # the documentation of that class for more details
            # @return [Node]
            def ros_node(name, options = Hash.new, &block)
                options[:class] = Spec::Node
                node = external_task_context(name, options, &block)
                node.ros_name = name
                begin
                    ROS.rospack_find(self.name)
                rescue ArgumentError => e
                    raise e, "could not find a ros package of name '#{self.name}'. Please check if you set the project name to a known ROS package: #{e.message}", e.backtrace
                end
                node.ros_package = self.name
                ros_nodes << node
                node
            end

            # Declares a ros launcher that exists within the package
            # defined by the project name
            #
            # ros_launcher "test"
            #     use_existing
            # end
            #
            # If use existing is given it will search for "test.lauch" in 
            # the ros package and extract the nodes started by the launch-
            # file
            def ros_launcher(name, &block)
                launcher = Spec::Launcher.new(self, name, &block)
                launcher.instance_eval(&block) if block_given?
                ros_launchers << launcher
                launcher
            rescue Exception => e
                raise RuntimeError, "Defining ROS Launcher failed -- #{e}", e.backtrace
            end

            def using_ros_package(name)
                using_task_library(name)
            end

            def simple_deployment
                raise NotImplementedError, "cannot create a simple deployment from a Orocos::Spec::ROS::Project"
            end
        end
    end
end

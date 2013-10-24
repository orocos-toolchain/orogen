module Orocos::ROS
    module Generation
        class Project < Orocos::Generation::Project
            include Spec::Package
            extend Logger::Hierarchy

            # @return [Array] array of ros nodes defined in this project
            attr_reader :ros_nodes

            # @return [Array] array of ros launch descriptions defined in this project
            attr_reader :ros_launchers

            def initialize
                super 

                @ros_nodes = []
                @ros_launchers = []
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
                begin
                    launcher = Spec::Launcher.new(self, name, &block)
                    launcher.instance_eval(&block) if block_given?
                    ros_launchers << launcher
                    launcher
                rescue Exception => e
                    raise RuntimeError, "Defining ROS Launcher failed -- #{e}"
                end
            end

            def orogen_project?
                false
            end

            # Allow ROS specs to be loaded from the default search spec directories
            # for ROS
            #
            # see Orocos::ROS.spec_search_directories
            def orogen_project_description(name)
                result = nil
                begin
                    result = super
                rescue ::Orocos::Generation::Project::MissingTaskLibrary => e
                    if p = Orocos::ROS.available_projects[name]
                        _, path = p
                        return register_orogen_file(path)
                    end
                    raise
                end
            end

            def define_default_deployments?
                false
            end

            def simple_deployment(name,klass)
                # n/a for ros nodes
            end
        end
    end
end

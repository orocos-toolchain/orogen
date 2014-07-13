module OroGen
    module ROS
    module Spec
        # Namespace containing functionality to parse XML based
        # information from ROS
        module XML
            # This class represents the node description
            # which can be extracted from a launch file
            class NodeDescription
                attr_reader :package
                attr_reader :type
                attr_reader :name

                # Define optional attributes -- based on the ROS Spec
                @optional_attr = { :respawn => false, :output => nil }
                @optional_attr.each do |attr_name,_|
                    attr_reader attr_name
                end

                # Optional attributes in the ROS launcher specification
                # @return [Hash] Hash of optional attributed along with the default value
                def self.optional_attr
                    @optional_attr
                end

                # Initialize the node description
                # @argument name Name of the node
                # @argument package Package name of the node
                # @argument type ROS node type of this node
                def initialize(name, package, type)
                    @name = name
                    @package = package
                    @type = type

                    NodeDescription.optional_attr.each do |op, val|
                        self.instance_variable_set "@#{op}", val
                    end
                end

                # Create a node description from a xml node
                # @return [OroGen::ROS::Spec::XML::NodeDescription] Node description object
                def self.from_xml_node(node)
                    name = node.attributes["name"].to_s
                    package = node.attributes["pkg"].to_s
                    type = node.attributes["type"].to_s

                    nd = NodeDescription.new(name, package, type)

                    NodeDescription.optional_attr.each do |o,_|
                        if attr = node.attributes[o.to_s]
                            nd.instance_variable_set("@#{o}",attr)
                        end
                    end
                    nd
                end

                # String description of this object
                # @return [String] 
                def to_s
                    desc = "NodeDescription: name: #{name}, package: #{package}, type: #{type}"
                    NodeDescription.optional_attr.each do |o,_|
                        val = instance_variable_get("@#{o}")
                        desc += ", #{o}: #{val}"
                    end
                    desc
                end
            end
        end

        # Launcher specification
        # ROS allows to define launch files in order to handle 
        # the startup of multiple nodes or network of nodes
        #
        # This class allows to store the information contained in
        # a ROS launcher specification in an equivalent to
        # Orocos' deployments
        #
        class Launcher < OroGen::Spec::Deployment
            extend Logger::Hierarchy

            # [Package] Project this launcher is part of
            attr_reader :project

            # [String] Name of the launcher
            attr_reader :name
            alias :nodes :task_activities

            # [Boolean] Flag if the launch file content should be loaded
            # to extract node definitions
            attr_reader :loaded_launch_file

            # [String] Path to the launch file that is or has been loaded if
            # #load_launch_file is set
            attr_reader :launch_file

            # Initialize the project
            #
            # Automatically tries to resolve the corresponding launch
            # files in the file systems
            # @throw [ArgumentError] Raises if a corresponding launch file could not be found
            def initialize(project, name = nil)
                @project = project
                @name = name
                @nodes = []
                @task_activities = []
                @load_launch_file = nil

                # search for launch file where project.name == ros package name
                @launch_file = project.ros_loader.roslaunch_find(project.name, name)
            end

            # Loads the launch file if not already loaded
            def load_launch_file
                if !loaded_launch_file?
                    @loaded_launch_file = true
                    parse(launch_file)
                end
            end

            # Test whether the launch file has been automatically loaded
            def loaded_launch_file?
                !!@loaded_launch_file
            end

            # Parse the launch file
            # @return [String] Absolute path to the lauch file that has been
            #     loaded
            #
            def parse(path)
                if File.exists?(path)
                    @node_descriptors = Launcher.parse(path)
                    @node_descriptors.each do |d|
                        node(d.name, "#{d.package}::#{d.type}")
                    end
                else
                    raise ArgumentError, "Launcher: could not find launch file: '#{path}' in #{Dir.pwd}"
                end
                File.absolute_path(path)
            end

            # Declares a node in a ROS launcher
            #
            # This is equivalently to declaring a task in a RTT deployment
            #
            # @return [OroGen::Spec::TaskDeployment] A task deployment
            def node(name, klass)
                task_deployment = task(name, klass)

                # Update with ros information
                node = task_deployment.task_model
                node.ros_name = node.name.split("::")[1]
                node.ros_package = node.project.name
                @nodes << node

                task_deployment
            end

            # Parses a given launch file and extracts the launch information
            # @return [Set<Spec::XML::NodeDescription>] Extract all nodes from a launch file
            def self.parse(filename)
                if not File.exists?(filename)
                    raise ArgumentError, "#{self}: could not find file '#{filename}'"
                end

                nodes = []
                File.open(filename) do |f|
                    doc = REXML::Document.new(f)
                    doc.each_element('//node') do |n|
                        nodes << XML::NodeDescription.from_xml_node(n)
                    end
                end
                nodes
            end

            # String representaiton of this launcher 
            def to_s
                "Launcher: #{name}, launch_file: #{@launch_file}, loaded: #{loaded_launch_file?}"
            end
        end
    end
    end
end


require 'nokogiri'
require 'utilrb'

module Orocos::ROS
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
            
                def self.optional_attr
                    @optional_attr
                end
            
                def initialize(name, package, type)
                    @name = name
                    @package = package
                    @type = type
            
                    NodeDescription.optional_attr.each do |op, val|
                        self.instance_variable_set "@#{op}", val
                    end
                end
            
                def self.from_xml_node(node)
                    name = node.attribute("name").to_s
                    package = node.attribute("pkg").to_s
                    type = node.attribute("type").to_s
            
                    nd = NodeDescription.new(name, package, type)
            
                    NodeDescription.optional_attr.each do |o,_|
                        if attr = node.attribute("#{o}")
                            nd.instance_variable_set("@#{o}",attr)
                        end
                    end
                    nd
                end
            
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

        # Launch specification
        class Launcher < Orocos::Spec::Deployment
            extend ::Logger::Root("Orocos::ROS::Launcher", ::Logger::WARN)

            # Project this launcher is part of
            attr_reader :project

            # Name of the launcher
            attr_reader :name
            alias :nodes :task_activities

            attr_reader :reuse_existing
            attr_reader :launch_file

            def initialize(project = nil, name = nil)
                @project = project
                @name = name
                @nodes = []
                @task_activities = []
                @reuse_existing = nil
                @launch_file = nil
            end

            def reuse_existing
                if !@reuse_existing
                    @reuse_existing = true
                    # search for name.launch, project.name == ros package name
                    @launch_file = Orocos::ROS.roslaunch_find(project.name, name)
                    parse(@launch_file)
                end
            end

            def reuse_existing?
                !!@reuse_existing
            end

            # Parse the launch file
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
            def self.parse(filename)
                if not File.exists?(filename)
                    raise ArgumentError, "#{self}: could not find file '#{filename}'"
                end
        
                nodes = []
                File.open(filename) do |f|
                    doc = Nokogiri::XML(f)
                    doc.search('node').each do |n|
                        nodes << XML::NodeDescription.from_xml_node(n)
                    end
                end
                nodes
            end

            def self.load_specs(*args)
            end

            def to_s
                "Launcher: #{name}, use_existing: #{@reuse_existing}, #{@launch_file}"
            end
        end # Launch
    end # Spec
end # Orocos::ROS


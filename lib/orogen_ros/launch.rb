require 'nokogiri'
require 'orogen_ros/node'

module Orocos::ROS
    module Generation
        class Project < Orocos::Generation::Project
            attr_reader :ros_nodes

            attr_reader :ros_launchers

            def initialize
                super 

                @ros_nodes = []
                @ros_launchers = []
            end

            def node_type(name, options = Hash.new, &block)
                options[:class] = Orocos::ROS::Spec::Node
                node = task_context(name, options, &block)
                @ros_nodes << node
                node.ros_name = name
                node.ros_package = self.name
                node
            end

            def launcher(name, &block)
                launcher = Spec::Launcher.new(self, name, &block)
                @ros_launchers << launcher
                launcher
            end

            def orogen_project?
                false
            end

        end
    end

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
                    name = node.attribute("name")
                    package = node.attribute("package")
                    type = node.attribute("type")
            
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
        class Launcher
            attr_reader :project
            attr_reader :name
            attr_reader :nodes

            attr_reader :reuse_existing
            attr_reader :launch_file
        
            def initialize(project = nil, name = nil)
                @project = project
                @name = name
                @nodes = []
                @reuse_existing = false
                @launch_file = nil

                # search for name.launch
                filename = locate_launch_file(name)
            end

            def reuse_existing
                @reuse_existing = true
            end

            # Locate the launch file
            def locate_launch_file(name)
                name = name.gsub(/\.launch$/,"")
                name += ".launch"
                if reuse_existing
                    @launch_file = load_launch_file(name)
                end
            end

            def load_launch_file(name)
                if File.exists?(name)
                    @node_descriptors = Launcher.parse(name)
                    @node_descriptors.each do |d|
                        node = Node.new(d.name, d.type)
                        node.ros_package = d.package
                        @nodes << node
                    end
                else
                    raise ArgumentError, "Launcher: could not find launch file: '#{name}' in #{Dir.pwd}"
                end
                File.absolute_path(name)
            end

            def node(name, type)
                node = Node.new(name, type)
                @nodes << node
                node
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

            def setup
            end

            def run
            end

            def self.load_specs(*args)
            end

            def to_s
                "Launcher: #{name}, use_existing: #{reuse_existing}, #{launch_file}"
            end
        end # Launch
    end # Spec
end # Orocos::ROS


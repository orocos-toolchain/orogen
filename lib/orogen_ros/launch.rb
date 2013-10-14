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
                    name = node.attribute("name")
                    package = node.attribute("pkg")
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
        class Launcher < Orocos::Spec::Deployment
            extend ::Logger::Root("Orocos::ROS::Launcher", ::Logger::WARN)

            attr_reader :project
            attr_reader :name
            attr_reader :nodes

            attr_reader :reuse_existing
            attr_reader :launch_file

            attr_reader :pid
        
            def initialize(project = nil, name = nil)
                @project = project
                @name = name
                @nodes = []
                @task_activities = []
                @reuse_existing = false
                @launch_file = nil

                @pid = nil

                # search for name.launch, project.name == ros package name
                filename = roslaunch_find(project.name, name)
            end

            def reuse_existing
                @reuse_existing = true
            end

            def kill
                ::Process.kill('INT', @pid)
            end

            # Spawn the launch file
            # @return [int] pid of the launch process
            def spawn
                @pid = Orocos::ROS.roslaunch(project.name, "#{name}.launch")
            end

            # Locate the launch file in a given ros package 
            # @return [String] absolute path to the launch file
            # @throws [ArgumentError] if the launch file cannot be found in the ros package
            def roslaunch_find(package_name, launch_name)
                package_path = Orocos::ROS.rospack_find(package_name)
                launch_path = File.join(package_path, "launch")

                launch_name = launch_name.gsub(/\.launch$/,"")
                launch_name += ".launch"
                if reuse_existing
                    launch_path = File.join(launch_path, launch_name)
                    if !File.file?(launch_path)
                        raise ArgumentError, "there is no launch_file called #{launch_name} in #{package_name} (looked in #{launch_path})"
                    end
                    launch_path

                    @launch_file = parse(launch_path)
                end
            end

            # Parse the launch file
            def parse(path)
                if File.exists?(path)
                    @node_descriptors = Launcher.parse(path)
                    @node_descriptors.each do |d|
                        puts "NODE #{d}"
                        node(d.name, "#{d.package}::#{d.type}")
                    end
                else
                    raise ArgumentError, "Launcher: could not find launch file: '#{path}' in #{Dir.pwd}"
                end
                File.absolute_path(path)
            end

            def node(name, klass)
                task_deployment = nil
                begin 
                    task_deployment = task(name, klass)
                    Launcher.warn "Orocos::ROS: found task: '#{name}' => '#{klass}' from using statement."
                rescue Exception => e
                    if reuse_existing
                        Launcher.warn "Orocos::ROS: using ros node definition from launch file: '#{name}' => '#{klass}' from launch file."
                        task_context = Node.new(name, klass)
                        task_deployment = Orocos::Spec::TaskDeployment.new(name, task_context)
                        @task_activities << task_deployment
                    else
                        raise
                    end
                end
                @nodes << task_context
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

            def run
                spawn
            end

            def self.load_specs(*args)
            end

            def to_s
                "Launcher: #{name}, use_existing: #{reuse_existing}, #{launch_file}"
            end
        end # Launch
    end # Spec
end # Orocos::ROS


module OroGen
    module ROS
    module Spec
        class Package < OroGen::Spec::Project
            extend Logger::Hierarchy

            # The loader that should be used for ROS-specific stuff
            # @return [Loader]
            attr_reader :ros_loader

            # The name of the root model for all ROS nodes
            OROGEN_ROS_NODE_NAME = "ROS::Node"
            
            def initialize(root_loader, ros_loader = root_loader)
                super(root_loader)
                @ros_loader = ros_loader
                self.define_default_deployments = false
            end

            def default_task_superclass
                if @default_task_superclass.nil?
                    @default_task_superclass = loader.task_model_from_name "ROS::Node"
                else @default_task_superclass
                end
            end

            # Load a typekit to import RTT-to-ROS mappings
            def import_types_from(typekit)
                using_typekit(typekit)
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
                task = external_task_context(name, Hash[:class => Spec::Node].merge(options), &block)
                self_tasks[task.name] = task
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
                launcher = Spec::Launcher.new(self, name)
                launcher.instance_eval(&block) if block_given?
                deployers[launcher.name] = launcher
                launcher
            rescue Exception => e
                raise e, "defining ROS launcher #{name} on #{self} failed: #{e.message}", e.backtrace
            end

            def using_ros_package(name)
                using_task_library(name)
            end

            def simple_deployment
                raise NotImplementedError, "cannot create simple deployments on #{self.class}"
            end

            def __eval__(deffile, deftext, verbose = (::OroGen.logger.level == ::Logger::DEBUG))
                if !deffile
                    @load_doc = false
                    instance_eval deftext
                else
                    @load_doc = ::File.file?(deffile)
                    instance_eval deftext, deffile, 1
                end
                self
            rescue ::Exception => e
                if true || verbose then ::Kernel.raise
                else
                    this_level = ::Kernel.caller.size
                    until_here = e.backtrace[-(this_level-1)..-1] || []
                    subcalls = e.backtrace[0, e.backtrace.size - this_level - 1] || []
                    subcalls.delete_if { |line| line =~ /eval|method_missing/ && line !~ /\.orogen/ }
                    subcalls = subcalls.map { |line| line.gsub(/:in `(?:block in )?__eval__'/, '') }
                    ::Kernel.raise e, e.message, (subcalls + until_here)
                end
            end

        end
    end
    end
end

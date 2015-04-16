require 'set'

module OroGen
    module Gen
    module RTT_CPP
        module TaskDeploymentGeneration
            def generate_activity_setup
                if @activity_setup
                    @activity_setup.call
                else
                    result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(
            #{rtt_scheduler},
            #{rtt_priority},
            task_#{name}->engine(),
            "#{name}");
                    EOD
                end
            end

            def to_deployer_xml
                @activity_xml.call
            end

            # Returns the scheduler constant name for this task's scheduler
            # class. Call #realtime and #non_realtime to change the task
            # scheduling class
	    def rtt_scheduler
		if @realtime then 'ORO_SCHED_RT'
		else 'ORO_SCHED_OTHER'
		end
	    end

	    # Returns the Orocos value for this task's priority
	    def rtt_priority
		case @priority
		when :highest
		    'RTT::os::HighestPriority'
		when :lowest
		    'RTT::os::LowestPriority'
		when Integer
		    @priority
		end
	    end

            def method_missing(*args, &block)
                if project.deffile && File.file?(project.deffile)
                    OroGen.check_for_stray_dots(project.deffile, name, args)
                end
                super
            end
        end

        module DeploymentGeneration
            def dependencies
                result = []
                result << BuildDependency.new(
                    "OrocosRTT",
                    "orocos-rtt-#{Generation.orocos_target}").
                    in_context('core', 'include').
                    in_context('core', 'link')

                if browse
                    result << BuildDependency.new(
                        "OrocosOCL",
                        "orocos-ocl-#{Generation.orocos_target}").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end
                if corba_enabled?
                    result << BuildDependency.new(
                        "OrocosCORBA",
                        "orocos-rtt-corba-#{Generation.orocos_target}").
                        in_context('corba', 'include').
                        in_context('corba', 'link')
                end
                if transports.include? 'ros'
                    result << BuildDependency.new(
                        "ROSLIB", "roslib").
                        in_context('core', 'include').
                        in_context('core', 'link')
                    result << BuildDependency.new(
                        "ROSCPP", "roscpp").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end

                used_typekits.each do |tk|
                    next if tk.virtual?
                    result << BuildDependency.new(
                        "#{tk.name}_TYPEKIT",
                        tk.pkg_name).
                        in_context('core', 'include').
                        in_context('core', 'link')

                    transports.each do |transport_name|
                        result << BuildDependency.new(
                            "#{tk.name}_TRANSPORT_#{transport_name.upcase}",
                            tk.pkg_transport_name(transport_name)).
                            in_context('core', 'include').
                            in_context('core', 'link')
                    end
                end

                project.used_libraries.each do |pkg|
                    result << BuildDependency.new(
                        "#{pkg.name}",
                        "#{pkg.name}").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end

                used_task_libraries.each do |pkg|
                    next if pkg.name == project.name

                    result << BuildDependency.new(
                        "#{pkg.name}_TASKLIB",
                        "#{pkg.name}-tasks-#{Generation.orocos_target}").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end

                # Task files could be using headers from external libraries, so add the relevant
                # directory in our include path
                project.tasklib_dependencies.
                    find_all { |builddep| builddep.in_context?('core', 'include') }.
                    each do |builddep|
                        builddep = BuildDependency.new(builddep.var_name, builddep.pkg_name)
                        builddep.in_context('core', 'include')
                        result << builddep
                    end

                result.to_a.sort_by { |dep| dep.var_name }
            end

            def used_task_libraries
                task_models = Set.new
                task_activities.each do |task|
                    task_models |= task.task_model.ancestors.to_set
                end
                task_models.delete_if do |task|
                    !task.project.orogen_project?
                end

                dependencies = Hash.new
                task_models.each do |model|
                    if p = dependencies[model.project.name]
                        if p != model.project
                            raise InternalError, "found two Project objects that seem to refer to the same project: #{p.name}"
                        end
                    else
                        dependencies[model.project.name] = model.project
                    end
                end
                dependencies.values
            end

            def to_deployer_xml
                result = []
                result << <<-EOHEADER
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE properties SYSTEM "cpf.dtd">
<properties>
                EOHEADER
                used_typekits.each do |tk|
                    next if tk.virtual?
                    result << "<simple name=\"Import\" type=\"string\"><value>#{tk.name}</value></simple>"
                end
                used_task_libraries.each do |pkg|
                    result << "<simple name=\"Import\" type=\"string\"><value>#{pkg.name}</value></simple>"
                end

                task_activities.each do |task|
                    result << task.to_deployer_xml
                end
                result << "</properties>"
                result.join("\n")
            end

            # Generates the code associated with this deployment setup
            def generate
                deployer = self

                if !corba_enabled? && !@browse
                    OroGen.warn "the deployment #{name} will do nothing. Either generate with --transports=corba or use the 'browse' statement"
                end

		main = Generation.render_template 'main.cpp', binding
		Generation.save_automatic "main-#{name}.cpp", main
                pkg = if install?
                          Generation.render_template 'deployment.pc', binding
                      else
                          Generation.render_template 'local_deployment.pc', binding
                      end
                Generation.save_automatic "#{name}.pc.in", pkg
                cmake = Generation.render_template 'config/Deployment.cmake', binding
                Generation.save_automatic "config/#{name}Deployment.cmake", cmake
            end
        end

        Spec::TaskDeployment.include TaskDeploymentGeneration
        Spec::Deployment.include DeploymentGeneration
    end
    end
end


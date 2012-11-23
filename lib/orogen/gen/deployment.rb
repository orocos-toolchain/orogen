require 'set'

module Orocos
    module Generation
        module TaskDeploymentGeneration
            def generate_activity_setup
                if @activity_setup
                    @activity_setup.call
                else
                    result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(
            #{rtt_scheduler},
            #{rtt_priority},
            task_#{name}.engine(),
            "#{name}");
                    EOD
                end
            end

            def to_deployer_xml
                @activity_xml.call
            end

            # Check for the case when there is an superfluous dot at
            # the end of a task statement
            def check_for_stray_dots(filename, name, args)
                # Building the regular expression to 
                # match on the method name and arguments
                regexp_expression = "#{name}.*"
                args.each do |element|
                    regexp_expression << "#{element}.*"
                end
                regexp = Regexp.new(regexp_expression)

                # Check the spec to locate the error in case
                # of stray dots
                File.open(filename) do |file|
                    begin 
                        line_counter = 0
                        previous_non_empty_line_number = 0
                        previous_non_empty_line = nil
                        while true
                            line = file.readline
                            line_counter += 1
                            if regexp.match(line)
                                if previous_non_empty_line =~ /\.$/
                                    raise ArgumentError, "stray dot in statement: #{previous_non_empty_line.strip} (line #{previous_non_empty_line_number})"
                                end
                            end

                            if line =~ /.+/
                                previous_non_empty_line = line
                                previous_non_empty_line_number = line_counter
                            end
                        end
                    rescue EOFError
                    end
                end
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
                    STDERR.puts "WARN: the deployment #{name} will do nothing. Either generate with --transports=corba or use the 'browse' statement"
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


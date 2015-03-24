require 'set'

module Orocos
    module Generation
        class CppProxyGeneration
            attr_reader :project
            attr_reader :proxy_gen
            attr_reader :task
            def initialize(cmp, t)
                @project = cmp
                @task = t
                @proxy_gen = self
            end
            
            def dependencies(includeSelf = true)
                result = []
                if(includeSelf and project.typekit)
                    tk = project.typekit
                    result << BuildDependency.new(
                        "#{tk.name}_TYPEKIT",
                        "#{tk.name}-typekit-gnulinux").
                        in_context('core', 'include').
                        in_context('core', 'link')
                        
                    project.enabled_transports.each do |transport_name|
                        result << BuildDependency.new(
                                "#{tk.name}_TRANSPORT_#{transport_name.upcase}",
                                "#{tk.name}-transport-#{transport_name}-gnulinux").
                                in_context('core', 'include').
                                in_context('core', 'link')
                    end
                end
                project.used_typekits.each do |tk|
                    next if tk.name == "rtt"
                    next if tk.name == "logger"
                    result << BuildDependency.new(
                        "#{tk.name}_TYPEKIT",
                        tk.pkg_name).
                        in_context('core', 'include').
                        in_context('core', 'link')
                        
                    project.enabled_transports.each do |transport_name|
                        result << BuildDependency.new(
                            "#{tk.name}_TRANSPORT_#{transport_name.upcase}",
                            tk.pkg_transport_name(transport_name)).
                            in_context('core', 'include').
                            in_context('core', 'link')
                    end                    
                end
                result << BuildDependency.new(
                            "orocos_cpp_base",
                            "orocos_cpp_base").
                            in_context('core', 'include').
                            in_context('core', 'link')
                
                result
            end

            # Generates the code associated with cpp proxies
            def generate
                proxy_hpp = Generation.render_template "proxies", "Task.hpp", binding
                proxy_cpp = Generation.render_template "proxies", "Task.cpp", binding
                file_proxy_hpp = Generation.save_automatic "proxies", "#{task.basename}.hpp", proxy_hpp
                file_proxy_cpp = Generation.save_automatic "proxies", "#{task.basename}.cpp", proxy_cpp

                cmake = Generation.render_template 'proxies', 'CMakeLists.txt', binding
                Generation.save_automatic('proxies', "CMakeLists.txt", cmake)
                
                pc = Generation.render_template "proxies", "proxies.pc", binding
                Generation.save_automatic "proxies", "#{project.name}-proxies.pc.in", pc
            end
        end
    end
end


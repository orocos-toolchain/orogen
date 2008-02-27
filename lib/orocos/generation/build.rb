require 'orocos/generation/base'

module Orocos
    class Generation
	def build_system
	    FileUtils.mkdir_p('.orocos')
	    FileUtils.cp_r Generation.template_path('config'), '.orocos'

	    component = self

	    # Generate the toplevel CMakeLists.txt
	    root_cmake = Generation.render_template 'CMakeLists.txt', binding
	    Generation.save_public_automatic 'CMakeLists.txt', root_cmake

	    # Generate the main.cpp file, which includes the ORO_main entry
	    # point
	    main = Generation.render_template 'main.cpp', binding
	    Generation.save_automatic 'main.cpp', main
	end
    end
end


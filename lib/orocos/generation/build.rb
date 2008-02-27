require 'orocos/generation/base'

module Orocos
    class Generation
	def build_system
	    FileUtils.mkdir_p('.orocos')
	    FileUtils.cp_r Generation.template_path('build', 'config'), '.orocos'

	    # Generate the toplevel CMakeLists.txt
	    template = Generation.load_template 'build', 'CMakeLists.txt'
	    Generation.save_user 'CMakeLists.txt', template.result(binding)

	    # Now, generate a CMakeLists.txt for all subdirectories of .orocos
	    # for which there is a template in build/
	    Dir.new('.orocos').each do |dir|
		template = begin
			       Generation.load_template 'build', "#{dir}-CMakeLists.txt"
			   rescue ArgumentError
			       next
			   end

		Generation.save_automatic dir, 'CMakeLists.txt', template.result(binding)
	    end
	end
    end
end


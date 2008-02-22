require 'orocos/generation/base'

module Orocos
    module Generation
	def self.build_system(component_name)
	    FileUtils.mkdir_p('.orocos')
	    FileUtils.cp_r template_path('build', 'config'), '.orocos'

	    # Generate the toplevel CMakeLists.txt
	    template = load_template 'build', 'CMakeLists.txt'
	    save_user 'CMakeLists.txt', template.result(binding)

	    # Now, generate a CMakeLists.txt for all subdirectories of .orocos
	    # for which there is a template in build/
	    Dir.new('.orocos').each do |dir|
		template = begin
			       load_template 'build', "#{dir}-CMakeLists.txt"
			   rescue ArgumentError
			       next
			   end

		save_automatic dir, 'CMakeLists.txt', template.result(binding)
	    end
	end
    end
end


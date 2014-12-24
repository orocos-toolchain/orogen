module OroGen
    module Gen
        module RTT_CPP
	AUTOMATIC_AREA_NAME = '.orogen'

	@templates = Hash.new
	class << self
	    # The set of templates already loaded as a path => ERB object hash
	    attr_reader :templates
	end

        # Returns the directory where Orogen's lib part sits (i.e. where
        # autobuild.rb and autobuild/ are)
        def self.base_dir
	    File.expand_path(File.join('..', '..'), File.dirname(__FILE__))
        end

	# call-seq:
	#   template_path(path1, path2, ..., file_name)
	#
	# Returns the full path for the template path1/path2/.../file_name.
	# Templates names are the path relative to the template base directory,
	# which is the orocos/templates directory directly in Orocos.rb
	# sources.
	def self.template_path(*path)
	    reldir = File.join('orogen', 'templates', *path)
	    File.expand_path(reldir, base_dir)
	end

	# call-seq:
	#   load_template path1, path2, ..., file_name => erb object
	#
	# Loads the template file located at
	# template_dir/path1/path2/.../file_name and return the ERB object
	# generated from it. template_dir is the templates/ directory located
	# in Orocos.rb sources.
	#
	# A template is only loaded once. See Generation.templates.
	def self.load_template(*path)
	    if template = templates[path]
		template
	    else
		template_file   = template_path(*path)

		templates[path] = begin
                                      ERB.new(File.read(template_file), nil, "<>", path.join('_').downcase.gsub(/[\/\.-]/, '_'))
				  rescue Errno::ENOENT
				      raise ArgumentError, "template #{File.join(*path)} does not exist"
				  end
                templates[path].filename = template_file
                templates[path]
	    end
	end

	# call-seq:
	#   render_template path1, path2, file_name, binding
	#
	# Render the template found at path1/path2/file_name and render it
	# using the provided binding
	def self.render_template(*args)
	    binding = args.pop
	    template = load_template(*args)
	    debug "rendering #{File.join(*args)}"
	    template.result(binding)
        rescue Exception => e
            raise e, "while rendering #{File.join(*args)}: #{e.message}", e.backtrace
	end

        class << self
            # The set of files generated so far, as a set of absolute paths
            attr_reader :generated_files
        end
        @generated_files = Set.new

	def self.save_generated(overwrite, *args) # :nodoc:
	    if args.size < 2
		raise ArgumentError, "expected at least 2 arguments, got #{args.size}"
	    end

	    data      = args.pop
	    file_path = File.expand_path(File.join(*args))
	    dir_name  = File.dirname(file_path)
	    FileUtils.mkdir_p(dir_name)

            generated_files << file_path
	    if File.exists?(file_path)
		if File.read(file_path) != data
		    if overwrite
			info "  overwriting #{file_path}"
		    else
			info "  will not overwrite #{file_path}"
			return file_path
		    end
		else
		    debug "  #{file_path} has not changed"
		    return file_path
		end
	    else
		info "  creating #{file_path}"
	    end

	    File.open(file_path, 'w') do |io|
		io.write data
	    end
            file_path
	end

	# call-seq:
	#   save_automatic path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/.../file_name file of the
	# automatically-generated part of the project (i.e. under .orogen)
	def self.save_automatic(*args)
	    save_generated true, AUTOMATIC_AREA_NAME, *args
	end
	
	# call-seq:
	#   save_public_automatic path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/file_name file of the
	# user-written part of the project. It differs from save_user because
	# it will happily overwrite an existing file.
	def self.save_public_automatic(*args)
	    save_generated true, *args
	end
	
	# call-seq:
	#   save_user path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/file_name file of the
	# user-written part of the project, if the said file does
	# not exist yet
	def self.save_user(*args)
	    result = save_generated false, *args

	    # Save the template in path1/path2/.../orogen/file_name
	    args = args.dup
	    args.unshift "templates"
	    save_generated true, *args
            result
	end

        # Removes from the given path all files that have not been generated
        def self.cleanup_dir(*path)
            dir_path = File.expand_path(File.join(*path))

            Find.find(dir_path) do |file|
                if File.directory?(file) && File.directory?(File.join(file, "CMakeFiles"))
                    # This looks like a build directory. Ignore
                    Find.prune
                
                elsif File.file?(file) && !File.symlink?(file) && !generated_files.include?(file)
                    info "   removing #{file}"
                    FileUtils.rm_f file
                end
            end
        end

	def self.really_clean
	    # List all files in templates and compare them w.r.t.  the ones in
	    # the user-side of the project. Remove those that are identical
	    base_dir     = Pathname.new('.')
	    template_dir = Pathname.new('templates')
	    template_dir.find do |path|
		next unless path.file?
		template_data = File.read(path.to_s)
		relative = path.relative_path_from(template_dir)

		if relative.file?
		    user_data = File.read(relative.to_s)
		    if user_data == template_data
			logger.info "removing #{relative} as it is the same than in template"
			FileUtils.rm_f relative.to_s
		    end
		end
	    end
	    
	    # Call #clean afterwards, since #clean removes the templates/ directory
	    clean
	end

	def self.clean
	    FileUtils.rm_rf Generation::AUTOMATIC_AREA_NAME
	    FileUtils.rm_rf "build"
	    FileUtils.rm_rf "templates"
	end
        end
    end
end


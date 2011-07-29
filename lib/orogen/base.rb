require 'utilrb/module/include'
require 'utilrb/logger'
require 'fileutils'
require 'erb'
require 'typelib'

require 'fileutils'
require 'set'
require 'find'

class Module
    def enumerate_inherited_set(each_name, attribute_name = each_name) # :nodoc:
	class_eval <<-EOD
	def all_#{attribute_name}; each_#{each_name}.to_a end
	def self_#{attribute_name}; @#{attribute_name} end
	def each_#{each_name}(&block)
	    if block_given?
		if superclass
		    superclass.each_#{each_name}(&block)
		end
		@#{attribute_name}.each(&block)
	    else
		enum_for(:each_#{each_name})
	    end
	end
	EOD
    end

    def enumerate_inherited_map(each_name, attribute_name = each_name) # :nodoc:
	class_eval <<-EOD
	def all_#{attribute_name}; each_#{each_name}.to_a end
	def self_#{attribute_name}; @#{attribute_name}.values end
	def has_#{attribute_name}?(name); !!find_#{each_name}(name) end

	def find_#{each_name}(name)
	    if v = @#{attribute_name}[name]
		v
	    elsif superclass
		superclass.find_#{each_name}(name)
	    end
	end
	def each_#{each_name}(&block)
	    if block_given?
		if superclass
		    superclass.each_#{each_name}(&block)
		end
		@#{attribute_name}.each_value(&block)
	    else
		enum_for(:each_#{each_name})
	    end
	end
	EOD
    end

    # call-seq:
    #   dsl_attribute(name)
    #   dsl_attribute(name) { |value| ... }
    #
    # This defines a +name+ instance method on the given class which accepts zero or one argument
    #
    # Without any argument, it acts as a getter for the +@name+ attribute. With
    # one argument, it acts instead as a setter for the same attribute and
    # returns self. If a block has been given to +dsl_attribute+, any new value
    # is passed to the block, whose return value is actually saved in the
    # instance variable.  This block can therefore both filter the value
    # (convert it to a desired form) and validate it.
    #
    # The goal of this method is to have a nicer way to handle attribute in DSLs: instead
    # of 
    #
    #    model = create_model do
    #	    self.my_model_attribute = 'bla'
    #
    #	    if (my_model_attribute)
    #		<do something>
    #	    end
    #	 end
    #
    # (or worse, using set_ and get_ prefixes), we can do
    #
    #    model = create_model do
    #	    my_model_attribute 'bla', arg0, arg1, ...
    #
    #	    if (my_model_attribute)
    #		<do something>
    #	    end
    #	 end
    #
    def dsl_attribute(name, &filter_block)
	class_eval do
            if filter_block
                define_method("__dsl_attribute__#{name}__filter__", &filter_block)
            end

	    define_method(name) do |*value|
		if value.empty?
		    instance_variable_get("@#{name}")
		elsif filter_block
                    if filter_block.arity >= 0 && value.size != filter_block.arity
                        raise ArgumentError, "too much arguments. Got #{value.size}, expected #{filter_block.arity}"
                    end

		    filtered_value = send("__dsl_attribute__#{name}__filter__", *value)
		    instance_variable_set("@#{name}", filtered_value)
		    self
		else
                    if value.size == 1
                        instance_variable_set("@#{name}", value.first)
                    else
                        instance_variable_set("@#{name}", value)
                    end
		    self
		end
	    end
	end
    end
end


module Orocos
    OROGEN_LIB_DIR = File.expand_path(File.dirname(__FILE__))
    module Generation
        class InternalError < RuntimeError; end

        class ConfigError < Exception; end
	AUTOMATIC_AREA_NAME = '.orogen'


	class << self
	    attr_reader :logger
	end
	@logger = Logger.new(STDOUT)
	logger.level = Logger::WARN
        logger.formatter = lambda { |severity, time, progname, msg| "#{severity}: #{msg}\n" }
	extend Logger::Forward
 
	@templates = Hash.new
	class << self
	    # The set of templates already loaded as a path => ERB object hash
	    attr_reader :templates
	end

        # Returns the directory where Orogen's lib part sits (i.e. where
        # autobuild.rb and autobuild/ are)
        def self.base_dir
	    File.expand_path('..', File.dirname(__FILE__))
        end

	# call-seq:
	#   template_path(path1, path2, ..., file_name)
	#
	# Returns the full path for the template path1/path2/.../file_name.
	# Templates names are the path relative to the template base directory,
	# which is the orocos/templates directory directly in Orocos.rb
	# sources.
	def self.template_path(*path)
	    reldir = File.join('templates', *path)
	    File.expand_path(reldir, File.dirname(__FILE__))
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
		template_data   = begin
				      File.open(template_path(*path))
				  rescue Errno::ENOENT
				      raise ArgumentError, "template #{File.join(*path)} does not exist"
				  end

		templates[path] = ERB.new(template_data.read, nil, nil, path.join('_').gsub(/[\/\.-]/, '_'))
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
	    logger.debug "rendering #{File.join(*args)}"
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
			logger.info "  overwriting #{file_path}"
		    else
			logger.info "  will not overwrite #{file_path}"
			return file_path
		    end
		else
		    logger.debug "  #{file_path} has not changed"
		    return file_path
		end
	    else
		logger.info "  creating #{file_path}"
	    end

	    File.open(file_path, 'w') do |io|
		io.write data
	    end
            file_path
	end

        # Removes from the given path all files that have not been generated
        def self.cleanup_dir(*path)
            dir_path = File.expand_path(File.join(*path))

            Find.find(dir_path) do |file|
                if File.directory?(file) && File.directory?(File.join(file, "CMakeFiles"))
                    # This looks like a build directory. Ignore
                    Find.prune
                
                elsif File.file?(file) && !File.symlink?(file) && !generated_files.include?(file)
                    logger.info "   removing #{file}"
                    FileUtils.rm_f file
                end
            end
        end

        # call-seq:
        #   touch path1, path2, ..., file_name
        #
        # Creates an empty file path1/path2/.../file_name
        def self.touch(*args)
            path = File.expand_path(File.join(*args))
            FileUtils.touch path
            generated_files << path
        end

	# call-seq:
	#   save_automatic path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/.../file_name file of the
	# automatically-generated part of the component (i.e. under .orogen)
	def self.save_automatic(*args)
	    save_generated true, AUTOMATIC_AREA_NAME, *args
	end
	
	# call-seq:
	#   save_public_automatic path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/file_name file of the
	# user-written part of the component. It differs from save_user because
	# it will happily overwrite an existing file.
	def self.save_public_automatic(*args)
	    save_generated true, *args
	end
	
	# call-seq:
	#   save_user path1, path2, ..., file_name, data
	#
	# Save the provided data in the path1/path2/file_name file of the
	# user-written part of the component, if the said file does
	# not exist yet
	def self.save_user(*args)
	    result = save_generated false, *args

	    # Save the template in path1/path2/.../orogen/file_name
	    args = args.dup
	    args.unshift "templates"
	    save_generated true, *args
            result
	end

	# Returns the C++ code which changes the current namespace from +old+
	# to +new+. +indent_size+ is the count of indent spaces between
	# namespaces.
	def self.adapt_namespace(old, new, indent_size = 4)
	    old = old.split('/').delete_if { |v| v.empty? }
	    new = new.split('/').delete_if { |v| v.empty? }
	    indent = old.size * indent_size

	    result = ""

	    while !old.empty? && old.first == new.first
		old.shift
		new.shift
	    end
	    while !old.empty?
		indent -= indent_size
		result << " " * indent + "}\n"
		old.shift
	    end
	    while !new.empty?
		result << "#{" " * indent}namespace #{new.first} {\n"
		indent += indent_size
		new.shift
	    end

	    result
	end

	def self.really_clean
	    # List all files in templates and compare them w.r.t.  the ones in
	    # the user-side of the component. Remove those that are identical
	    base_dir     = Pathname.new('.')
	    template_dir = Pathname.new('templates')
	    template_dir.find do |path|
		next unless path.file?
		template_data = File.read(path.to_s)
		relative = path.relative_path_from(template_dir)

		if relative.file?
		    user_data = File.read(relative.to_s)
		    if user_data == template_data
			Generation.logger.info "removing #{relative} as it is the same than in template"
			FileUtils.rm_f relative.to_s
		    end
		end
	    end
	    
	    # Call #clean afterwards, since #clean removes the templates/ directory
	    clean
	end

        # Returns the unqualified version of +type_name+
        def self.unqualified_cxx_type(type_name)
            type_name.
                gsub(/(^|[^\w])const($|[^\w])/, '').
                gsub(/&/, '').
                strip
        end

	def self.clean
	    FileUtils.rm_rf Generation::AUTOMATIC_AREA_NAME
	    FileUtils.rm_rf "build"
	    FileUtils.rm_rf "templates"
	end

        class BuildDependency
            attr_reader :var_name
            attr_reader :pkg_name

            attr_reader :context

            def initialize(var_name, pkg_name)
                @var_name = var_name
                @pkg_name = pkg_name
                @context = []
            end

            def in_context(*args)
                context << args.to_set
                self
            end

            def remove_context(*args)
                args = args.to_set
                @context = context.dup
                context.delete_if do |ctx|
                    (args & ctx).size == args.size
                end
                self
            end

            def in_context?(*args)
                args = args.to_set
                context.any? do |ctx|
                    (args & ctx).size == args.size
                end
            end
        end

        def self.cmake_pkgconfig_require(depspec, context = 'core')
            depspec.inject([]) do |result, s|
                result << "pkg_check_modules(#{s.var_name} REQUIRED #{s.pkg_name})"
                if s.in_context?(context, 'include')
                    result << "include_directories(${#{s.var_name}_INCLUDE_DIRS})"
                    result << "add_definitions(${#{s.var_name}_CFLAGS_OTHER})"
                end
                if s.in_context?(context, 'link')
                    result << "foreach(#{s.var_name}_lib ${#{s.var_name}_LIBRARIES})"
                    result << "  set(_#{s.var_name}_lib NOTFOUND)"
                    result << "  find_library(_#{s.var_name}_lib NAMES ${#{s.var_name}_lib} HINTS ${#{s.var_name}_LIBRARY_DIRS})"
                    result << "  if (NOT _#{s.var_name}_lib)"
                    result << "    set(_#{s.var_name}_lib ${#{s.var_name}_lib})"
                    result << "  endif()"
                    result << "  list(APPEND _#{s.var_name}_LIBRARIES ${_#{s.var_name}_lib})"
                    result << "endforeach()"
                    result << "set(#{s.var_name}_LIBRARIES ${_#{s.var_name}_LIBRARIES})"
                end
                result
            end.join("\n")
        end

        def self.cmake_pkgconfig_link(context, target, depspec)
            depspec.inject([]) do |result, s|
                if s.in_context?(context, 'link')
                    result << "target_link_libraries(#{target} ${#{s.var_name}_LIBRARIES})"
                end
                result
            end.join("\n")
        end

        def self.cmake_pkgconfig_link_corba(target, depspec)
            cmake_pkgconfig_link('corba', target, depspec)
        end
        def self.cmake_pkgconfig_link_noncorba(target, depspec)
            cmake_pkgconfig_link('core', target, depspec)
        end

        def self.verify_valid_identifier(name)
            name = name.to_s if name.respond_to?(:to_sym)
            if name !~ /^[a-zA-Z0-9_][a-zA-Z0-9_]*$/
                raise ArgumentError, "task name '#{name}' invalid: it can contain only alphanumeric characters and '_', and cannot start with a number"
            end
            name
        end

        def self.each_orogen_plugin_dir(&block)
            if dirs = ENV['OROGEN_PLUGIN_PATH']
                dirs.split(':').each(&block)
            else
                [].each(&block)
            end
        end

        def self.load_plugins
	    original_load_path = $LOAD_PATH.dup
            each_orogen_plugin_dir do |dir|
		$LOAD_PATH << dir
	    end
            each_orogen_plugin_dir do |dir|
	        Dir.glob(File.join(dir, '*.rb')).each do |file|
	            logger.info "loading plugin #{file}"
		    begin
	                require file
		    rescue Exception => e
		        logger.warn "could not load plugin #{file}: #{e.message}"
		    end
	        end
            end
	ensure
	    if original_load_path
		$LOAD_PATH.clear
	        $LOAD_PATH.concat(original_load_path)
	    end
        end
    end

    # Load a separate typelib registry containing the types defined by the given
    # oroGen project
    def self.registry_of(typekit_name)
        registry = Typelib::Registry.new
        typekit_pkg =
            Utilrb::PkgConfig.new("#{typekit_name}-typekit-#{Orocos::Generation.orocos_target}")

        tlb = typekit_pkg.type_registry
        if tlb
            registry.import(tlb)
        end

        registry
    end
end


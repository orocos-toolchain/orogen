require 'utilrb/module/dsl_attribute'
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
	class_eval <<-EOD, __FILE__, __LINE__
	def find_#{attribute_name}(name) 
            each_#{each_name} do |n|
                return n if n.name == name
            end
        end
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
	class_eval <<-EOD, __FILE__, __LINE__
        attr_reader :#{attribute_name}
	def all_#{attribute_name}; each_#{each_name}.to_a end
	def self_#{attribute_name}; @#{attribute_name}.values end
	def has_#{attribute_name}?(name); !!find_#{each_name}(name) end

	def find_#{each_name}(name)
            name = name.to_str
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
end


module OroGen
    def self.beautify_loading_errors(filename)
        yield
    rescue Exception => e
        # Two options:
        #  * the first line of the backtrace is the orogen file
        #    => change it into a ConfigError. If, in addition, this is a
        #       NoMethodError then change it into a statement error
        #  * the second line of the backtrace is in the orogen file
        #    => most likely a bad argument, transform it into a ConfigError
        #       too
        #  * all other cases are reported as internal errors
        file_pattern = /#{Regexp.quote(File.basename(filename))}/
        if e.backtrace.first =~ file_pattern
            if e.kind_of?(NoMethodError) || e.kind_of?(NameError)
                e.message =~ /undefined (?:local variable or )?method `([^']+)'/
                method_name = $1
                raise Generation::ConfigError, "unknown statement '#{method_name}'", e.backtrace
            else
                raise Generation::ConfigError, e.message, e.backtrace
            end
        elsif (e.backtrace[1] =~ file_pattern) || e.kind_of?(ArgumentError)
            raise Generation::ConfigError, e.message, e.backtrace
        end
        raise
    end

    def self.verify_valid_identifier(name)
        name = name.to_s if name.respond_to?(:to_sym)
        name = name.to_str
        if name !~ /^[a-zA-Z0-9_][a-zA-Z0-9_]*$/
            raise ArgumentError, "task name '#{name}' invalid: it can contain only alphanumeric characters and '_', and cannot start with a number"
        end
        name
    end

    def self.validate_toplevel_type(type)
        if type < Typelib::ArrayType
            raise Generation::ConfigError, "array types can be used only in a structure"
        end
    end

    # Returns the unqualified version of +type_name+
    def self.unqualified_cxx_type(type_name)
        type_name.
            gsub(/(^|[^\w])const($|[^\w])/, '').
            gsub(/&/, '').
            strip
    end

    def self.orocos_target=(target)
        @orocos_target = target.to_s
    end

    @orocos_target = nil
    def self.orocos_target
        user_target = ENV['OROCOS_TARGET']
        if @orocos_target
            @orocos_target.dup
        elsif user_target && !user_target.empty?
            user_target
        else
            'gnulinux'
        end
    end
end


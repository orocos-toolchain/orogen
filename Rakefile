require './lib/orogen/version'
require 'shellwords'

task :setup, :cmake_args do |t, args|
    args.with_defaults(:cmake_args => "")
    begin
        cmake_args = Shellwords.split(args[:cmake_args])
        if cmake_args.grep(/-DOROCOS_TARGET=/).empty? && ENV['OROCOS_TARGET']
            cmake_args << "-DOROCOS_TARGET=#{ENV['OROCOS_TARGET']}"
        end

        FileUtils.mkdir_p File.join('rtt-typelib', 'build')
        Dir.chdir(File.join('rtt-typelib', 'build')) do
            puts "running cmake with #{cmake_args.join(" ")}"
            if !system('cmake', '..', *cmake_args)
                raise "cannot configure the rtt-typelib's build system"
            end
            if !system('make', 'install')
                raise "cannot build or install the rtt-typelib support library"
            end
        end

        require 'typelib'
        require 'orogen'
        STDERR.puts "oroGen is ready to use"
    rescue LoadError => e
        STDERR.puts "cannot load oroGen: #{e.message}"
        STDERR.puts "  did you install Typelib's Ruby bindings and update the RUBYLIB environment variable accordingly ?"
        STDERR.puts "  did you add #{File.expand_path("lib", File.dirname(__FILE__))} to RUBYLIB ?"
        exit(1)
    end
end
task :default => :setup

begin
    require 'hoe'
    namespace 'dist' do
        config = Hoe.spec 'orogen' do
            self.developer "Sylvain Joyeux", "sylvain.joyeux@dfki.de"

            self.summary = 'Component generation for Orocos::RTT'
            self.description = paragraphs_of('README.txt', 3..6).join("\n\n")
            self.changes     = paragraphs_of('History.txt', 0..1).join("\n\n")

            extra_deps << 
                ['utilrb',   '>= 1.3.4'] <<
                ['rake',     '>= 0.8'] <<
                ['nokogiri', '>= 1.3.3']
        end

        Rake.clear_tasks(/dist:publish_docs/)
        Rake.clear_tasks(/dist:(re|clobber_|)docs/)
        task 'publish_docs' => 'redocs' do
            if !system('doc/misc/update_github')
                raise "cannot update the gh-pages branch for GitHub"
            end
            if !system('git', 'push', 'git@github.com:doudou/orogen.git', '+gh-pages')
                raise "cannot push the documentation"
            end
        end
    end

rescue LoadError
    STDERR.puts "cannot load the Hoe gem. Distribution is disabled"
rescue Exception => e
    if e.message !~ /\.rubyforge/
        STDERR.puts "WARN: cannot load the Hoe gem, or Hoe fails. Publishing tasks are disabled"
        STDERR.puts "WARN: error message is: #{e.message}"
    end
end

do_doc = begin
             require 'rdoc/task'
             true
         rescue LoadError => e
             STDERR.puts "WARN: cannot load RDoc, documentation generation disabled"
             STDERR.puts "WARN:   #{e.message}"
         end

if do_doc
    task 'docs' => 'doc:all'
    task 'clobber_docs' => 'doc:clobber'
    task 'redocs' do
        Rake::Task['clobber_docs'].invoke
        if !system('rake', 'doc:all')
            raise "failed to regenerate documentation"
        end
    end

    namespace 'doc' do
        task 'all' => %w{api}
        task 'clobber' => 'clobber_api'
        RDoc::Task.new("api") do |rdoc|
            rdoc.rdoc_dir = 'doc'
            rdoc.title    = "oroGen"
            rdoc.options << '--show-hash'
            rdoc.rdoc_files.include('lib/**/*.rb')
        end
    end
end


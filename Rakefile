require './lib/orogen/version'

task :setup do
    begin
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

            extra_dev_deps <<
                ['webgen', '>= 0.5.9']
        end

        Rake.clear_tasks(/dist:publish_docs/)
        Rake.clear_tasks(/dist:(re|clobber_|)docs/)
        task 'publish_docs' => 'redocs' do
            if !system('doc/misc/update_github')
                raise "cannot update the gh-pages branch for GitHub"
            end
            if !system('git', 'push', 'github', 'gh-pages')
                raise "cannot push the documentation"
            end
        end
    end

rescue LoadError
    STDERR.puts "cannot load the Hoe gem. Distribution is disabled"
rescue Exception => e
    if e.message !~ /\.rubyforge/
        STDERR.puts "cannot load the Hoe gem, or Hoe fails. Distribution is disabled"
        STDERR.puts "error message is: #{e.message}"
    end
end

do_doc = begin
             require 'webgen/webgentask'
             require 'rdoc/task'
             true
         rescue LoadError => e
             STDERR.puts "ERROR: cannot load webgen and/or RDoc, documentation generation disabled"
             STDERR.puts "ERROR:   #{e.message}"
         end

if do_doc
    task 'doc' => 'doc:all'
    task 'clobber_docs' => 'doc:clobber'
    task 'redocs' do
        Rake::Task['clobber_docs'].invoke
        if !system('rake', 'doc:all')
            raise "failed to regenerate documentation"
        end
    end

    namespace 'doc' do
        task 'all' => %w{guide api}
        task 'clobber' => 'clobber_guide'
        Webgen::WebgenTask.new('guide') do |website|
            website.clobber_outdir = true
            website.directory = File.join(Dir.pwd, 'doc', 'guide')
            website.config_block = lambda do |config|
                config['output'] = ['Webgen::Output::FileSystem', File.join(Dir.pwd, 'doc', 'html')]
            end
        end
        RDoc::Task.new("api") do |rdoc|
            rdoc.rdoc_dir = 'doc/html/api'
            rdoc.title    = "oroGen"
            rdoc.options << '--show-hash'
            rdoc.rdoc_files.include('lib/**/*.rb')
        end
    end
end


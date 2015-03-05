require './lib/orogen/version'
require 'shellwords'

task :setup do
    begin
        require 'typelib'
        require 'orogen'
        STDERR.puts "oroGen is ready to use"
    rescue Exception => e
        STDERR.puts "cannot load oroGen"
        STDERR.puts "  did you install Typelib's Ruby bindings and update the RUBYLIB environment variable accordingly ?"
        STDERR.puts "  did you add #{File.expand_path("lib", File.dirname(__FILE__))} to RUBYLIB ?"
        STDERR.puts "the error is: #{e.message}"
        e.backtrace.each do |line|
            STDERR.puts "  #{line}"
        end
        exit(1)
    end
end

begin
    require 'hoe'
    Hoe::plugin :yard
    Hoe::RUBY_FLAGS.gsub!(/-w/, '')

    config = Hoe.spec 'orogen' do
        self.developer "Sylvain Joyeux", "sylvain.joyeux@dfki.de"

        self.summary = 'Component generation for Orocos::RTT'
        self.description = paragraphs_of('README.markdown', 3..6).join("\n\n")
        self.changes     = paragraphs_of('History.txt', 0..1).join("\n\n")
        licenses << "GPLv2 or later"

        extra_deps <<
            ['utilrb',   '>= 1.3.4'] <<
            ['rake',     '>= 0.8'] <<
            ['hoe-yard', '>= 0.1.2']

        extra_dev_deps <<
            ['flexmock', '>= 0.8.6']
    end

    namespace 'dist' do
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
    hoe_spec.test_globs = ['test/suite.rb']

    task :doc => :yard
rescue LoadError
    STDERR.puts "cannot load the Hoe gem. Distribution is disabled"
rescue Exception => e
    if e.message !~ /\.rubyforge/
        STDERR.puts "WARN: cannot load the Hoe gem, or Hoe fails. Publishing tasks are disabled"
        STDERR.puts "WARN: error message is: #{e.message}"
    end
end

Rake.clear_tasks(/^default$/)
task :default => :setup

require 'utilrb/doc/rake'
Utilrb.doc :include => ['lib/**/*.rb'],
    :title => 'oroGen',
    :plugins => ['utilrb']


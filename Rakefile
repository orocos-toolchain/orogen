# frozen_string_literal: true

require './lib/orogen/version'
require 'shellwords'

require 'bundler/gem_tasks'
require 'rake/testtask'

task :setup do
    begin
        require 'typelib'
        require 'orogen'
    rescue ScriptErrors => e
        warn <<~MESSAGE
            cannot load oroGen
            - did you install Typelib\'s Ruby bindings and update the RUBYLIB environment
            variable accordingly ?
            - did you add #{File.expand_path('lib', __dir__)} to RUBYLIB ?

            The error is: #{e.message}
        MESSAGE
        e.backtrace.each do |line|
            warn "  #{line}"
        end
        exit(1)
    end
end

task default: :setup

Rake::TestTask.new(:test) do |t|
    t.libs << 'lib'
    t.libs << 'test'
    t.warning = false
    t.test_files = FileList['test/**/test_*.rb']
                   .exclude('test/gen/**/*.rb')
end

Rake::TestTask.new('test:gen') do |t|
    t.libs << 'lib'
    t.libs << 'test'
    t.warning = false
    t.test_files = FileList['test/gen/**/test_*.rb']
end

task 'test:all' => ['test', 'test:gen']

task gem: :build

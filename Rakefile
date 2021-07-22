# frozen_string_literal: true

require "./lib/orogen/version"
require "shellwords"

require "bundler/gem_tasks"
require "rake/testtask"

task :setup do
    begin
        require "typelib"
        require "orogen"
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

ENV["OROGEN_DISABLE_PLUGINS"] = "1"

TESTOPTS = ENV.delete("TESTOPTS") || ""

RUBOCOP_REQUIRED = (ENV["RUBOCOP"] == "1")
USE_RUBOCOP = (ENV["RUBOCOP"] != "0")
USE_JUNIT = (ENV["JUNIT"] == "1")
REPORT_DIR = ENV["REPORT_DIR"] || File.expand_path("test_reports", __dir__)

def minitest_set_options(test_task, name)
    minitest_options = []
    if USE_JUNIT
        minitest_options += [
            "--junit", "--junit-jenkins",
            "--junit-filename=#{REPORT_DIR}/#{name}.junit.xml"
        ]
    end

    minitest_args =
        if minitest_options.empty?
            ""
        else
            "\"" + minitest_options.join("\" \"") + "\""
        end
    test_task.options = "#{TESTOPTS} #{minitest_args} -- --simplecov-name=#{name}"
end

Rake::TestTask.new(:test) do |t|
    t.libs << "lib"
    minitest_set_options(t, "core")
    t.libs << "test"
    t.warning = false

    t.test_files = FileList.new("test/**/test_*.rb") do |fl|
        fl.exclude { |p| %r{^test/gen}.match?(p) && (p != "test/gen/test_base.rb") }
    end
end

Rake::TestTask.new("test:gen") do |t|
    t.libs << "lib"
    minitest_set_options(t, "gen")
    t.libs << "test"
    t.warning = false
    t.test_files = FileList["test/gen/**/test_*.rb"]
end

task "test:all" => ["test", "test:gen"]

if USE_RUBOCOP
    begin
        require "rubocop/rake_task"
        RuboCop::RakeTask.new do |t|
            if USE_JUNIT
                t.formatters << "junit"
                t.options << "-o" << "#{REPORT_DIR}/rubocop.junit.xml"
            end
        end
        task "test" => "rubocop"
    rescue LoadError
        raise if RUBOCOP_REQUIRED
    end
end

task gem: :build

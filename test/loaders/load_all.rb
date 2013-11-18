require 'orogen'
require 'orogen/loaders'


loader = OroGen::Loaders::PkgConfig.new(ENV['OROCOS_TARGET'])
OroGen::Loaders::RTT.setup_loader(loader)
loader.available_projects.each_key do |name|
    puts "loading #{name}"
    orogen = loader.project_model_from_name(name)
    puts "loaded #{name}"
end

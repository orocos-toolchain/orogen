require 'orogen'
require 'orogen/loaders'

#OroGen.logger.level = Logger::DEBUG
loader = OroGen::Loaders::Aggregate.new
OroGen::Loaders::RTT.setup_loader(loader)
pkgconfig_loader = OroGen::Loaders::PkgConfig.new(ENV['OROCOS_TARGET'], loader)
loader.add pkgconfig_loader
pkgconfig_loader.available_projects.each_key do |name|
    puts "loading #{name}"
    orogen = loader.project_model_from_name(name)
    puts "loaded #{name}"
end

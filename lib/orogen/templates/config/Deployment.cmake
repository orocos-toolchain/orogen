# Task files could be using headers in tasks/ so add the relevant directory in
# our include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/<%= project.name %>)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>)

<% dependencies = deployer.dependencies %>
<%= Generation.cmake_pkgconfig_require(dependencies) %>

# Link directories need to be set before(!) providing the target
orogen_pkg_check_modules(service_discovery service_discovery)
if(service_discovery_FOUND)
    add_definitions(-DOROGEN_SERVICE_DISCOVERY_ACTIVATED)
    message(STATUS "ServiceDiscovery library found: activating service discovery functionality for tasks")
    include_directories(${service_discovery_INCLUDE_DIRS})
    add_definitions(${service_discovery_CFLAGS_OTHER})
    link_directories(${service_discovery_LIBRARY_DIRS})
endif()

find_package(Boost REQUIRED COMPONENTS system program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_definitions(-DRTT_COMPONENT)
add_executable(<%= deployer.name %> ${CMAKE_CURRENT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/main-<%= deployer.name %>.cpp)
<% if project.typekit %>
target_link_libraries(<%= deployer.name %> <%= project.name %>-typekit-${OROCOS_TARGET})
<% deployer.transports.each do |transport_name| %>
target_link_libraries(<%= deployer.name %> <%= project.name %>-transport-<%= transport_name %>-${OROCOS_TARGET})
<% end %>
<% end %>

if(service_discovery_FOUND)
    target_link_libraries(<%= deployer.name %> ${service_discovery_LIBRARIES})
endif()

target_link_libraries(<%= deployer.name %> ${Boost_PROGRAM_OPTIONS_LIBRARIES} ${Boost_SYSTEM_LIBRARIES})

list(APPEND CMAKE_PREFIX_PATH ${OrocosRTT_PREFIX})
find_package(RTTPlugin COMPONENTS rtt-typekit <%= deployer.rtt_transports.map { |transport_name| "rtt-transport-#{transport_name}" }.join(" ") %>)
target_link_libraries(<%= deployer.name %> ${RTT_PLUGIN_rtt-typekit_LIBRARY})
<% deployer.rtt_transports.each do |transport_name| %>
target_link_libraries(<%= deployer.name %> ${RTT_PLUGIN_rtt-transport-<%= transport_name %>_LIBRARY})
<% end %>
<% if !project.self_tasks.empty? %>
target_link_libraries(<%= deployer.name %> <%= project.name %>-tasks-${OROCOS_TARGET})
<% end %>
<%= Generation.cmake_pkgconfig_link_noncorba(deployer.name, dependencies) %>

<%= if deployer.corba_enabled?
Generation.cmake_pkgconfig_link_corba(deployer.name, dependencies)
end %>

<% if deployer.install? %>
install(TARGETS <%= deployer.name %>
    RUNTIME DESTINATION bin)
<% end %>

add_dependencies(<%= deployer.name %>
    check-uptodate)

configure_file(<%= Generation::AUTOMATIC_AREA_NAME %>/<%= deployer.name %>.pc.in
    orogen-<%= deployer.name %>.pc @ONLY)
<% if deployer.install? %>
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/orogen-<%= deployer.name %>.pc
    DESTINATION lib/pkgconfig)
<% end %>


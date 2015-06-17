# Generated from orogen/lib/orogen/templates/typekit/build.cmake

cmake_minimum_required(VERSION 2.8.3)

set(OROCOS_PKG_DIR "${OROCOS_SUFFIX}/${PROJECT_NAME}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/config")
include(OrogenPkgCheckModules)

execute_process(COMMAND cmake -E create_symlink
    ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>)

include_directories(BEFORE ${CMAKE_CURRENT_LIST_DIR})
include_directories(BEFORE ${CMAKE_CURRENT_LIST_DIR}/types)

<% if typekit.has_opaques_with_templates? %>
include_directories(BEFORE ${CMAKE_SOURCE_DIR}/typekit)
<% end %>

# Now set up the dependencies
<%= typekit_deps = typekit.dependencies
    Generation.cmake_pkgconfig_require(typekit_deps) %>
set(PKG_CFLAGS ${OrocosRTT_CFLAGS})
<%  typekit_deps.each do |dep_def|
        if dep_def.in_context?('core', 'include') %>
set(PKG_CFLAGS ${PKG_CFLAGS} ${<%= dep_def.var_name %>_CFLAGS})
        <% end %>
    <% end %>
string(REPLACE ";" "\" \"" PKG_CFLAGS "\"${PKG_CFLAGS}\"")

# Generate the base typekit shared library
set(libname <%= typekit.name %>-typekit)
orocos_typekit(${libname}
    <%= relatives = []
        implementation_files.each do |file|
        relatives << typekit.relative_path(file)
        end
        relatives.sort.map { |filepath| '${CMAKE_CURRENT_LIST_DIR}/'+filepath }.join("\n    ") %>
    ${TYPEKIT_ADDITIONAL_SOURCES}
    ${TOOLKIT_ADDITIONAL_SOURCES})

<%= Generation.cmake_pkgconfig_link_noncorba('${libname}', typekit_deps) %>
target_link_libraries(${libname} ${TYPEKIT_ADDITIONAL_LIBRARIES} ${TOOLKIT_ADDITIONAL_LIBRARIES})
target_link_libraries(${libname} LINK_INTERFACE_LIBRARIES)
set_target_properties(${libname} PROPERTIES INTERFACE_LINK_LIBRARIES "")
set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "-Wno-unused-private-field")
set(PKG_CONFIG_FILE ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-typekit-${OROCOS_TARGET}.pc)
configure_file(${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-typekit.pc.in ${PKG_CONFIG_FILE} @ONLY)

install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/Types.hpp
    ${CMAKE_CURRENT_LIST_DIR}/Opaques.hpp
    DESTINATION include/orocos/<%= typekit.name %>)

install(FILES
    <% if typekit.has_opaques_with_templates? %>
    ${PROJECT_SOURCE_DIR}/typekit/Opaques.hpp
    <% end %>
    <%= relatives = []
        plugin_header_files.each do |file|
        relatives << typekit.relative_path(file)
    end
    relatives.sort.map { |filepath| '${CMAKE_CURRENT_LIST_DIR}/'+filepath }.join("\n    ") %>
    DESTINATION include/orocos/<%= typekit.name %>/typekit)

<% (public_header_files + typekit.included_files).each do |inc|
    full_path = Pathname.new(typekit.automatic_dir).join('types', inc)
    next if !full_path.exist?
    target_dir = File.dirname(inc)
    if full_path.symlink?
       full_path = full_path.readlink
    end
    source_file = full_path.relative_path_from(Pathname.new(typekit.automatic_dir)) %>
install(FILES ${CMAKE_CURRENT_LIST_DIR}/<%= source_file.to_path %>
    DESTINATION include/orocos/<%= target_dir %>)
<% end %>

install(FILES ${PKG_CONFIG_FILE}
    DESTINATION lib/pkgconfig)

install(FILES ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>.tlb
    ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>.typelist
    DESTINATION share/orogen)

<% typekit.each_plugin do |plg|
    if plg.separate_cmake? %>
    include(${CMAKE_CURRENT_LIST_DIR}/transports/<%= plg.name %>/build.cmake)
    <% end
end %>

# Force the user to regenerate its typekit if the inputs changed
set(TK_STAMP "${CMAKE_CURRENT_LIST_DIR}/stamp")
get_directory_property(include_path INCLUDE_DIRECTORIES)
foreach(dep <%= typekit.included_files.to_set.sort.join(" ") %>)
    find_file(dep_full_path ${dep} PATHS ${include_path})
    if (NOT dep_full_path)
        message(FATAL_ERROR "cannot find ${dep} in ${include_path}. This should never happen, report this as an oroGen bug")
    endif()
    list(APPEND TYPEKIT_SOURCE_HEADERS ${dep_full_path})
    unset(dep_full_path CACHE)
endforeach()
add_custom_command(
    OUTPUT "${TK_STAMP}"
    DEPENDS ${TYPEKIT_SOURCE_HEADERS}
    COMMENT "Typekit input changed. Run make <%= typekit.name %>-regen in your build directory first"
    COMMAND /bin/false)
add_custom_target(<%= typekit.name %>-check-typekit-uptodate ALL DEPENDS "${TK_STAMP}")
add_dependencies(${libname} <%= typekit.name %>-check-typekit-uptodate)
add_dependencies(check-uptodate <%= typekit.name %>-check-typekit-uptodate)

add_custom_target(<%= typekit.name %>-regen
    <% ruby_bin   = RbConfig::CONFIG['RUBY_INSTALL_NAME']
       typegen_bin = 'typegen' %>
   <%= typegen_bin %> <%= RTT_CPP.command_line_options.join(" ") %>
   WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/<%= RTT_CPP.relative_generation_directory %>)

#if(NOT TARGET regen)
#    add_custom_target(regen)
#endif()
#add_dependencies(regen <%= typekit.name %>-regen)

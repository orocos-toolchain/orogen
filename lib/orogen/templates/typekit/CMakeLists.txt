# Generated from orogen/lib/orogen/templates/typekit/CMakeLists.txt

##
# Header for typegen-generated typekits
#
<% if typekit.standalone? %>
project("<%= typekit.name %>-typekit")
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")

# In Orogen components, the build target is specified at generation time
# But we allow to override in case of typegen/USE_OROCOS_RTT:
if (NOT OROCOS_TARGET)
  set(OROCOS_TARGET "<%= typekit.orocos_target %>")
endif()
# If UseOrocos-RTT.cmake is used, install libs in /target/package subdir in order to allow
# multi-target/package installs.
if ( USE_OROCOS_RTT )
  if ( NOT OROCOS_SUFFIX )
    set (OROCOS_SUFFIX "/${OROCOS_TARGET}")
  endif()
  set(OROCOS_PKG_DIR "${OROCOS_SUFFIX}/<%= typekit.name %>")
endif()

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif (CXX_SUPPORTS_WALL)

# Define the necessary RPath information for all binaries. That can be stripped
# later
option(WITH_RPATH "enables or disables embedding RPath information in binaries" ON)
if(WITH_RPATH)
    include(RPATHHandling)
    CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/orocos:${CMAKE_INSTALL_PREFIX}/lib/orocos${OROCOS_PKG_DIR}/types")
endif(WITH_RPATH)

# Set the build type to debug by default
IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug )
ENDIF ( NOT CMAKE_BUILD_TYPE )

include(OrogenPkgCheckModules)
orogen_pkg_check_modules(OrocosRTT REQUIRED "orocos-rtt-${OROCOS_TARGET}>=1.99.4")

ADD_CUSTOM_TARGET(regen
    <% ruby_bin   = RbConfig::CONFIG['RUBY_INSTALL_NAME']
       typegen_bin = 'typegen' %>
   <%= typegen_bin %> <%= RTT_CPP.command_line_options.join(" ") %>
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/<%= RTT_CPP.relative_generation_directory %>)

<% end %>
# END if typekit.standalone?
##

# Manually define '-DIS_ROS_PACKAGE' or include the UseOrocos.cmake file to automatically detect&set this flag.
if ( IS_ROS_PACKAGE )
    # When we import a typekit in a ROS package, we expect it to be in the package/lib directory
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX}/types)
endif( IS_ROS_PACKAGE ) 

execute_process(COMMAND cmake -E create_symlink
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>)

include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/types)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OrocosRTT_INCLUDE_DIRS})
add_definitions(${OrocosRTT_CFLAGS_OTHER})
link_directories(${OrocosRTT_LIBRARY_DIRS})

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
set(libname <%= typekit.name %>-typekit-${OROCOS_TARGET})
add_library(${libname} SHARED
    <%= relatives = []
        implementation_files.each do |file| 
	    relatives << typekit.cmake_relative_path(file)
        end
        relatives.sort.join("\n    ") %>
    ${TYPEKIT_ADDITIONAL_SOURCES}
    ${TOOLKIT_ADDITIONAL_SOURCES})

<%= Generation.cmake_pkgconfig_link_noncorba('${libname}', typekit_deps) %>
target_link_libraries(${libname} ${OrocosRTT_LIBRARIES} ${TYPEKIT_ADDITIONAL_LIBRARIES} ${TOOLKIT_ADDITIONAL_LIBRARIES})
set_target_properties(${libname} PROPERTIES LINK_INTERFACE_LIBRARIES ${OrocosRTT_LIBRARIES})
set_target_properties(${libname} PROPERTIES INTERFACE_LINK_LIBRARIES ${OrocosRTT_LIBRARIES})
if(WITH_RPATH AND APPLE)
  set_target_properties( ${libname} PROPERTIES
    INSTALL_NAME_DIR "@rpath"
    LINK_FLAGS "-Wl,-rpath ${CMAKE_INSTALL_PREFIX}/lib -rpath ${CMAKE_INSTALL_PREFIX}/lib/orocos -rpath ${CMAKE_INSTALL_PREFIX}/lib/orocos${OROCOS_PKG_DIR}/types")
endif()
set(PKG_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-typekit-${OROCOS_TARGET}.pc)
configure_file(<%= typekit.name %>-typekit.pc.in ${PKG_CONFIG_FILE} @ONLY)

install(TARGETS ${libname} LIBRARY DESTINATION lib/orocos${OROCOS_PKG_DIR}/types)
install(FILES Types.hpp Opaques.hpp
    DESTINATION include/orocos/<%= typekit.name %>)
install(FILES
    <% if typekit.has_opaques_with_templates? %>
    ${PROJECT_SOURCE_DIR}/typekit/Opaques.hpp
    <% end %>
    <%= relatives = []
        plugin_header_files.each do |file|
	    relatives << typekit.cmake_relative_path(file)
	end
	relatives.sort.join("\n    ") %>
    DESTINATION include/orocos/<%= typekit.name %>/typekit)

<% (public_header_files + typekit.included_files).each do |inc|
    full_path = Pathname.new(typekit.automatic_dir).join('types', inc)
    next if !full_path.exist?
    target_dir = File.dirname(inc)
    if full_path.symlink?
       full_path = full_path.readlink
    end
    source_file = full_path.relative_path_from(Pathname.new(typekit.automatic_dir)) %>
install(FILES <%= source_file.to_path %>
    DESTINATION include/orocos/<%= target_dir %>)
<% end %>
install(FILES ${PKG_CONFIG_FILE}
    DESTINATION lib/pkgconfig)
install(FILES <%= typekit.name %>.tlb
    <%= typekit.name %>.typelist
    DESTINATION share/orogen)

<% typekit.each_plugin do |plg|
    if plg.separate_cmake? %>
    add_subdirectory(transports/<%= plg.name %>)
    <% end
end %>

# Force the user to regenerate its typekit if the inputs changed
set(TK_STAMP "${CMAKE_CURRENT_SOURCE_DIR}/stamp")
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
    COMMENT "Typekit input changed. Run make regen in your build directory first"
    COMMAND /bin/false)
add_custom_target(check-typekit-uptodate ALL DEPENDS "${TK_STAMP}")
add_dependencies(${libname} check-typekit-uptodate)


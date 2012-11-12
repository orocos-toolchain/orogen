# Generated from orogen/lib/orogen/templates/typekit/CMakeLists.txt

##
# Header for typegen-generated typekits
#
<% if typekit.standalone? %>
project("<%= typekit.name %>-typekit")
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")

# In Orogen components, the build target is specified at generation time
set(OROCOS_TARGET "<%= typekit.orocos_target %>")
# By default, install libs in /target/ subdir in order to allow
# multi-target installs.
if ( NOT OROCOS_SUFFIX )
    set (OROCOS_SUFFIX "/${OROCOS_TARGET}")
endif()

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall", CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif (CXX_SUPPORTS_WALL)

# Define the necessary RPath information for all binaries. That can be stripped
# later
option(WITH_RPATH "enables or disables embedding RPath information in binaries" ON)
if(WITH_RPATH)
    include(RPATHHandling)
    CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/orocos")
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
    <%= typegen_bin %> <%= Orocos::Generation.command_line_options.join(" ") %>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/<%= Orocos::Generation.relative_generation_directory %>)

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
include_directories(${CMAKE_SOURCE_DIR}/typekit)
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
target_link_libraries(${libname} LINK_INTERFACE_LIBRARIES ${OrocosRTT_LIBRARIES})

set(PKG_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-typekit-${OROCOS_TARGET}.pc)
configure_file(<%= typekit.name %>-typekit.pc.in ${PKG_CONFIG_FILE} @ONLY)

install(TARGETS ${libname} LIBRARY DESTINATION lib/orocos/types)
install(FILES
    Types.hpp
    <% if typekit.has_opaques_with_templates? %>
    Opaques.hpp
    <% end %>
    DESTINATION include/orocos/<%= typekit.name %>)
install(FILES
    <%= relatives = []
        public_header_files.each do |file|
	    relatives << typekit.cmake_relative_path(file)
	end
	relatives.sort.join("\n    ") %>
    DESTINATION include/orocos/<%= typekit.name %>/typekit)
<% typekit.local_headers(false).each do |path, dest_path| %>

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/<%= (Pathname.new( typekit.base_dir ) + path).relative_path_from(Pathname.new(typekit.automatic_dir)) %>
    DESTINATION include/orocos/<%= typekit.name %>/types/<%= typekit.name %>/<%= File.dirname(dest_path) %>)
<% end %>
install(FILES ${PKG_CONFIG_FILE}
    DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>.tlb
    ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>.typelist
    DESTINATION share/orogen)

<% typekit.each_plugin do |plg|
    if plg.separate_cmake? %>
    add_subdirectory(transports/<%= plg.name %>)
    <% end
end %>

# Force the user to regenerate its typekit if the inputs changed
set(TK_STAMP "${CMAKE_CURRENT_SOURCE_DIR}/stamp")
add_custom_command(
    OUTPUT "${TK_STAMP}"
    DEPENDS <%= loads = []
        typekit.loads.each do |file| 
	    loads << typekit.cmake_relative_path(file)
        end
        loads.join(" ") %> <%= extloads = []
        typekit.external_loads.each do |file| 
	    extloads << typekit.cmake_relative_path(file)
        end
        extloads.join(" ") %>
    COMMENT "Typekit input changed. Run make regen in your build directory first"
    COMMAND /bin/false)
add_custom_target(check-typekit-uptodate ALL DEPENDS "${TK_STAMP}")
add_dependencies(${libname} check-typekit-uptodate)


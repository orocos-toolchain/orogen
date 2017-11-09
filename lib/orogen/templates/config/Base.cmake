include(OroGenTools)
ADD_CUSTOM_TARGET(regen
    <% ruby_bin   = RbConfig::CONFIG['RUBY_INSTALL_NAME'] %>
    <%= ruby_bin %> -S orogen <%= RTT_CPP.command_line_options.join(" ") %> <%= project.deffile %>
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_custom_command(
    OUTPUT ${PROJECT_SOURCE_DIR}/<%= RTT_CPP::AUTOMATIC_AREA_NAME %>/<%= File.basename(project.deffile) %>
    DEPENDS <%= project.deffile %>
    COMMENT "oroGen specification file changed. Run make regen first."
    COMMAND /bin/false)

option(DISABLE_REGEN_CHECK "disable the check that verifies whether regen should be run" OFF)

if (NOT DISABLE_REGEN_CHECK)
<% if File.file?(project.deffile) %>
add_custom_target(check-uptodate ALL
    DEPENDS "${PROJECT_SOURCE_DIR}/<%= RTT_CPP::AUTOMATIC_AREA_NAME %>/<%= File.basename(project.deffile) %>")
<% else %>
add_custom_target(check-uptodate ALL)
<% end %>
endif()

# In Orogen project, the build target is specified at generation time
set(OROCOS_TARGET "<%= project.orocos_target %>")

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)

# Do not enable Wall on WIN32 -- it's really ALL warnings. However, set the
# _CRT_DISABLE_SECURE_WARNINGS which is currently VERY verbose on RTT, boost and
# others
if (WIN32)
    add_definitions("-D_CRT_SECURE_NO_WARNINGS")
else()
    CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
    if (CXX_SUPPORTS_WALL)
        add_definitions ("-Wall")
    endif()
endif()
CHECK_CXX_COMPILER_FLAG("-Wno-unused-local-typedefs" CXX_SUPPORTS_WUNUSED_LOCAL_TYPEDEFS)
if (CXX_SUPPORTS_WUNUSED_LOCAL_TYPEDEFS)
    add_definitions ("-Wno-unused-local-typedefs")
endif()
check_cxx_compiler_flag("-Wunused-variable" CXX_SUPPORTS_WUNUSED_VARIABLE)

if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    message(STATUS "running on Linux, implementing the __orogen_getTID() operation on all tasks")
    add_definitions(-DHAS_GETTID)
else()
    message(STATUS "NOT running on Linux (cmake reports ${CMAKE_SYSTEM_NAME}). The __orogen_getTID() operation will be a dummy")
endif()

# Define the necessary RPath information for all binaries. That can be stripped
# later
option(WITH_RPATH "enables or disables embedding RPath information in binaries" ON)
if(WITH_RPATH)
    include(RPATHHandling)
    CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/orocos:${CMAKE_INSTALL_PREFIX}/lib/orocos/types")
endif(WITH_RPATH)

# Set the build type to debug by default
IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug )
ENDIF ( NOT CMAKE_BUILD_TYPE )

# Check for Doxygen and enable documentation building if available
FIND_PACKAGE( Doxygen )
IF ( DOXYGEN_FOUND )
  MESSAGE(STATUS "documentation can be built" )
  CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile @ONLY)
  ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
ELSE ( DOXYGEN_FOUND )
  MESSAGE(STATUS "documentation generation disabled" )
ENDIF ( DOXYGEN_FOUND )

#
# Start setting up the build itself
#

# First, we need the Orocos::RTT, and optionally the CORBA part
orogen_pkg_check_modules(OrocosRTT REQUIRED "orocos-rtt-${OROCOS_TARGET}>=2.1.0")

# Add generic include directories
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/)

<% if project.typekit %>
# Take care of the typekit
ADD_SUBDIRECTORY( ${CMAKE_CURRENT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/typekit )
if (NOT DISABLE_REGEN_CHECK)
add_dependencies(check-uptodate check-typekit-uptodate)
endif()
<% end %>

# Take care of the task library
<% if !project.self_tasks.empty? %>
ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/tasks)
<% end %>

<% additional_plugin_source_dirs.each do  |dir| %>
ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/<%= dir %>)
<% end %>

configure_file(<%= Generation::AUTOMATIC_AREA_NAME %>/orogen-project-<%= project.name %>.pc.in
    orogen-project-<%= project.name %>.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/orogen-project-<%= project.name %>.pc
    DESTINATION lib/pkgconfig)

INSTALL(FILES ${PROJECT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/<%= File.basename(project.deffile) %>
    DESTINATION share/orogen)

# Finally, add deployment code
<% project.deployers.each do |deployer| %>
include(<%= Generation::AUTOMATIC_AREA_NAME %>/config/<%= deployer.name %>Deployment.cmake)
<% end %>

# Install typelib and/or Roby plugins
if (EXISTS ${PROJECT_SOURCE_DIR}/scripts/typelib.rb)
    install(FILES ${PROJECT_SOURCE_DIR}/scripts/typelib.rb
        DESTINATION share/typelib/ruby
        RENAME <%= project.name %>.rb)
endif()
if (EXISTS ${PROJECT_SOURCE_DIR}/scripts/roby.rb)
    install(FILES ${PROJECT_SOURCE_DIR}/scripts/roby.rb
        DESTINATION share/orocos/roby
        RENAME <%= project.name %>.rb)
endif()


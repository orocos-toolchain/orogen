INCLUDE(FindPkgConfig) # This is the Cmake 2.6 FindPkgConfig macro
SET (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wall)

INCLUDE(RPATHHandling)
CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib")

IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release )
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

INCLUDE(RPATHHandling)
CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/orocos")

#
# Check for Doxygen and enable documentation building
#
FIND_PACKAGE( Doxygen )
IF ( DOXYGEN )
  MESSAGE( "Found Doxygen -- documentation can be built" )

ELSE ( DOXYGEN )
  MESSAGE( "Doxygen not found -- unable to build documentation" )
ENDIF ( DOXYGEN )

INCLUDE(DependentOption)
DEPENDENT_OPTION( DOC_GENERATE_API "Build API Documentation" OFF "DOXYGEN" OFF )


#
# An option for tests, to make it easy to turn off all tests
#
OPTION( BUILD_TESTS "Turn me off to disable compilation of all tests" OFF )

FIND_PACKAGE(OrocosRTT REQUIRED)
SET(OROCOS_COMPONENT_INCLUDE ${OROCOS_RTT_INCLUDE_DIRS})

<% if component.toolkit %>
pkg_check_modules(TYPELIB REQUIRED typelib)
    <% if component.toolkit.corba_enabled? %>
FIND_PACKAGE(OrocosCORBA REQUIRED COMPONENTS Toolkit)
    <% end %>
<% end %>

<% if component.corba_enabled? %>
    <% if !component.toolkit || !component.toolkit.corba_enabled? %>
FIND_PACKAGE(OrocosCORBA REQUIRED)
    <% end %>
<% end %>

SET(OROCOS_COMPONENT_LIBRARIES ${OROCOS_RTT_LIBS})

IF(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/toolkit)
    ADD_SUBDIRECTORY( ${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/toolkit )
    INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/toolkit")
    SET(OROCOS_COMPONENT_LIBRARIES ${OROCOS_COMPONENT_LIBRARIES} ${CMAKE_PROJECT_NAME}-toolkit-${OROCOS_TARGET})
ENDIF(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/toolkit)

INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>)
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR})

<% component.used_toolkits.each do |name, _| %>
pkg_check_modules(<%= name %>_TOOLKIT REQUIRED <%= name %>-toolkit-${OROCOS_TARGET})
INCLUDE_DIRECTORIES(${<%= name %>_TOOLKIT_INCLUDE_DIRS})
LINK_DIRECTORIES(${<%= name %>_TOOLKIT_LIBRARY_DIRS})
list(APPEND DEPENDENCIES_LDFLAGS ${<%= name %>_TOOLKIT_LDFLAGS})
<% end %>

<% component.used_libraries.each do |pkg|
name = pkg.name %>
pkg_check_modules(<%= name %> REQUIRED <%= name %>)
INCLUDE_DIRECTORIES(${<%= name %>_INCLUDE_DIRS})
LINK_DIRECTORIES(${<%= name %>_LIBRARY_DIRS})
list(APPEND DEPENDENCIES_LDFLAGS ${<%= name %>_LDFLAGS})
<% end %>

<% component.used_task_libraries.each do |pkg|
name = pkg.name %>
pkg_check_modules(<%= name %>_TASKLIB REQUIRED <%= name %>-tasks-${OROCOS_TARGET})
INCLUDE_DIRECTORIES(${<%= name %>_TASKLIB_INCLUDE_DIRS})
LINK_DIRECTORIES(${<%= name %>_TASKLIB_LIBRARY_DIRS})
list(APPEND DEPENDENCIES_LDFLAGS ${<%= name %>_TASKLIB_LDFLAGS})
<% end %>

MESSAGE(STATUS "External dependencies libraries: ${DEPENDENCIES_LDFLAGS}")

MACRO(TARGET_APPEND_LDFLAGS TARGET FLAGS)
    GET_TARGET_PROPERTY(CURRENT_LDFLAGS <%= component.name %> LINK_FLAGS)
    IF(NOT CURRENT_LDFLAGS)
        SET(CURRENT_LDFLAGS ${FLAGS})
    ELSE(NOT CURRENT_LDFLAGS)
        list(APPEND CURRENT_LDFLAGS ${FLAGS})
    ENDIF(NOT CURRENT_LDFLAGS)

    IF(CURRENT_LDFLAGS)
        STRING(REGEX REPLACE ";" " " CURRENT_LDFLAGS "${CURRENT_LDFLAGS}")
        SET_TARGET_PROPERTIES(${TARGET} PROPERTIES LINK_FLAGS "${CURRENT_LDFLAGS}")
    ENDIF(CURRENT_LDFLAGS)
ENDMACRO(TARGET_APPEND_LDFLAGS)

<% if !component.self_tasks.empty? %>
ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/tasks)
ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/tasks)
<% end %>

INCLUDE_DIRECTORIES(${OrocosCORBA_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OrocosCORBA_DEFINES})

ADD_CUSTOM_TARGET(regen
    orogen <%= "--corba" if component.corba_enabled? %> <%= component.deffile %>
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

<% component.deployers.each do |deploy| %>
    <%= deploy.cmake_code %>
<% end %>


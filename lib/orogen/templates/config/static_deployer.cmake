#
#
# WARNING: BIG FAT WARNING !!!! BIG FAT WARNING !!! BIG FAT WARNING
# WARNING: this file is NOT used by CMake. It gets included in the OrocosComponent.cmake
# WARNING: configuration file for deployments that use Corba, but the file in itself
# WARNING: is NOT used in the finally generated module (even though it is present there)
#
#

<% if deployer.browse %>
pkg_check_modules(OrocosOCL REQUIRED orocos-ocl-${OROCOS_TARGET})
INCLUDE_DIRECTORIES(${OrocosOCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${OrocosOCL_LIBRARY_DIRS})
<% end %>

ADD_EXECUTABLE(<%= component.name %> ${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/main.cpp)
TARGET_LINK_LIBRARIES(<%= component.name %> ${OROCOS_COMPONENT_LIBRARIES})

<% component.used_toolkits.each do |name, _| %>
TARGET_LINK_LIBRARIES(<%= component.name %> ${<%= name %>_TOOLKIT_LIBS})
<% end %>
<% if !component.tasks.empty? %>
TARGET_LINK_LIBRARIES(<%= component.name %> <%= component.name %>-tasks-${OROCOS_TARGET})
<% end %>
INSTALL(TARGETS <%= component.name %>
    RUNTIME DESTINATION bin)

TARGET_APPEND_LDFLAGS(<%= component.name %> "${DEPENDENCIES_LDFLAGS}")
<% if component.corba_enabled? %>
TARGET_LINK_LIBRARIES(<%= component.name %> ${OrocosCORBA_LIBRARIES})
<% end %>

<% if deployer.browse %>
TARGET_APPEND_LDFLAGS(<%= component.name %> "${OrocosOCL_LDFLAGS}")
<% end %>

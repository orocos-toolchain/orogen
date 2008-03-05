PROJECT(<%= component.name %>)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/config")
INCLUDE(OrocosComponent)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)


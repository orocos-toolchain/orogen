IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release )
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

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
FIND_PACKAGE(OrocosCORBA REQUIRED COMPONENTS Toolkit)

SET(OROCOS_COMPONENT_INCLUDE ${OROCOS_RTT_INCLUDE_DIRS} ${OCL_INCLUDE_DIR})
SET(OROCOS_COMPONENT_LIBRARIES ${OCL_LIBS} ${OROCOS_RTT_LIBS} )

IF(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/.orocos/toolkit)
    ADD_SUBDIRECTORY( ${CMAKE_SOURCE_DIR}/.orocos/toolkit )
    INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/.orocos/toolkit")
    LINK_DIRECTORIES("${CMAKE_BINARY_DIR}/.orocos/toolkit")
    SET(OROCOS_COMPONENT_LIBRARIES ${OROCOS_COMPONENT_LIBRARIES} ${CMAKE_PROJECT_NAME}-toolkit-${OROCOS_TARGET})
ENDIF(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/.orocos/toolkit)



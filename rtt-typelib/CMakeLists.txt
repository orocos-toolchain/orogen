cmake_minimum_required(VERSION 2.6)
project(rtt_typelib)

include(FindPkgConfig)

pkg_check_modules(OrocosRTT REQUIRED orocos-rtt-$ENV{OROCOS_TARGET})
include_directories(${OrocosRTT_INCLUDE_DIRS})
link_directories(${OrocosRTT_LIBRARY_DIRS})
add_definitions(${OrocosRTT_CFLAGS})

pkg_check_modules(OrocosRTT_MQUEUE orocos-rtt-mqueue-$ENV{OROCOS_TARGET})
if (OrocosRTT_MQUEUE_FOUND)
    message(STATUS "MQ transport found, MQ typelib marshaller enabled")
    include_directories(${OrocosRTT_MQUEUE_INCLUDE_DIRS})
    link_directories(${OrocosRTT_MQUEUE_LIBRARY_DIRS})
    add_definitions(${OrocosRTT_MQUEUE_CFLAGS})
    set(mqsupport_sources MQTypelibMarshaller.cpp)
else()
    message(STATUS "MQ transport not found, MQ typelib marshaller disabled")
endif()

pkg_check_modules(Typelib REQUIRED typelib)
include_directories(${Typelib_INCLUDE_DIRS})
link_directories(${Typelib_LIBRARY_DIRS})
add_definitions(${Typelib_CFLAGS})

add_library(rtt-typelib SHARED
    TypelibMarshallerBase.cpp ${mqsupport_sources})

target_link_libraries(rtt-typelib
    ${OrocosRTT_LIBRARIES}
    ${Typelib_LIBRARIES})
if (OrocosRTT_MQUEUE_FOUND)
    target_link_libraries(rtt-typelib
        ${OrocosRTT_MQUEUE_LIBRARIES})
endif()

install(TARGETS rtt-typelib
    LIBRARY DESTINATION lib)
install(FILES TypelibMarshallerBase.hpp TypelibMarshaller.hpp
    TypelibMarshallerHandle.hpp
    MQTypelibMarshaller.hpp OpaqueTypelibMarshaller.hpp
    DESTINATION include/rtt/typelib)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rtt-typelib.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/rtt-typelib.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rtt-typelib.pc
    DESTINATION lib/pkgconfig)


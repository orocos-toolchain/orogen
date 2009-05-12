include(<%= component.name %>TaskLib)
ADD_LIBRARY(${<%= component.name.upcase %>_TASKLIB_NAME} SHARED 
    ${<%= component.name.upcase %>_TASKLIB_SOURCES})

TARGET_LINK_LIBRARIES(${<%= component.name.upcase %>_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${<%= component.name.upcase %>_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${<%= component.name.upcase %>_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${<%= component.name.upcase %>_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${<%= component.name.upcase %>_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${<%= component.name.upcase %>_TASKLIB_HEADERS}
    DESTINATION include/orocos/<%= component.name.downcase %>)


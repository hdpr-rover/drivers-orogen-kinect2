# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(kinect2TaskLib)
ADD_LIBRARY(${KINECT2_TASKLIB_NAME} SHARED 
    ${KINECT2_TASKLIB_SOURCES})
add_dependencies(${KINECT2_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${KINECT2_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${KINECT2_TASKLIB_DEPENDENT_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libglut.so
    )
SET_TARGET_PROPERTIES(${KINECT2_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${KINECT2_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${KINECT2_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${KINECT2_TASKLIB_HEADERS}
    DESTINATION include/orocos/kinect2)


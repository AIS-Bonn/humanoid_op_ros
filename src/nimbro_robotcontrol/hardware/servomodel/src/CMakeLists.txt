
add_library(servomodel
	servocommandgenerator.cpp
	#torqueestimator.cpp
	${MOC_FILES}
)

target_link_libraries(servomodel
	${catkin_LIBRARIES}
)

install(TARGETS servomodel
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if(SERVOMODEL_TESTBENCH)
	add_subdirectory(testbench)
endif()

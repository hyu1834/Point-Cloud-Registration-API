SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ADD_EXECUTABLE(distance_geometry_registration example_distance_geometry_registration.cpp)
TARGET_LINK_LIBRARIES(distance_geometry_registration
					  ${PCL_LIBRARIES}
					  registration_static
					 )
					 

ADD_EXECUTABLE(bisecting_linear_line_complex_registration example_bisecting_linear_line_complex_registration.cpp)
TARGET_LINK_LIBRARIES(bisecting_linear_line_complex_registration
					  ${PCL_LIBRARIES}
					  registration_static
					 )
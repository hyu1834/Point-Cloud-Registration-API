FILE(GLOB SOURCES "src/*.cpp")

# ADD_LIBRARY(registration_shared SHARED ${SOURCES})
ADD_LIBRARY(registration_static STATIC ${SOURCES})

# TARGET_INCLUDE_DIRECTORIES(registration_shared PUBLIC include)
TARGET_INCLUDE_DIRECTORIES(registration_static PUBLIC include)

# TARGET_LINK_LIBRARIES(registration_shared
# 					  ${PCL_LIBRARIES}
# 					 )

TARGET_LINK_LIBRARIES(registration_static
					  ${PCL_LIBRARIES}
					 )
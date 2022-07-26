cmake_minimum_required(VERSION 3.1.0)
project(client)

set(CMAKE_CXX_STANDARD 11)

# Enable C++11 for cmake older than 3.1, but for * sake just upgrade your cmake, even ubuntu 16.04 is supporting cmake 3.5
#add_compile_options(-std=c++11)

#set(SDK_PATH "./sdk/")
#
#FILE(GLOB SDK_SRC
#	"${SDK_PATH}/src/impl/unix/*.cpp"
#	"${SDK_PATH}/src/*.cpp"
#	"${SDK_PATH}/src/Sockets/*.cpp"
#)



find_package(Boost COMPONENTS system filesystem REQUIRED)
link_directories(/usr/local/lib/)

include_directories(
	# The path of the folders holding headers, not for source file
	include
	#${PROJECT_SOURCE_DIR}/sdk/include
	#${PROJECT_SOURCE_DIR}/sdk/src
	#${PROJECT_SOURCE_DIR}/sdk/src/Sockets
	#${PROJECT_SOURCE_DIR}/sdk/src/impl/unix
	${PROJECT_SOURCE_DIR}/rosbridge/rosbridgecpp
)

add_subdirectory(${PROJECT_SOURCE_DIR}/rosbridge/rosbridgecpp rosbridgecpp)

#add_library(sdk_lib SHARED ${SDK_SRC})
#target_link_libraries(sdk_lib rt pthread)
#
#add_library(common_lib
#	src/common/errhandle.cpp
#	src/common/tcpprotocol.cpp
#)


# Tray module (sim version)
add_executable(rosbridge_test
	src/client.cpp
)
target_link_libraries(rosbridge_test
	#common_lib
	#pthread
	rosbridgecpp
	#${Boost_FILESYSTEM_LIBRARY}
	#${Boost_SYSTEM_LIBRARY}
)

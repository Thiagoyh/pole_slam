cmake_minimum_required(VERSION 3.0.2)
project(pole_slam)

add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include(FindPkgConfig)

find_package(PCL 1.8 REQUIRED)
find_package(absl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Lua REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sphercial
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  "/usr/include/pcl-1.8"
  "/usr/include/eigen3"
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h" "include/common/*.cpp", "include/common/*.h")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cpp")
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

set(EXE_TEST test_)
set(EXE_RELEASE pole_slam)

#add_library(${EXE_TEST} STATIC ${ALL_SRCS})


add_executable(${EXE_TEST} src/pcd_test_main.cpp ${ALL_SRCS})
target_link_libraries(${EXE_TEST} ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            glog gflags)

target_include_directories(${EXE_TEST} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

target_include_directories(${EXE_TEST} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")

target_link_libraries(${EXE_TEST}  ${Boost_LIBRARIES})

target_link_libraries(${EXE_TEST} ${LUA_LIBRARIES})

target_include_directories(${EXE_TEST} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(${EXE_TEST} ${catkin_EXPORTED_TARGETS})


add_executable(${EXE_RELEASE} src/spherical_main.cpp ${ALL_SRCS})
target_link_libraries(${EXE_RELEASE} ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            glog gflags)

target_include_directories(${EXE_RELEASE} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

target_include_directories(${EXE_RELEASE} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")

target_link_libraries(${EXE_RELEASE}  ${Boost_LIBRARIES})

target_link_libraries(${EXE_RELEASE} ${LUA_LIBRARIES})

target_include_directories(${EXE_RELEASE} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(${EXE_RELEASE} ${catkin_EXPORTED_TARGETS})

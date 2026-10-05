cmake_minimum_required(VERSION 3.0.2)
project(dynamic_occpuancy_map)

add_definitions(-std=c++14 -g -O3 -ftree-vectorize -ffast-math -march=native -lpthread)
#add_definitions(-std=c++14)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  gazebo_msgs
  nav_msgs
  message_generation
  message_filters
  sensor_msgs #
  image_transport
)

add_message_files(
  FILES
  PredictedEllipse.msg
  PredictedEllipseArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visualization_msgs gazebo_msgs nav_msgs sensor_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )

add_executable(map_sim_example src/map_sim_example.cpp)
add_dependencies(map_sim_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_sim_example ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dsp_fov_grid test/test_dsp_fov_grid.cpp)
  if(TARGET test_dsp_fov_grid)
    target_link_libraries(test_dsp_fov_grid ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_dsp_probability_ellipse test/test_dsp_probability_ellipse.cpp)
  if(TARGET test_dsp_probability_ellipse)
    target_link_libraries(test_dsp_probability_ellipse ${catkin_LIBRARIES})
  endif()
endif()

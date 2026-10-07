cmake_minimum_required(VERSION 3.0.2)
project(coni_mpc)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf2_ros
        mavros_msgs
        geometry_msgs
        visualization_msgs
        trajectory_msgs
        sensor_msgs
        message_generation
        message_filters
        pcl_conversions)
find_package(gflags REQUIRED)
if(NOT ${gflags_FOUND})
        message(FATAL_ERROR "Could not find gflags\nPlease use `sudo apt-get install libgflags-dev' to install")
endif()
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree io)

add_message_files(
        FILES
        LocalBarrier.msg)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs)



if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
        #  LIBRARIES coni_mpc
        #  CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(depth_cbf_preprocessor
        STATIC
        src/depth_cbf/point_cloud_preprocessor.cpp)
target_link_libraries(depth_cbf_preprocessor
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        Eigen3::Eigen)

add_executable(marsim_cloud_preprocessor_node
        src/depth_cbf/marsim_cloud_preprocessor_node.cpp)
target_link_libraries(marsim_cloud_preprocessor_node
        depth_cbf_preprocessor
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        Eigen3::Eigen)

add_library(depth_cbf_regressor
        STATIC
        src/depth_cbf/depth_cbf_regressor.cpp)
target_link_libraries(depth_cbf_regressor
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        Eigen3::Eigen)

add_executable(depth_cbf_regression_node
        src/depth_cbf/depth_cbf_regression_node.cpp)
add_dependencies(depth_cbf_regression_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_cbf_regression_node
        depth_cbf_regressor
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        Eigen3::Eigen)

add_executable(depth_cbf_m25_trajectory_node
        src/depth_cbf/depth_cbf_m25_trajectory_node.cpp)
target_link_libraries(depth_cbf_m25_trajectory_node
        ${catkin_LIBRARIES}
        Eigen3::Eigen)

add_executable(depth_cbf_m25_map_publisher_node
        src/depth_cbf/depth_cbf_m25_map_publisher_node.cpp)
target_link_libraries(depth_cbf_m25_map_publisher_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(depth_cbf_m25_evaluation_node
        src/depth_cbf/depth_cbf_m25_evaluation_node.cpp)
target_link_libraries(depth_cbf_m25_evaluation_node
        depth_cbf_preprocessor
        depth_cbf_regressor
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        Eigen3::Eigen)

set(ACADO_CODEGEN_DIR ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen)
file(READ ${ACADO_CODEGEN_DIR}/acado_common.h ACADO_COMMON_HEADER)

set(acado_mpc_dep_src
        ${ACADO_CODEGEN_DIR}/acado_integrator.c
        ${ACADO_CODEGEN_DIR}/acado_solver.c
        ${ACADO_CODEGEN_DIR}/acado_auxiliary_functions.c)

if(ACADO_COMMON_HEADER MATCHES "#define ACADO_QP_SOLVER ACADO_QPOASES3")
        list(APPEND acado_mpc_dep_src
                ${ACADO_CODEGEN_DIR}/acado_qpoases3_interface.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Bounds.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Constraints.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Flipper.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Indexlist.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Matrices.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/MessageHandling.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Options.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/QProblem.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/QProblemB.c
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Utils.c)
        set(ACADO_QP_BACKEND_DEFS
                __CODE_GENERATION__
                QPOASES_CUSTOM_INTERFACE=acado_qpoases3_interface.h)
        set(ACADO_QP_BACKEND_INCLUDES
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases3/include)
else()
        list(APPEND acado_mpc_dep_src
                ${ACADO_CODEGEN_DIR}/acado_qpoases_interface.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Bounds.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Constraints.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/CyclingManager.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Indexlist.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/MessageHandling.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/QProblem.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/QProblemB.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/SubjectTo.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Utils.cpp
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp)
        set(ACADO_QP_BACKEND_DEFS
                QPOASES_CUSTOM_INTERFACE=acado_qpoases_interface.hpp)
        set(ACADO_QP_BACKEND_INCLUDES
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/INCLUDE
                ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC)
endif()

add_library(acado_mpc
        STATIC
        src/acado_mpc/mpc_controller.cpp
        src/acado_mpc/mpc_wrapper.cpp
        src/acado_mpc/mpc_common.cpp
        ${acado_mpc_dep_src})
# The generated ACADO RTI already owns its preparation/feedback cycle.  The
# outer field-profile refinement is reserved for the OsqpEigen backend, so
# downstream controller/simulator code can select the architecture without a
# runtime guess based on the target name.
target_compile_definitions(acado_mpc PUBLIC
        CONI_MPC_EXTERNAL_FIELD_MICRO_ITERATIONS=0)
target_compile_definitions(acado_mpc PRIVATE ${ACADO_QP_BACKEND_DEFS})
target_include_directories(acado_mpc PUBLIC
        ${ACADO_CODEGEN_DIR}
        ${ACADO_QP_BACKEND_INCLUDES})
target_link_libraries(acado_mpc
        ${catkin_LIBRARIES}
        Eigen3::Eigen)


add_library(NumericalCoNiMPC
        STATIC
        parameters/adaptive_trend_gain.cpp
        parameters/non_inertial_predictor.cpp
        src/coni_mpc/num_sim_mpc.cpp
        src/coni_mpc/mpc_base.cpp)
add_dependencies(NumericalCoNiMPC
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_include_directories(NumericalCoNiMPC PUBLIC
        ${PROJECT_SOURCE_DIR}/include/coni_mpc)
target_link_libraries(NumericalCoNiMPC
        acado_mpc
        ${catkin_LIBRARIES}
        Eigen3::Eigen)

add_library(PolytopicVoHrvoFilter
        STATIC
        src/num_sim/polytopic_vo_hrvo_filter.cpp)
target_include_directories(PolytopicVoHrvoFilter PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(PolytopicVoHrvoFilter
        Eigen3::Eigen)

add_executable(num_sim_non_one_point_node
        src/num_sim/num_sim_non_one_point_node.cpp)
target_link_libraries(num_sim_non_one_point_node
        NumericalCoNiMPC
        PolytopicVoHrvoFilter
        gflags
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        Eigen3::Eigen)

add_executable(adaptive_trend_gain_demo
        src/tools/adaptive_trend_gain_demo.cpp)
target_link_libraries(adaptive_trend_gain_demo
        NumericalCoNiMPC
        ${catkin_LIBRARIES}
        Eigen3::Eigen)

add_executable(non_inertial_predictor_demo
        src/tools/non_inertial_predictor_demo.cpp)
target_link_libraries(non_inertial_predictor_demo
        NumericalCoNiMPC
        ${catkin_LIBRARIES}
        Eigen3::Eigen)

add_executable(polytopic_vo_hrvo_smoke_test
        src/tools/polytopic_vo_hrvo_smoke_test.cpp)
target_link_libraries(polytopic_vo_hrvo_smoke_test
        PolytopicVoHrvoFilter
        Eigen3::Eigen)

add_executable(field_replay_tool test/field_replay_tool.cpp)
target_link_libraries(field_replay_tool Eigen3::Eigen)
add_executable(field_replay_event_tool test/field_replay_event_tool.cpp)
target_link_libraries(field_replay_event_tool Eigen3::Eigen)

if(CATKIN_ENABLE_TESTING)
        add_test(NAME polytopic_vo_hrvo_smoke_test
                 COMMAND polytopic_vo_hrvo_smoke_test)
        catkin_add_gtest(point_cloud_preprocessor_test
                test/point_cloud_preprocessor_test.cpp)
        if(TARGET point_cloud_preprocessor_test)
                target_link_libraries(point_cloud_preprocessor_test
                        depth_cbf_preprocessor
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
                        Eigen3::Eigen)
        endif()
        catkin_add_gtest(depth_cbf_regressor_test
                test/depth_cbf_regressor_test.cpp)
        if(TARGET depth_cbf_regressor_test)
                target_link_libraries(depth_cbf_regressor_test
                        depth_cbf_regressor
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
                        Eigen3::Eigen)
        endif()
        catkin_add_gtest(field_hocbf_test test/field_hocbf_test.cpp)
        catkin_add_gtest(voxel_online_data_pool_test test/voxel_online_data_pool_test.cpp)
        if(TARGET voxel_online_data_pool_test)
                target_link_libraries(voxel_online_data_pool_test gtest_main Eigen3::Eigen)
        endif()
        if(TARGET field_hocbf_test)
                target_link_libraries(field_hocbf_test gtest_main Eigen3::Eigen)
        endif()
        catkin_add_gtest(field_replay_diagnostics_test test/field_replay_diagnostics_test.cpp)
        if(TARGET field_replay_diagnostics_test)
                target_link_libraries(field_replay_diagnostics_test gtest_main Eigen3::Eigen)
        endif()
endif()

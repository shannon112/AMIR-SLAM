cmake_minimum_required(VERSION 2.8)
project(octomap_server)

add_compile_options(-std=c++11)
option(GFLAGS "Enable Google Flags." ON)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet
  message_generation
  cartographer_ros_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

add_message_files(
  DIRECTORY msg
  FILES PosePointCloud2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime message_generation
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

# GFlags.
if (GFLAGS)
  # Don't search with REQUIRED as we can continue without gflags.
  find_package(Gflags)
  if (GFLAGS_FOUND)
    message("-- Found Google Flags header in: ${GFLAGS_INCLUDE_DIRS}, "
      "in namespace: ${GFLAGS_NAMESPACE}")
    add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})
  else (GFLAGS_FOUND)
    message("-- Did not find Google Flags (gflags), Building without gflags "
      "- no tests or tools will be built!")
    update_cache_variable(GFLAGS OFF)
  endif (GFLAGS_FOUND)
else (GFLAGS)
  message("-- Google Flags disabled; no tests or tools will be built!")
  # Mark as advanced (remove from default GUI view) the gflags search
  # variables in case user enabled GFLAGS, FindGflags did not find it, so
  # made search variables visible in GUI for user to set, but then user disables
  # GFLAGS instead of setting them.
  mark_as_advanced(FORCE GFLAGS_INCLUDE_DIR
                         GFLAGS_LIBRARY
                         GFLAGS_NAMESPACE)
endif (GFLAGS)

if (GFLAGS)
  include_directories(${GFLAGS_INCLUDE_DIRS})
endif (GFLAGS)

# Change the default build type from Debug to Release, while still
# supporting overriding the build type.
#
# The CACHE STRING logic here and elsewhere is needed to force CMake
# to pay attention to the value of these variables.
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)

# ===================================================
# Octomap server
# ===================================================
#add_library(${PROJECT_NAME} src/OctomapServer.cpp src/OctomapServerMultilayer.cpp src/TrackingOctomapServer.cpp)
#target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# octomap_server_node
#add_executable(octomap_server_node src/octomap_server_node.cpp)
#target_link_libraries(octomap_server_node ${PROJECT_NAME} ${LINK_LIBS})

# octomap_server_static
#add_executable(octomap_server_static src/octomap_server_static.cpp)
#target_link_libraries(octomap_server_static ${PROJECT_NAME} ${LINK_LIBS})

# octomap_server_multilayer
#add_executable(octomap_server_multilayer src/octomap_server_multilayer.cpp)
#target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS})

# octomap_saver
#add_executable(octomap_saver src/octomap_saver.cpp)
#target_link_libraries(octomap_saver ${PROJECT_NAME} ${LINK_LIBS})

# octomap_tracking
#add_executable(octomap_tracking_server_node src/octomap_tracking_server_node.cpp)
#target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS})

# ===================================================
# Pointcloud  (local map optimizer)
# ===================================================
add_library(Submap3dServer src/Submap3dServer.cpp )
target_link_libraries(Submap3dServer ${LINK_LIBS})
add_dependencies(Submap3dServer ${PROJECT_NAME}_gencfg)
target_compile_definitions(Submap3dServer PUBLIC COLOR_OCTOMAP_SERVER)

# submap3d_server_node
add_executable(submap3d_server_node src/submap3d_server.cpp)
target_link_libraries(submap3d_server_node Submap3dServer ${LINK_LIBS})

# ===================================================
# Pointcloud  (local map visualizer)
# ===================================================
add_library(Submap3dVisualizer src/Submap3dVisualizer.cpp)
target_link_libraries(Submap3dVisualizer ${LINK_LIBS})
add_dependencies(Submap3dVisualizer ${PROJECT_NAME}_gencfg)
target_compile_definitions(Submap3dVisualizer PUBLIC COLOR_OCTOMAP_SERVER)

# submap3d_visualizer_node
add_executable(submap3d_visualizer_node src/submap3d_visualizer.cpp)
target_link_libraries(submap3d_visualizer_node Submap3dVisualizer ${LINK_LIBS})

# ===================================================
# Pointcloud  (global map optimizer)
# ===================================================
# pose graph lib
add_library(PoseGraph3d src/pose_graph_3d/pose_graph_3d.cc)
target_link_libraries(PoseGraph3d ${CERES_LIBRARIES} ${LINK_LIBS})

# pose graph test
add_executable(pose_graph_3d src/pose_graph_3d/pose_graph_3d_test.cc)
target_link_libraries(pose_graph_3d PoseGraph3d ${CERES_LIBRARIES} ${LINK_LIBS})

# submap3d_optimizer lib
add_library(Submap3dOptimizer src/Submap3dOptimizer.cpp)
target_link_libraries(Submap3dOptimizer PoseGraph3d ${LINK_LIBS})
add_dependencies(Submap3dOptimizer ${PROJECT_NAME}_gencfg)
target_compile_definitions(Submap3dOptimizer PUBLIC COLOR_OCTOMAP_SERVER)

# submap3d_optimizer_node
add_executable(submap3d_optimizer_node src/submap3d_optimizer.cpp)
target_link_libraries(submap3d_optimizer_node Submap3dOptimizer ${LINK_LIBS})

# ===================================================
# Octomap color server
# ===================================================
add_library(${PROJECT_NAME}_color src/OctomapServer.cpp src/OctomapServerMultilayer.cpp src/TrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME}_color ${LINK_LIBS})
add_dependencies(${PROJECT_NAME}_color ${PROJECT_NAME}_gencfg)
target_compile_definitions(${PROJECT_NAME}_color PUBLIC COLOR_OCTOMAP_SERVER)

# octomap_color_server_node
add_executable(octomap_color_server_node src/octomap_server_node.cpp)
target_link_libraries(octomap_color_server_node ${PROJECT_NAME}_color ${LINK_LIBS})


# ===================================================
# Nodelet
# ===================================================
# add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
# target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

# add_library(octomap_color_server_nodelet src/octomap_server_nodelet.cpp)
# target_link_libraries(octomap_color_server_nodelet ${PROJECT_NAME}_color ${LINK_LIBS})

# ===================================================
# install targets:
# ===================================================
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_color
#   octomap_server_node
#   octomap_color_server_node
#   octomap_server_static
#   octomap_server_multilayer
#   octomap_saver
#   octomap_tracking_server_node
#   octomap_server_nodelet
#   octomap_color_server_nodelet
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES 
#   nodelet_plugins.xml
#   color_nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

cmake_minimum_required(VERSION 3.8)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_models REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(loop_fusion_node
  src/pose_graph_node.cpp
  src/pose_graph.cpp
  src/keyframe.cpp
  src/utility/CameraPoseVisualization.cpp
  src/ThirdParty/DBoW/BowVector.cpp
  src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp
  src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp
  src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp
  src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp
)
target_compile_features(loop_fusion_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



ament_target_dependencies(loop_fusion_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  camera_models
)

target_link_libraries(loop_fusion_node ${OpenCV_LIBS} ${CERES_LIBRARIES} Eigen3::Eigen)

install(TARGETS loop_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ../support_files
  DESTINATION share)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

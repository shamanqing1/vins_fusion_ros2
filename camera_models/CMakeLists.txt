cmake_minimum_required(VERSION 3.8)
project(camera_models)

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

add_executable(Calibrations 
  src/intrinsic_calib.cc
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc)

target_include_directories(Calibrations PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Calibrations PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  Calibrations
  rclcpp
  std_msgs
)
target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Eigen3::Eigen)

install(TARGETS Calibrations
  DESTINATION lib/${PROJECT_NAME})

add_library(camera_models
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc)

target_include_directories(camera_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(camera_models PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(camera_models ${OpenCV_LIBS} ${CERES_LIBRARIES} Eigen3::Eigen)

ament_export_targets(camera_models HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS camera_models
  EXPORT camera_models
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


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

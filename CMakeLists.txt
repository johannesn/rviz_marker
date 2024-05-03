cmake_minimum_required(VERSION 3.8)
project(rviz_marker)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(marker_array src/marker_array.cpp)
ament_target_dependencies(marker_array rclcpp visualization_msgs)
target_include_directories(marker_array PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(marker_array PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(marker src/marker.cpp)
ament_target_dependencies(marker rclcpp visualization_msgs)
target_include_directories(marker PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(base_link src/base_link.cpp)
ament_target_dependencies(base_link rclcpp tf2 tf2_ros)
target_include_directories(base_link PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(base_link PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS marker_array marker base_link
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()

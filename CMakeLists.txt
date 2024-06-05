cmake_minimum_required(VERSION 3.8)
project(rt2_844_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)

# Tests
#if(BUILD_TESTING)
if(1)
  # tracetools is exported by rclcpp
  find_package(lifecycle_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  include_directories(
    include
    ${std_msgs_INCLUDE_DIRS}
  )

  add_executable(test_publisher
    src/test_publisher.cpp
  )
  target_link_libraries(test_publisher PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_static_exec
    src/test_static_exec.cpp
  )
  target_link_libraries(test_static_exec PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )


  add_executable(test_inter
    src/test_inter.cpp
  )
  target_link_libraries(test_inter PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_intra
    src/test_intra.cpp
  )
  target_link_libraries(test_intra PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_lifecycle_node
    src/test_lifecycle_node.cpp
  )
  target_link_libraries(test_lifecycle_node PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )

  add_executable(test_lifecycle_client
    src/test_lifecycle_client.cpp
  )
  target_link_libraries(test_lifecycle_client PRIVATE
    ${lifecycle_msgs_TARGETS}
    rclcpp::rclcpp
  )

  add_executable(test_ping
    src/test_ping.cpp
  )
  target_link_libraries(test_ping PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_pong
    src/test_pong.cpp
  )
  target_link_libraries(test_pong PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_timer
    src/test_timer.cpp
  )
  target_link_libraries(test_timer PRIVATE
    rclcpp::rclcpp
  )

  add_executable(test_service_ping
    src/test_service_ping.cpp
  )
  target_link_libraries(test_service_ping PRIVATE
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
  )

  add_executable(test_service_pong
    src/test_service_pong.cpp
  )
  target_link_libraries(test_service_pong PRIVATE
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
  )

  add_executable(test_intra_process
    src/test_intra_process.cpp
  )
  target_link_libraries(test_intra_process PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_ipb
    src/test_ipb.cpp
  )
  target_link_libraries(test_ipb PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_ipb2
    src/test_ipb2.cpp
  )
  target_link_libraries(test_ipb2 PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_ipb3
    src/test_ipb3.cpp
  )
  target_link_libraries(test_ipb3 PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_ipb4
    src/test_ipb4.cpp
  )
  target_link_libraries(test_ipb4 PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_generic_pubsub
    src/test_generic_pubsub.cpp
  )
  target_link_libraries(test_generic_pubsub PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_pub_nsub
    src/test_pub_nsub.cpp
  )
  target_link_libraries(test_pub_nsub PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_generic2
    src/test_generic2.cpp
  )
  target_link_libraries(test_generic2 PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_subscriber
    src/test_subscriber.cpp
  )
  target_link_libraries(test_subscriber PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_node_only
    src/test_node_only.cpp
  )
  target_link_libraries(test_node_only PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_mpub_nsub
    src/test_mpub_nsub.cpp
  )
  target_link_libraries(test_mpub_nsub PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_mpub_nsub_multi
    src/test_mpub_nsub_multi.cpp
  )
  target_link_libraries(test_mpub_nsub_multi PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_mpub_nsub_intra
    src/test_mpub_nsub_intra.cpp
  )
  target_link_libraries(test_mpub_nsub_intra PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_create_many_nodes
    src/test_create_many_nodes.cpp
  )
  target_link_libraries(test_create_many_nodes PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  install(TARGETS
    test_inter
    test_intra
    test_lifecycle_node
    test_lifecycle_client
    test_ping
    test_pong
    test_publisher
    test_static_exec
    test_service_ping
    test_service_pong
    test_timer
    test_intra_process
    test_ipb
    test_ipb2
    test_ipb3
    test_ipb4
    test_generic_pubsub
    test_pub_nsub
    test_generic2
    test_subscriber
    test_node_only
    test_mpub_nsub
    test_mpub_nsub_multi
    test_mpub_nsub_intra
    test_create_many_nodes
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  set(_test_tracetools_pytest_tests
    test/test_buffer.py
    test/test_executor.py
    test/test_intra.py
    test/test_intra_pub_sub.py
    test/test_lifecycle_node.py
    test/test_node.py
    test/test_pub_sub.py
    test/test_publisher.py
    test/test_service.py
    test/test_subscription.py
    test/test_timer.py
  )
  foreach(_test_path ${_test_tracetools_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

endif()

# Install
ament_auto_package(INSTALL_TO_SHARE
  launch
)
#ament_package()
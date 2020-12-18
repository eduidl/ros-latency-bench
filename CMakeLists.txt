cmake_minimum_required(VERSION 3.0.2)
project(latency_bench)

add_compile_options(
  -std=c++14
  -Wall
  -Wextra
  -Wpedantic
)

find_package(catkin REQUIRED
  roscpp
  message_generation
)

add_message_files(
  FILES
    Stamp.msg
    TwoStamps.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
)

foreach(BIN
  source_pub
  sub_callback
  timer_callback
  spin_once
  latency_stat
)
  add_executable(${BIN}
    src/${BIN}.cpp
  )

  target_link_libraries(${BIN}
    ${catkin_LIBRARIES}
  )

  target_include_directories(${BIN} PRIVATE
    ${catkin_INCLUDE_DIRS}
  )

  add_dependencies(${BIN}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
endforeach()

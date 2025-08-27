cmake_minimum_required(VERSION 3.16)
project(microstrain_mips)

# 必須パッケージ
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(diagnostic_updater REQUIRED)

# ----------------------
# メッセージ定義
# ----------------------
# rosidl_generate_interfaces(${PROJECT_NAME}
#   # "msg/StatusMsg.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

# include ディレクトリ
include_directories(
  include/${PROJECT_NAME}
  MIPSDK/C/Library/Include
  MIPSDK/C/Utilities/Include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# MIPSDK ソース
set(MIPSRC
  MIPSDK/C/Library/Source/mip.c
  MIPSDK/C/Library/Source/mip_sdk_base.c
  MIPSDK/C/Library/Source/mip_sdk_inteface.c
  MIPSDK/C/Library/Source/mip_sdk_3dm.c
  MIPSDK/C/Library/Source/mip_sdk_filter.c
  MIPSDK/C/Library/Source/mip_sdk_system.c
  MIPSDK/C/Library/Source/mip_sdk_ahrs.c
  MIPSDK/C/Library/Source/mip_sdk_gps.c
  MIPSDK/C/Library/Source/ring_buffer.c
  MIPSDK/C/Utilities/Source/byteswap_utilities.c
)

# ----------------------
# コアライブラリ
# ----------------------
add_library(microstrain_mips_core
  src/microstrain_3dm.cpp
  src/mip_sdk_user_functions.c
  ${MIPSRC}
)
ament_target_dependencies(microstrain_mips_core
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs
  # diagnostic_updater
  # rosidl_default_runtime  # メッセージ依存
)

# ----------------------
# Diagnostic updater ライブラリ
# ----------------------
# add_library(microstrain_diagnostic_updater src/microstrain_diagnostic_updater.cpp)
# ament_target_dependencies(microstrain_diagnostic_updater
#   rclcpp diagnostic_updater
#   rosidl_default_runtime
# )

# ----------------------
# ノード
# ----------------------
add_executable(microstrain_mips_node src/microstrain_3dm_node.cpp)
target_link_libraries(microstrain_mips_node
  microstrain_mips_core
  # microstrain_diagnostic_updater
)
ament_target_dependencies(microstrain_mips_node
  rclcpp tf2 tf2_ros
  std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs
  # diagnostic_updater
  rosidl_default_runtime
)

# ----------------------
# テスト実行ファイル
# ----------------------
add_executable(GX4_45_Test src/GX4-45_Test.c src/mip_sdk_user_functions.c ${MIPSRC})
target_link_libraries(GX4_45_Test rt)

# ----------------------
# インストール
# ----------------------
install(TARGETS
  microstrain_mips_core
  # microstrain_diagnostic_updater
  microstrain_mips_node
  DESTINATION lib/${PROJECT_NAME}
  # GX4_45_Test
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)

install(DIRECTORY launch config msg
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------
# export
# ----------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()

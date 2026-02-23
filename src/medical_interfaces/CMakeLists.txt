cmake_minimum_required(VERSION 3.8)
project(medical_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# メッセージファイルの登録
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VitalSigns.msg"
)

ament_package()

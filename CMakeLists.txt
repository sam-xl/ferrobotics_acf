
cmake_minimum_required(VERSION 3.5)

project(ferrobotics_acf)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
	"msg/ACFTelem.msg"
)

set(srv_files
	"srv/SetDuration.srv"
	"srv/SetFloat.srv"
)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	DEPENDENCIES builtin_interfaces
)

# install(PROGRAMS
#   scripts/joint_state_publisher.py
#   scripts/acf_interface.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
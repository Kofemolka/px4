px4_add_module(
	MODULE modules__navput_tunnel
	MAIN navput_tunnel
	SRCS
		NavputTunnel.cpp
		NavputTunnel.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		cdr
		uorb_msgs
		px4_work_queue
	INCLUDES
		${PX4_BINARY_DIR}/msg
)

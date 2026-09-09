px4_add_module(
	MODULE modules__navputer__spoofer
	MAIN spoofer
	SRCS
		gnss_spoof_gen.cpp
		gnss_spoof_gen.hpp
	DEPENDS
		px4_work_queue
)

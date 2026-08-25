px4_add_module(
	MODULE modules__navputer__gps_spoof_test
	MAIN gps_spoof_test
	SRCS
		gps_spoof_test.cpp
		gps_spoof_test.hpp
	DEPENDS
		px4_work_queue
)

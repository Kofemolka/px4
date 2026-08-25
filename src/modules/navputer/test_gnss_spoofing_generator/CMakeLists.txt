px4_add_module(
	MODULE modules__navputer__gnss_spoof_gen
	MAIN gnss_spoof_gen
	SRCS
		gnss_spoof_test.cpp
		gnss_spoof_test.hpp
	DEPENDS
		px4_work_queue
)

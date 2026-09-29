px4_add_unit_gtest(
	SRC test_gnss_analyzer.cpp
	EXTRA_SRCS
		../gnss_analyzer.cpp
		../gnss_kf.cpp
		${CMAKE_SOURCE_DIR}/platforms/posix/src/px4/common/test_stubs/stub_daemon.cpp
	COMPILE_FLAGS
		-DMODULE_NAME=\"navputer\"
	INCLUDES
		..
		${CMAKE_CURRENT_SOURCE_DIR}/../../../ekf2/EKF
		${CMAKE_CURRENT_SOURCE_DIR}/../../../ekf2/EKF/python
	LINKLIBS
		ecl_EKF
		px4_platform
		uORB
)

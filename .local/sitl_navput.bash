#!/bin/bash

set -e

build_tests=OFF

if [[ $# -gt 0 ]]; then
	if [[ "$1" == "--build-tests" ]]; then
		build_tests=ON

	else
		echo "Usage: $0 [--build-tests]" >&2
		exit 2
	fi
fi

export PX4_HOME_LAT=49.796766
export PX4_HOME_LON=24.347826
export PX4_HOME_ALT=270
export PX4_CMAKE_BUILD_TYPE=Debug

cd ..

rm -f build/px4_sitl_navput/rootfs/parameters.bson build/px4_sitl_navput/rootfs/parameters_backup.bson
rm -rf build/px4_sitl_navput/rootfs/zenoh

make px4_sitl_navput CMAKE_ARGS="-DCMAKE_TESTING=${build_tests}"

if [[ "${build_tests}" == ON ]]; then
	cmake --build build/px4_sitl_navput --target unit-test_gnss_analyzer
fi

cd build/px4_sitl_navput/rootfs
PX4_SIM_MODEL=navput_quadx PX4_SIMULATOR=sihsim ../bin/px4

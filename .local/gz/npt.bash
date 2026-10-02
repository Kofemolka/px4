cd ../..

make px4_sitl_navput

cd build/px4_sitl_navput/rootfs
PX4_SIM_MODEL=navput_gazebo ../bin/px4 -i 1

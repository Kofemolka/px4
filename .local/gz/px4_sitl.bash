cd ../..

make px4_sitl
cd build/px4_sitl_default/rootfs

PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=lawn PX4_GZ_NO_FOLLOW=1 ../bin/px4

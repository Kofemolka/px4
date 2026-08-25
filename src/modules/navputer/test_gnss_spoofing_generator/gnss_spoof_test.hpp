#pragma once

#include <cstdint>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>

class GnssSpoofTest final : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	GnssSpoofTest();
	~GnssSpoofTest() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class Mode : uint8_t
	{
		None,
		Velocity,
		Position,
	};

	void Run() override;
	void spoofVelocity(sensor_gps_s &gps);
	void spoofPosition(sensor_gps_s &gps);
	void setMode(Mode mode);

	uORB::Subscription _gps0_sub{ORB_ID(vehicle_gps_position), 0};
	uORB::PublicationMulti<sensor_gps_s> _gps1_pub{ORB_ID(vehicle_gps_position)};
	Mode _mode{Mode::None};
};

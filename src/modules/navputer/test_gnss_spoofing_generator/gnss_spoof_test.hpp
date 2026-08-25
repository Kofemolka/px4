#pragma once

#include <cstdint>
#include <lib/geo/geo.h>
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
		VelocityFixedRotation,
		VelocityFixedMagnitudeOffset,
		PositionFixedOffset,
		//VelocityRampRotation,
		//VelocityRampMagnitudeOffset,
		//PositionRampOffset,
	};

private:
	void Run() override;
	void maybeInitOrigin(sensor_gps_s& gps);
	void spoofVelocityByFixedRotation(sensor_gps_s &gps);
	void spoofVelocityByFixedMagnitudeOffset(sensor_gps_s &gps);
	void spoofPositionFixedOffset(sensor_gps_s &gps);
	void setMode(Mode mode);

private:
	MapProjection _origin_projection{};
	bool _origin_initialized{false};
	uORB::Subscription _gps0_sub{ORB_ID(vehicle_gps_position), 0};
	uORB::PublicationMulti<sensor_gps_s> _gps1_pub{ORB_ID(vehicle_gps_position)};
	uint64_t _elapsed_from_last_spoof_us{0};
	Mode _mode{Mode::None};
};

#pragma once

#include <cstdint>
#include <lib/geo/geo.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>

class GnssSpoofGen final : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	GnssSpoofGen();
	~GnssSpoofGen() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int maybeParseOffsetCommand(int argc, char *argv[], GnssSpoofGen& instance);
	static int maybeParseCarryOffCommand(int argc, char *argv[], GnssSpoofGen& instance);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class Mode : uint8_t
	{
		None,
		GradualOffset,
		GradualCarryOff,
	};

	struct SpoofPositionContext
	{
		uint64_t last_sample_time_us{0};
		uint64_t last_log_time_us{0};
		matrix::Vector3f target_ned{}; // m
		float max_speed{0.f}; // m/s
		float progress{0.f}; // [0, 1]
		sensor_gps_s start_sample;
		matrix::Vector3f start_ned;
	};

private:
	void Run() override;
	int print_status() override;

	void setMode(Mode mode);
	void maybeInitOrigin(sensor_gps_s& gps);

	void spoofGradualOffset(sensor_gps_s &gps);
	void spoofGradualCarryOff(sensor_gps_s &gps);

	void resetSpoofPositionContextNED(const matrix::Vector2f& tgt_offset_ned, const float max_speed);
	bool resetSpoofPositionContextGCS(const matrix::Vector2f& tgt_offset_ned, const float max_speed);

private:
	MapProjection _origin_projection{};
	bool _origin_initialized{false};
	uORB::Subscription _gps0_sub{ORB_ID(vehicle_gps_position), 0};
	uORB::PublicationMulti<sensor_gps_s> _gps1_pub{ORB_ID(vehicle_gps_position)};
	bool _gps1_advertised{false};
	uint64_t _elapsed_from_last_spoof_us{0};
	Mode _mode{Mode::None};

	SpoofPositionContext _pos_context;
};

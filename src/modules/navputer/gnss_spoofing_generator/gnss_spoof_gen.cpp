#include "gnss_spoof_gen.hpp"

#include <drivers/drv_hrt.h>

#include <cstring>
#include <charconv>
#include <math.h>

namespace
{
constexpr float kVelFixedRotationRad = M_PI_4; // PI/4 rad = 45 deg
constexpr float kVelFixedMagnitudeOffsetMS = 5.f; // m/s

constexpr float kDefaultOffset = 200.f; // m
constexpr float kDefaultOffsetMaxVelocity = 50.f; // m/s

constexpr double kDefaultCarryOffLatDeg = -12.0464; // gcs deg
constexpr double kDefaultCarryOffLonDeg = -77.0428; // gcs deg
constexpr float kDefaultCarryOffMaxVelocity = 5000.f; // m/s

constexpr float kMaxSlope = 1.875f;

bool parseFloat(const char* str, float& value)
{
        const char* str_end = str + strlen(str);
	auto [ptr, ec] = std::from_chars(str, str_end, value);
	return ec == std::errc{} && ptr == str_end && PX4_ISFINITE(value);
}
} // namespace

using namespace time_literals;

ModuleBase::Descriptor GnssSpoofGen::desc{task_spawn, custom_command, print_usage};

GnssSpoofGen::GnssSpoofGen() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool GnssSpoofGen::init()
{
	ScheduleOnInterval(10_ms);
	return true;
}

// h(u) = 10 * u^3 - 15 * u^4 + 6 * u^5
float rampQuintic(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const auto progress3 = progress * progress * progress;
	const auto progress4 = progress3 * progress;
	const auto progress5 = progress4 * progress;
	return 10.f * progress3 - 15.f * progress4 + 6.f * progress5;
}

// h`(u) = 30 * p^2 * (1 - u)^2
float rampQuinticDerivative(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const float progress2 = progress * progress;
	const float tmp = 1.f - progress;
	const float tmp2 = tmp * tmp;
	return 30.f * progress2 * tmp2;
}

// h(u) = u - 6u^3 + 8u^4 - 3u^5
float rampQuinticCarryOffInitial(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const auto progress3 = progress * progress * progress;
	const auto progress4 = progress3 * progress;
	const auto progress5 = progress4 * progress;
	return progress - 6.f * progress3 + 8.f * progress4 - 3.f * progress5;
}

// h`(u) = 1 - 18u^2 + 32u^3 - 15u^4
float rampQuinticDerivativeCarryOffInitial(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const auto progress2 = progress * progress;
	const auto progress3 = progress2 * progress;
	const auto progress4 = progress3 * progress;
	return 1.f - 18.f * progress2 + 32.f * progress3 - 15.f * progress4;
}

void GnssSpoofGen::spoofGradualOffset(sensor_gps_s &gps)
{
	if (!_origin_initialized)
	{
		return;
	}

	if (_pos_context.last_sample_time_us == 0)
	{
		PX4_INFO("offset started at sample=%" PRIu64, gps.timestamp_sample);
		_pos_context.last_sample_time_us = gps.timestamp_sample;
		return;
	}

	if (gps.timestamp_sample <= _pos_context.last_sample_time_us)
	{
		return;
	}

	const float dt = (gps.timestamp_sample - _pos_context.last_sample_time_us) / 1e6f;
	_pos_context.last_sample_time_us = gps.timestamp_sample;

	matrix::Vector2f spoofed_position_ne = _origin_projection.project(
		gps.latitude_deg,
		gps.longitude_deg);

	const float total_distance_m = _pos_context.target_ned.norm(); 

	if (total_distance_m <= 0 || _pos_context.max_speed <= 0)
	{
		return;
	}

	const float transition_duration_s = kMaxSlope * total_distance_m / _pos_context.max_speed;

	const auto elapsed_fraction = dt / transition_duration_s;
	_pos_context.progress = math::min(_pos_context.progress + elapsed_fraction, 1.f);

	const auto position_fraction = rampQuintic(_pos_context.progress);
	const auto position_rate_fraction = rampQuinticDerivative(_pos_context.progress);

	// offset = full_offset * ramp(progress)
	const matrix::Vector3f false_offset_position = _pos_context.target_ned * position_fraction;
	// offset_vel = full_offset * ramp_dt(progress) / duration
	const matrix::Vector3f false_offset_velocity = _pos_context.target_ned * position_rate_fraction / transition_duration_s;

	// spoofed pos
	spoofed_position_ne += matrix::Vector2f{false_offset_position(0), false_offset_position(1)};
	// spoofed alt
	gps.altitude_msl_m -= static_cast<double>(false_offset_position(2));
	gps.altitude_ellipsoid_m -= static_cast<double>(false_offset_position(2));
	// spoofed vel
	gps.vel_n_m_s += false_offset_velocity(0);
	gps.vel_e_m_s += false_offset_velocity(1);
	gps.vel_d_m_s += false_offset_velocity(2);
	gps.vel_m_s = sqrtf(gps.vel_n_m_s * gps.vel_n_m_s + gps.vel_e_m_s * gps.vel_e_m_s);
	gps.cog_rad = atan2f(gps.vel_e_m_s, gps.vel_n_m_s);

	double spoofed_lat;
	double spoofed_lon;

	_origin_projection.reproject(
		spoofed_position_ne(0),
		spoofed_position_ne(1),
		spoofed_lat,
		spoofed_lon);

	gps.latitude_deg = spoofed_lat;
	gps.longitude_deg = spoofed_lon;

	// debug info
	if (_pos_context.progress < 1.0f && hrt_elapsed_time(&_pos_context.last_log_time_us) > 1_s)
	{
		_pos_context.last_log_time_us = hrt_absolute_time();
		PX4_INFO(
			"offset tick: dt=%.3f progress=%.3f "
			"false_p=(%.1f,%.1f) false_v=(%.1f,%.1f)",
			(double)dt,
			(double)_pos_context.progress,
			(double)false_offset_position(0),
			(double)false_offset_position(1),
			(double)false_offset_velocity(0),
			(double)false_offset_velocity(1));
	}
}

void GnssSpoofGen::spoofGradualCarryOff(sensor_gps_s &gps)
{
	if (!_origin_initialized)
	{
		return;
	}

	if (_pos_context.last_sample_time_us == 0)
	{
		PX4_INFO("carryoff started at sample=%" PRIu64, gps.timestamp_sample);
		_pos_context.start_sample = gps;

		const matrix::Vector2f start_ne = _origin_projection.project(
				gps.latitude_deg,
				gps.longitude_deg);

		_pos_context.start_ned = matrix::Vector3f{
			start_ne(0),
			start_ne(1),
			0.f
		};

		_pos_context.last_sample_time_us = gps.timestamp_sample;
		return;
	}

	if (gps.timestamp_sample <= _pos_context.last_sample_time_us)
	{
		return;
	}

	const float dt = (gps.timestamp_sample - _pos_context.last_sample_time_us) / 1e6f;
	_pos_context.last_sample_time_us = gps.timestamp_sample;

	const matrix::Vector3f path_displacement = _pos_context.target_ned - _pos_context.start_ned;
	const float total_distance_m = path_displacement.norm();

	if (total_distance_m <= FLT_EPSILON || _pos_context.max_speed <= 0)
	{
		return;
	}

	const matrix::Vector3f start_velocity_ned{
		_pos_context.start_sample.vel_n_m_s,
		_pos_context.start_sample.vel_e_m_s,
		0.f,
	};

	const float start_speed_m_s = start_velocity_ned.norm();
	const float available_added_speed_m_s = _pos_context.max_speed - start_speed_m_s;

	if (available_added_speed_m_s <= 0.f)
	{
		//PX4_WARN("carryoff max speed must exceed start speed");
		return;
	}

	const float transition_duration_s = kMaxSlope * total_distance_m / available_added_speed_m_s;

	const float elapsed_fraction = dt / transition_duration_s;
	_pos_context.progress = math::min(_pos_context.progress + elapsed_fraction, 1.f);

	const float position_fraction = rampQuintic(_pos_context.progress);
	const float position_rate_fraction = rampQuinticDerivative(_pos_context.progress);

	const float initial_velocity_fraction = rampQuinticCarryOffInitial(_pos_context.progress);
	const float initial_velocity_rate_fraction = rampQuinticDerivativeCarryOffInitial(_pos_context.progress);

	const matrix::Vector3f false_position_ned =
		_pos_context.start_ned
		+ path_displacement * position_fraction // new trajectory contribution
		+ start_velocity_ned * (transition_duration_s * initial_velocity_fraction); // old trajectory contribution

	const matrix::Vector3f false_velocity_ned =
		path_displacement * (position_rate_fraction / transition_duration_s) // new trajectory contribution
		+ start_velocity_ned * initial_velocity_rate_fraction; // old trajectory contribution
	
	double spoofed_lat{};
	double spoofed_lon{};

	_origin_projection.reproject(
		false_position_ned(0),
		false_position_ned(1),
		spoofed_lat,
		spoofed_lon);

	gps.latitude_deg = spoofed_lat;
	gps.longitude_deg = spoofed_lon;

	gps.vel_n_m_s = false_velocity_ned(0);
	gps.vel_e_m_s = false_velocity_ned(1);

	gps.vel_m_s = sqrtf(gps.vel_n_m_s * gps.vel_n_m_s + gps.vel_e_m_s * gps.vel_e_m_s);

	if (gps.vel_m_s > 0.01f)
	{
		gps.cog_rad = atan2f(gps.vel_e_m_s, gps.vel_n_m_s);
	}
}

void GnssSpoofGen::setMode(Mode mode)
{
	_mode = mode;
	_elapsed_from_last_spoof_us = 0;
}

void GnssSpoofGen::maybeInitOrigin(sensor_gps_s& gps)
{
	if (!_origin_initialized)
	{
		_origin_projection.initReference(
			gps.latitude_deg,
			gps.longitude_deg,
			gps.timestamp_sample);
		_origin_initialized = true;
	}
}

void GnssSpoofGen::Run()
{
	if (should_exit())
	{
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (!_gps1_advertised)
	{
		if (orb_exists(ORB_ID(vehicle_gps_position), 0) != PX4_OK)
		{
			return; // GPS #0 simulator publisher is not ready yet.
		}

		if (!_gps1_pub.advertise())
		{
			PX4_ERR("failed to advertise GPS #1 test output");
			ScheduleClear();
			exit_and_cleanup(desc);
			return;
		}

		if (_gps1_pub.get_instance() != 1)
		{
			PX4_ERR("expected GPS instance 1, got %d",
			_gps1_pub.get_instance());
			ScheduleClear();
			exit_and_cleanup(desc);
			return;
		}

		_gps1_advertised = true;
		PX4_INFO("publishing clean GPS copy on instance 1");
	}

	sensor_gps_s gps{};

	if (!_gps0_sub.update(&gps))
	{
		return;
	}

	maybeInitOrigin(gps);

	switch (_mode)
	{
		case Mode::GradualOffset:
			spoofGradualOffset(gps);
			break;
		case Mode::GradualCarryOff:
			spoofGradualCarryOff(gps);
			break;
		case Mode::None:
			break;
	}

	gps.timestamp = hrt_absolute_time();
	_gps1_pub.publish(gps);
}

int GnssSpoofGen::task_spawn(int argc, char *argv[])
{
	GnssSpoofGen *instance = new GnssSpoofGen();

	if (instance)
	{
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}
	}

	PX4_ERR("init failed");
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

void GnssSpoofGen::resetSpoofPositionContextNED(const matrix::Vector2f& tgt_ned, const float max_speed)
{
	_pos_context = {};
	_pos_context.target_ned = matrix::Vector3f{tgt_ned(0), tgt_ned(1), 0.f};
	_pos_context.max_speed = max_speed;
}

bool GnssSpoofGen::resetSpoofPositionContextGCS(const matrix::Vector2f& tgt_offset_ned, const float max_speed)
{
	if (!_origin_initialized)
	{
		return false;
	}

	matrix::Vector2f spoofed_position_ne = _origin_projection.project(
		tgt_offset_ned(0),
		tgt_offset_ned(1));

	resetSpoofPositionContextNED(spoofed_position_ne, max_speed);
	return true;
}

int GnssSpoofGen::maybeParseOffsetCommand(int argc, char *argv[], GnssSpoofGen& instance)
{
	if (argc == 1)
	{
		instance.resetSpoofPositionContextNED({kDefaultOffset, kDefaultOffset}, kDefaultOffsetMaxVelocity);
	}
	if (argc == 3 || argc == 4)
	{
		float x = NAN;
		float y = NAN;
		float max_speed = NAN;
		if (!(parseFloat(argv[1], x) && parseFloat(argv[2], y)))
		{
			PX4_WARN("Failed to convert x,y into floats");
			return PX4_ERROR;
		}

		if (argc == 3)
		{
			max_speed = kDefaultOffsetMaxVelocity;
		}
		if (argc == 4 && !parseFloat(argv[3], max_speed))
		{
			PX4_WARN("Failed to convert max_speed into float");
			return PX4_ERROR;
		}

		if (max_speed <= 0.f)
		{
			PX4_WARN("Zero or negative max_speed detected");
			return PX4_ERROR;
		}

		instance.resetSpoofPositionContextNED({x, y}, max_speed);
	}

	PX4_INFO("offset accepted: D=(%.1f, %.1f, %.1f) m, max_speed=%.1f m/s",
		(double)instance._pos_context.target_ned(0),
		(double)instance._pos_context.target_ned(1),
		(double)instance._pos_context.target_ned(2),
		(double)instance._pos_context.max_speed);

	instance.setMode(Mode::GradualOffset);

	return PX4_OK;
}

int GnssSpoofGen::maybeParseCarryOffCommand(int argc, char *argv[], GnssSpoofGen& instance)
{
	float x = kDefaultCarryOffLatDeg;
	float y = kDefaultCarryOffLonDeg;
	float max_speed = kDefaultCarryOffMaxVelocity;

	bool gcs = false;

	if (argc == 1)
	{
		gcs = true;
	}
	if (argc == 4 || argc == 5)
	{
		if (!(parseFloat(argv[2], x) && parseFloat(argv[3], y)))
		{
			PX4_WARN("Failed to convert x,y into floats");
			return PX4_ERROR;
		}

		if (argc == 5 && !parseFloat(argv[4], max_speed))
		{
			PX4_WARN("Failed to convert max_speed into float");
			return PX4_ERROR;
		}
		if (max_speed <= 0.f)
		{
			PX4_WARN("Zero or negative max_speed detected");
			return PX4_ERROR;
		}

		if (strcmp(argv[1], "ned") == 0)
		{
			gcs = false;
		}
		else if (strcmp(argv[1], "gcs") == 0)
		{
			gcs = true;
		}
	}

	if (gcs)
	{
		if (!instance.resetSpoofPositionContextGCS({x, y}, max_speed))
		{
			PX4_WARN("Failed to resetSpoofPositionTransitionGCS");
			return PX4_ERROR;
		}
	}
	else
	{
		instance.resetSpoofPositionContextNED({x, y}, max_speed);
	}

	PX4_INFO("carry off accepted: TGT=(%.1f, %.1f, %.1f) m, max_speed=%.1f m/s",
		(double)instance._pos_context.target_ned(0),
		(double)instance._pos_context.target_ned(1),
		(double)instance._pos_context.target_ned(2),
		(double)instance._pos_context.max_speed);

	instance.setMode(Mode::GradualCarryOff);

	return PX4_OK;
}

int GnssSpoofGen::print_status()
{
	const char *mode_name = "clean";

	switch (_mode)
	{
		case Mode::GradualOffset:
			mode_name = "offset";
			break;
		case Mode::GradualCarryOff:
			mode_name = "carryoff";
			break;
		case Mode::None:
			break;
	}

	PX4_INFO("mode: %s", mode_name);
	PX4_INFO("GPS: input instance #0 -> output instance #%d (%s)",
		_gps1_pub.get_instance(),
		_gps1_advertised ? "advertised" : "not advertised");
	PX4_INFO("local origin: %s", _origin_initialized ? "initialized" : "not initialized");

	if (_mode == Mode::None)
	{
		return PX4_OK;
	}

	PX4_INFO("progress: %.1f %%", (double)(_pos_context.progress * 100.f));
	PX4_INFO("configured max speed: %.1f m/s", (double)_pos_context.max_speed);

	if (_origin_initialized && _mode == Mode::GradualCarryOff)
	{
		double origin_lat{};
		double origin_lon{};
		double target_lat{};
		double target_lon{};

		_origin_projection.reproject(0.f, 0.f, origin_lat, origin_lon);
		_origin_projection.reproject(_pos_context.target_ned(0),
		_pos_context.target_ned(1),
		target_lat, target_lon);

		PX4_INFO("origin GCS: (%.7f, %.7f)", origin_lat, origin_lon);
		PX4_INFO("target NED: (%.1f, %.1f, %.1f) m",
			(double)_pos_context.target_ned(0),
			(double)_pos_context.target_ned(1),
			(double)_pos_context.target_ned(2));
		PX4_INFO("target GCS: (%.7f, %.7f)", target_lat, target_lon);
	}

	return PX4_OK;
}

int GnssSpoofGen::custom_command(int argc, char *argv[])
{
	GnssSpoofGen *instance = get_instance<GnssSpoofGen>(desc);

	if (!instance)
	{
		return print_usage("not running");
	}

	if ((argc == 1 || argc == 3 || argc == 4) && strcmp(argv[0], "offset") == 0)
	{
		return maybeParseOffsetCommand(argc, argv, *instance);
	}
	if ((argc == 1 || argc == 4 || argc == 5) && strcmp(argv[0], "carryoff") == 0)
	{
		return maybeParseCarryOffCommand(argc, argv, *instance);
	}

	if (argc == 1 && strcmp(argv[0], "clear") == 0)
	{
		instance->setMode(Mode::None);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int GnssSpoofGen::print_usage(const char *reason)
{
	if (reason)
	{
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Minimal SITL GPS test publisher. It republishes GPS instance 0 as instance 1.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("spoofer", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("offset",   "Creates position offset from GPS#0. Usage: offset [<north> <east>] [<maxspeed>]");
	PRINT_MODULE_USAGE_COMMAND_DESCR("carryoff", "Changes the GPS position to a given one. Usage: carryoff [\"gcs\"|\"ned\" <x> <y>] [<maxspeed>]");
	PRINT_MODULE_USAGE_COMMAND_DESCR("clear",    "Disable spoofing and publish a clean GPS copy");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int spoofer_main(int argc, char *argv[])
{
	return ModuleBase::main(GnssSpoofGen::desc, argc, argv);
}

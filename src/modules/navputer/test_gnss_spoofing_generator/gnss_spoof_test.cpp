#include "gnss_spoof_test.hpp"

#include <drivers/drv_hrt.h>

#include <cstring>
#include <math.h>

namespace
{
constexpr float kVelFixedRotationRad = M_PI_4; // PI/4 rad = 45 deg
constexpr float kVelFixedMagnitudeOffsetMS = 5.f; // m/s
constexpr float kPosFixedOffset = 200.f; // m
} // namespace

using namespace time_literals;

ModuleBase::Descriptor GnssSpoofTest::desc{task_spawn, custom_command, print_usage};

GnssSpoofTest::GnssSpoofTest() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool GnssSpoofTest::init()
{
	if (!_gps1_pub.advertise())
	{
		PX4_ERR("failed to advertise GPS test output");
		return false;
	}

	if (_gps1_pub.get_instance() != 1)
	{
		PX4_ERR("expected GPS instance 1, got %d", _gps1_pub.get_instance());
		return false;
	}

	ScheduleOnInterval(10_ms);
	return true;
}

void GnssSpoofTest::spoofVelocityByFixedRotation(sensor_gps_s &gps)
{
	const float hor_speed = sqrtf(gps.vel_n_m_s * gps.vel_n_m_s + gps.vel_e_m_s * gps.vel_e_m_s);
	const float angle = atan2f(gps.vel_e_m_s, gps.vel_n_m_s); // from North to East angle
	const float spoofed_angle = angle + kVelFixedRotationRad;

	gps.vel_n_m_s = hor_speed * cosf(spoofed_angle);
	gps.vel_e_m_s = hor_speed * sinf(spoofed_angle);

	gps.vel_m_s = hor_speed;
	gps.cog_rad = atan2f(gps.vel_e_m_s, gps.vel_n_m_s);
}

void GnssSpoofTest::spoofVelocityByFixedMagnitudeOffset(sensor_gps_s &gps)
{
	const float hor_speed = sqrtf(gps.vel_n_m_s * gps.vel_n_m_s + gps.vel_e_m_s * gps.vel_e_m_s);
	const float angle = atan2f(gps.vel_e_m_s, gps.vel_n_m_s); // from North to East angle
	const float spoofed_hor_speed = hor_speed + kVelFixedMagnitudeOffsetMS;

	gps.vel_n_m_s = spoofed_hor_speed * cosf(angle);
	gps.vel_e_m_s = spoofed_hor_speed * sinf(angle);

	gps.vel_m_s = spoofed_hor_speed;
	gps.cog_rad = atan2f(gps.vel_e_m_s, gps.vel_n_m_s);
}

void GnssSpoofTest::spoofPositionFixedOffset(sensor_gps_s &gps)
{
	if (!_origin_initialized)
	{
		return;
	}

	matrix::Vector2f position_ne = _origin_projection.project(
		gps.latitude_deg,
		gps.longitude_deg);

	const matrix::Vector2f spoof_offset_ne{
		kPosFixedOffset, // North
		kPosFixedOffset  // East
	};

	position_ne += spoof_offset_ne;

	double spoofed_lat;
	double spoofed_lon;

	_origin_projection.reproject(
		position_ne(0),
		position_ne(1),
		spoofed_lat,
		spoofed_lon);

	gps.latitude_deg = spoofed_lat;
	gps.longitude_deg = spoofed_lon;
}

void GnssSpoofTest::setMode(Mode mode)
{
	_mode = mode;
	_elapsed_from_last_spoof_us = 0;
}

void GnssSpoofTest::maybeInitOrigin(sensor_gps_s& gps)
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

void GnssSpoofTest::Run()
{
	if (should_exit())
	{
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	sensor_gps_s gps{};

	if (!_gps0_sub.update(&gps))
	{
		return;
	}

	maybeInitOrigin(gps);

	switch (_mode)
	{
		case Mode::VelocityFixedRotation:
			spoofVelocityByFixedRotation(gps);
			break;

		case Mode::VelocityFixedMagnitudeOffset:
			spoofVelocityByFixedMagnitudeOffset(gps);
			break;

		case Mode::PositionFixedOffset:
			spoofPositionFixedOffset(gps);
			break;

		//case Mode::VelocityRampRotation:
		//case Mode::VelocityRampMagnitudeOffset:
		//case Mode::PositionRampOffset:
		case Mode::None:
			break;
	}

	gps.timestamp = hrt_absolute_time();
	_gps1_pub.publish(gps);
}

int GnssSpoofTest::task_spawn(int argc, char *argv[])
{
	GnssSpoofTest *instance = new GnssSpoofTest();

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

int GnssSpoofTest::custom_command(int argc, char *argv[])
{
	GnssSpoofTest *instance = get_instance<GnssSpoofTest>(desc);

	if (!instance)
	{
		return print_usage("not running");
	}

	if (argc == 1 && !strcmp(argv[0], "vel_frot"))
	{
		instance->setMode(Mode::VelocityFixedRotation);
		return PX4_OK;
	}

	if (argc == 1 && !strcmp(argv[0], "vel_fmag"))
	{
		instance->setMode(Mode::VelocityFixedMagnitudeOffset);
		return PX4_OK;
	}

	if (argc == 1 && !strcmp(argv[0], "pos_f"))
	{
		instance->setMode(Mode::PositionFixedOffset);
		return PX4_OK;
	}

	if (argc == 1 && !strcmp(argv[0], "stop"))
	{
		instance->setMode(Mode::None);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int GnssSpoofTest::print_usage(const char *reason)
{
	if (reason)
	{
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Minimal SITL GPS test publisher. It republishes GPS instance 0 as instance 1.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("gnss_spoof_gen", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("velocity", "Select the velocity spoofing stub");
	PRINT_MODULE_USAGE_COMMAND_DESCR("position", "Select the position spoofing stub");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Disable spoofing and publish a clean GPS copy");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gnss_spoof_gen_main(int argc, char *argv[])
{
	return ModuleBase::main(GnssSpoofTest::desc, argc, argv);
}

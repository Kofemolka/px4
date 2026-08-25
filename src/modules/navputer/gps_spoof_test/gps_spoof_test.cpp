#include "gps_spoof_test.hpp"

#include <drivers/drv_hrt.h>

#include <cstring>

using namespace time_literals;

ModuleBase::Descriptor GpsSpoofTest::desc{task_spawn, custom_command, print_usage};

GpsSpoofTest::GpsSpoofTest() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool GpsSpoofTest::init()
{
	if (!_gps1_pub.advertise()) {
		PX4_ERR("failed to advertise GPS test output");
		return false;
	}

	if (_gps1_pub.get_instance() != 1) {
		PX4_ERR("expected GPS instance 1, got %d", _gps1_pub.get_instance());
		return false;
	}

	ScheduleOnInterval(10_ms);
	return true;
}

void GpsSpoofTest::spoofVelocity(sensor_gps_s &gps)
{
	// TODO: implement velocity spoofing.
}

void GpsSpoofTest::spoofPosition(sensor_gps_s &gps)
{
	// TODO: implement position spoofing.
}

void GpsSpoofTest::setMode(Mode mode)
{
	_mode = mode;
}

void GpsSpoofTest::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	sensor_gps_s gps{};

	if (!_gps0_sub.update(&gps)) {
		return;
	}

	switch (_mode) {
	case Mode::Velocity:
		spoofVelocity(gps);
		break;

	case Mode::Position:
		spoofPosition(gps);
		break;

	case Mode::None:
		break;
	}

	gps.timestamp = hrt_absolute_time();
	_gps1_pub.publish(gps);
}

int GpsSpoofTest::task_spawn(int argc, char *argv[])
{
	GpsSpoofTest *instance = new GpsSpoofTest();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	PX4_ERR("init failed");
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int GpsSpoofTest::custom_command(int argc, char *argv[])
{
	GpsSpoofTest *instance = get_instance<GpsSpoofTest>(desc);

	if (!instance) {
		return print_usage("not running");
	}

	if (argc == 1 && !strcmp(argv[0], "velocity")) {
		instance->setMode(Mode::Velocity);
		return PX4_OK;
	}

	if (argc == 1 && !strcmp(argv[0], "position")) {
		instance->setMode(Mode::Position);
		return PX4_OK;
	}

	if (argc == 1 && !strcmp(argv[0], "stop")) {
		instance->setMode(Mode::None);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int GpsSpoofTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Minimal SITL GPS test publisher. It republishes GPS instance 0 as instance 1.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("gps_spoof_test", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("velocity", "Select the velocity spoofing stub");
	PRINT_MODULE_USAGE_COMMAND_DESCR("position", "Select the position spoofing stub");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Disable spoofing and publish a clean GPS copy");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gps_spoof_test_main(int argc, char *argv[])
{
	return ModuleBase::main(GpsSpoofTest::desc, argc, argv);
}

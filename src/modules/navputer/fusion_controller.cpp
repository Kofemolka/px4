#include "fusion_controller.hpp"

using namespace time_literals;

FusionController::FusionController(ModuleParams *parent, FusionControl &fc) :
	ModuleParams(parent),
	_fc(fc)
{
}

void FusionController::updateParams()
{
	updateParamsImpl();

	_fc.baro.enabled = _param_npt_fuse_baro.get();
	_fc.mag.enabled = _param_npt_fuse_mag.get();
	_fc.rngbcn.enabled = _param_npt_fuse_rngbc.get();
	_fc.of.enabled = _param_npt_fuse_of.get();
}

void FusionController::update(Ekf &ekf)
{
	if (!_param_npt_fuse_agp0.get()) {
		_fc.agp[0].enabled = false;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime latch_time = static_cast<hrt_abstime>(_param_npt_fc_agp_latch.get() * 1e6f);

	// each condition refreshes its own "last needed" timestamp while it holds, so the latch
	// below measures time since that specific condition cleared, not time since AGP turned on
	if (!ekf.global_origin_valid()) {
		_agp_last_origin_missing = now;
	}

	if (!ekf.isOtherSourceOfHorizontalPositionAidingThan(ekf.control_status_flags().aux_gpos)) {
		_agp_last_other_source_missing = now;
	}

	const bool origin_latched = hrt_elapsed_time(&_agp_last_origin_missing) < latch_time;
	const bool other_source_latched = hrt_elapsed_time(&_agp_last_other_source_missing) < latch_time;

	const bool agp_enabled = origin_latched || other_source_latched;

	if (agp_enabled != _fc.agp[0].enabled) {
		PX4_INFO("AGP fusion %s", agp_enabled ? "enabled" : "disabled");
	}

	_fc.agp[0].enabled = agp_enabled;
}

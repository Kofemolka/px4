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

	_fc.agp[0].enabled = false;
}

void FusionController::update(Ekf &ekf)
{
	if (!_param_npt_fuse_agp0.get()) {
		_fc.agp[0].enabled = false;

	} else {
		// AGP is only needed to bootstrap the origin or to stand in when nothing else is
		// aiding horizontal position; once another source is up, back off and let it lead.
		const bool agp_needed = !ekf.global_origin_valid()
				      || !ekf.isOtherSourceOfHorizontalPositionAidingThan(ekf.control_status_flags().aux_gpos);

		// once enabled, hold AGP fusion on for at least NPT_FC_AGP_LATCH even if agp_needed
		// drops out, to avoid rapidly toggling fusion on and off around the enable condition's edge
		bool agp_enabled = agp_needed;
		const hrt_abstime latch_time = _param_npt_fc_agp_latch.get() * 1_s;

		if (!agp_needed && _fc.agp[0].enabled && hrt_elapsed_time(&_agp_enabled_time) < latch_time) {
			agp_enabled = true;
		}

		if (agp_enabled != _fc.agp[0].enabled) {
			PX4_WARN("AGP fusion %s", agp_enabled ? "ENABLED" : "DISABLED");
		}

		if (agp_enabled && !_fc.agp[0].enabled) {
			_agp_enabled_time = hrt_absolute_time();
		}

		_fc.agp[0].enabled = agp_enabled;
	}
}

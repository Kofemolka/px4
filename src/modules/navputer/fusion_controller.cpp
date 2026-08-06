#include "fusion_controller.hpp"

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

		if(agp_needed != _fc.agp[0].enabled) {
			PX4_WARN("AGP fusion %s", agp_needed ? "ENABLED" : "DISABLED");
		}

		_fc.agp[0].enabled = agp_needed;
	}
}

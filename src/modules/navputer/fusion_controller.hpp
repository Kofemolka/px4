#ifndef FUSION_CONTROLLER_HPP
#define FUSION_CONTROLLER_HPP

#include "EKF/ekf.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

class FusionController : public ModuleParams {
public:
	FusionController(ModuleParams *parent, FusionControl &fc);

	void update(Ekf& _ekf);
	void setGpsTrusted(bool trusted);

protected:
	virtual void updateParams() override;

private:
	FusionControl &_fc;

	parameters *_params;

	hrt_abstime _agp_last_origin_missing{0};
	hrt_abstime _agp_last_other_source_missing{0};
	bool _gps_trusted{false};

	DEFINE_PARAMETERS(
		// per-source fusion enable
		(ParamBool<px4::params::NPT_FUSE_BARO>) _param_npt_fuse_baro,
		(ParamBool<px4::params::NPT_FUSE_MAG>) _param_npt_fuse_mag,
		(ParamBool<px4::params::NPT_FUSE_RNGBC>) _param_npt_fuse_rngbc,
		(ParamBool<px4::params::NPT_FUSE_AGP0>) _param_npt_fuse_agp0,
		(ParamBool<px4::params::NPT_FUSE_GPS>) _param_npt_fuse_gps,
		(ParamFloat<px4::params::NPT_FC_AGP_LATCH>) _param_npt_fc_agp_latch
	)
};

#endif

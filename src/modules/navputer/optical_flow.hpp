#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include "EKF/ekf.h"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_optical_flow.h>

// Direction-only optical flow fusion: constrains horizontal velocity to the
// gyro-compensated flow heading without asserting its magnitude.
class OpticalFlow : public ModuleParams
{
public:
	explicit OpticalFlow(ModuleParams *parent);

	void update(Ekf& _ekf);
private:
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::NPT_FUSE_OFH>) _param_npt_fuse_ofh
	)
};

#endif

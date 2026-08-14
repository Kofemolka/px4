#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include "EKF/ekf.h"

#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/aux_global_position.h>

// Direction-only optical flow fusion: constrains horizontal velocity to the
// gyro-compensated flow heading without asserting its magnitude.
class OpticalFlow : public ModuleParams
{
public:
	explicit OpticalFlow(ModuleParams *parent);

	void update(Ekf& ekf);
private:
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};

	uORB::PublicationMulti<aux_global_position_s> _aux_global_position_pub {ORB_ID(aux_global_position)};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::NPT_FUSE_OFH>) _param_npt_fuse_ofh
	)
};

#endif

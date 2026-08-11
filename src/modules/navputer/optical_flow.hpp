#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include "EKF/ekf.h"

#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_optical_flow.h>

class OpticalFlow
{
public:
	OpticalFlow();

	void update(Ekf& _ekf);
private:
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
};

#endif

#ifndef MLAT_AUX_HPP
#define MLAT_AUX_HPP

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <matrix/math.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/aux_global_position.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/ranging_beacon.h>

class MlatAux final
{
public:
	MlatAux() = default;

	void update();

	static constexpr int kMaxBeacons = 8;

private:
	// MLAT solver seed count (ported from lion::filters::MLAT / ArduPilot AP_NavEKF3_RngBcnFusion SolveMlat)
	static constexpr int kNumSeeds = 5;

	struct BeaconEntry {
		bool valid{false};
		uint8_t beacon_id{0};
		double lat{0.0};
		double lon{0.0};
		float alt{0.0f};
		float range{0.0f};
		float range_accuracy{0.0f};
		uint64_t timestamp_sample{0};
	};

	struct BeaconInput {
		matrix::Vector2f pos; // local north/east, m
		float range{0.f};     // flattened 2D ground range, m
		float range_accuracy{0.f};
		uint64_t timestamp_sample{0};
	};

	struct Candidate {
		bool valid{false};
		matrix::Vector2f anchor;
		matrix::Vector2f pos;
		float residual{1e10f};
		uint8_t votes{0};
	};

	static float squaredNorm(const matrix::Vector2f &v);

	// counts geometrically distinct beacons: beacons closer than kMinBeaconSeparation2 to an
	// already-counted one add no independent geometry and are folded together
	int countDistinctBeacons(const BeaconInput *inputs, int num_inputs);

	// centroid of all known beacon coordinates, used to bootstrap the projection reference
	// without depending on the EKF already having a global position (which may itself be
	// waiting on this AGP output)
	bool computeBeaconCentroid(double &lat, double &lon);

	// standard 2-D HDOP from the Fisher information matrix of unit line-of-sight vectors
	bool calcHdop(const matrix::Vector2f &from, const BeaconInput *inputs, int num_inputs, float &hdop);

	void solveCandidate(Candidate &candidate, const BeaconInput *inputs, int num_inputs);

	void buildLocalAnchors(Candidate (&candidates)[kNumSeeds], const matrix::Vector2f &last_pos);
	void buildGlobalAnchors(Candidate (&candidates)[kNumSeeds], const BeaconInput *inputs, int num_inputs);

	// merges candidates that converged within kBasinRadius2 of each other, keeping the
	// lower-residual member as representative and summing votes (ported from MLAT::evaluate())
	void mergeBasins(Candidate (&candidates)[kNumSeeds]);

	// picks the accepted solution among the merged basins, rejecting ambiguous cold-start fixes
	bool evaluate(Candidate (&candidates)[kNumSeeds], bool have_last_pos, const matrix::Vector2f &last_pos,
		      matrix::Vector2f &result);

	void updateBeaconStore();
	bool solve();

	uORB::Subscription _ranging_beacon_sub {ORB_ID(ranging_beacon)};
	uORB::Subscription _local_position_sub {ORB_ID(navput_local_position)};
	uORB::PublicationMulti<aux_global_position_s> _aux_global_position_pub {ORB_ID(aux_global_position)};

	BeaconEntry _beacons[kMaxBeacons] {};

	MapProjection _projection;

	hrt_abstime _last_run{0};
};

#endif // MLAT_AUX_HPP

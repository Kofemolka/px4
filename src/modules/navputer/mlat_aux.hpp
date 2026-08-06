#ifndef MLAT_AUX_HPP
#define MLAT_AUX_HPP

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/lat_lon_alt/lat_lon_alt.hpp>
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
		uint64_t timestamp_sample{0};
	};

	struct Candidate {
		bool valid{false};
		matrix::Vector2f anchor;
		matrix::Vector2f pos;
		float residual{1e10f};
		uint8_t votes{0};
	};

	struct Solution {
		matrix::Vector2f pos;
		float residual{1e10f};
	};

	static float squaredNorm(const matrix::Vector2f &v);

	// counts geometrically distinct beacons: beacons closer than kMinBeaconSeparation2 to an
	// already-counted one add no independent geometry and are folded together
	int countDistinctBeacons(const BeaconInput *inputs, int num_inputs);

	// centroid of all known beacon coordinates, used to bootstrap the projection reference
	// without depending on the EKF already having a global position (which may itself be
	// waiting on this AGP output)
	bool computeBeaconCentroid(double &lat, double &lon);

	// bearing from (lat1,lon1) to (lat2,lon2), radians, 0=N, clockwise (spherical approximation,
	// sufficient for choosing a WGS84 directional radius -- ported from lion::geo::get_bearing)
	static float bearingRad(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

	// WGS84 radius of curvature in the direction from (center_lat,center_lon) to
	// (target_lat,target_lon) -- differs from a single global/mean radius by direction and
	// latitude; ported from lion::geo::get_local_earth_radius (Euler's radius-of-curvature formula)
	static float directionalEarthRadius(double center_lat_deg, double center_lon_deg,
					     double target_lat_deg, double target_lon_deg);

	// ratio between the radius _projection assumes (CONSTANTS_RADIUS_OF_EARTH) and the true
	// directional radius toward (lat,lon) from _projection's own reference point -- multiplying
	// a physical range by this factor expresses it in the same (uniform-radius-approximated)
	// metric that _projection's positions already implicitly use. Ported from
	// lion::filters::MLAT/BeaconsSource's "distortion_factor" (see lion/beacons_source.cpp:
	// project_beacon() and lion/filters/mlat.cpp: prepare_distances()).
	float distortionFactor(double lat, double lon) const;

	// picks the lat/lon _projection should be centered on this cycle, in priority order:
	// the EKF's own current position (converted out of navput_local_position's own reference),
	// else the last position MLAT itself solved for, else the beacon centroid as a last resort
	// for a true cold start. Returns false only if none of the three are available yet.
	// *centered_on_known_point is true unless the beacon-centroid fallback was used, which the
	// caller uses to decide between local (seeded near the center) and global (bounding-box)
	// anchors, matching lion::BeaconsSource::update_projection()'s re-centering behavior.
	bool selectProjectionCenter(const navput_local_position_s &local_pos, matrix::Vector2d &center_lat_lon,
				    bool &centered_on_known_point);

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
		      Candidate &solution);

	void updateBeaconStore();
	bool solve();

	uORB::Subscription _ranging_beacon_sub {ORB_ID(ranging_beacon)};
	uORB::Subscription _local_position_sub {ORB_ID(navput_local_position)};
	uORB::PublicationMulti<aux_global_position_s> _aux_global_position_pub {ORB_ID(aux_global_position)};

	BeaconEntry _beacons[kMaxBeacons] {};

	MapProjection _projection;

	// last position MLAT itself successfully solved for (lat, lon), used as a fallback
	// projection center (before falling back further to the beacon centroid) when the EKF's
	// own position is still unknown -- see selectProjectionCenter()
	bool _has_last_solution{false};
	matrix::Vector2d _last_solution_lat_lon{};

	hrt_abstime _last_run{0};
};

#endif // MLAT_AUX_HPP

/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file elevation_initializer.cpp
 * Implementation of the ElevationInitializer.
 *
 * @author
 */

#include "elevation_initializer.hpp"

#include <px4_platform_common/defines.h>
#include <matrix/helper_functions.hpp>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
constexpr char kHeightMapDirectory[] = PX4_STORAGEDIR"/navputer/heightmaps";
constexpr size_t kFileNameBuffSize = 96;
constexpr int kDimension = 1201;
constexpr off_t kHgt1201FileSize = static_cast<off_t>(kDimension) * kDimension * sizeof(int16_t);
constexpr int16_t kHgtNoData = INT16_MIN;

bool makeHgtPath(double latitude, double longitude, char *path, size_t path_size)
{
	// Southwestern corner of the containing 1deg × 1deg tile
	const int tile_lat = static_cast<int>(floor(latitude));
	const int tile_lon = static_cast<int>(floor(longitude));

	const char lat_hemisphere = tile_lat >= 0 ? 'N' : 'S';
	const char lon_hemisphere = tile_lon >= 0 ? 'E' : 'W';

	const int written = snprintf(
		   path,
		   path_size,
		   "%s/%c%02d%c%03d.hgt",
		   kHeightMapDirectory,
		   lat_hemisphere,
		   abs(tile_lat),
		   lon_hemisphere,
		   abs(tile_lon));

	return written > 0 && static_cast<size_t>(written) < path_size;
}

} // namespace

bool ElevationInitializer::isPreLookupStateOk(const Ekf& ekf, double& latitude, double& longitude, float& altitude)
{
	if (_state != State::Pending)
	{
		return false;
	}

	hrt_abstime origin_time;
	ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude);

	if (!ekf.isGlobalHorizontalPositionValid())
	{
		return false;
	}

	// Needed condition for PX4 to backtrack its elevation when the altitude origin will be changed
	if (!ekf.isLocalVerticalPositionValid())
	{
		return false;
	}

	return true;
}

void ElevationInitializer::update(Ekf& ekf)
{
	double latitude{NAN};
	double longitude{NAN};
	float altitude{NAN};

	if (!isPreLookupStateOk(ekf, latitude, longitude, altitude))
	{
		return;
	}

	const hrt_abstime lookup_started_at = hrt_absolute_time();
	const auto res = lookup(latitude, longitude);
	const hrt_abstime elapsed_time = hrt_elapsed_time(&lookup_started_at);
	PX4_INFO("ElevationInitializer: Lookup lasted for %llu milliseconds", elapsed_time / 1000ULL);
	
	if (!res.success)
	{
		_state = State::Done;
		PX4_WARN("ElevationInitializer: Failed to find the region in a height map");
		return;
	}
	else if (res.nodata || !PX4_ISFINITE(res.terrain_elevation_m))
	{
		_state = State::Done;
		PX4_WARN("ElevationInitializer: The region in the height map has no data");
		return;
	}

	if (PX4_ISFINITE(altitude))
	{
		PX4_INFO("ElevationInitializer: rewriting the old altitude %d with the new altitude %d.",
			 static_cast<int>(altitude), static_cast<int>(res.terrain_elevation_m));
	}

	if (!ekf.setEkfGlobalOrigin(latitude, longitude, res.terrain_elevation_m))
	{
		_state = State::Done;
		PX4_WARN("ElevationInitializer: Failed to change Ekf global origin");
		return;
	}

	// Get origin and recheck if altitude is ok now
	{
		hrt_abstime new_origin_time;
		double new_latitude{NAN};
		double new_longitude{NAN};
		float new_altitude{NAN};
		ekf.getEkfGlobalOrigin(new_origin_time, new_latitude, new_longitude, new_altitude);

		if (!PX4_ISFINITE(new_altitude))
		{
			_state = State::Done;
			PX4_WARN("ElevationInitializer: Altitude origin is invalid after origin change");
			return;
		}
	}

	PX4_INFO("ElevationInitializer: Successfully changing the altitude origin");
	_state = State::Done;
}

ElevationInitializer::LookupResult ElevationInitializer::lookup(double latitude, double longitude)
{
	LookupResult res;

	if (!PX4_ISFINITE(latitude) || !PX4_ISFINITE(longitude) || latitude <= -90.0 || latitude >= 90.0)
	{
		PX4_WARN("ElevationInitializer: wrong lon/lat");
		return res;
	}

	longitude = matrix::wrap(longitude, -180.0, 180.0);

	// Example: N50E025.hgt
	char path[kFileNameBuffSize] {};

	if (!makeHgtPath(latitude, longitude, path, sizeof(path)))
	{
		PX4_WARN("ElevationInitializer: failed to create filename with given lat/lon");
		return {};
	}
	PX4_INFO("ElevationInitializer: search for the needed HGT tile: %s", path);

	const int fd = ::open(path, O_RDONLY);
	if (fd < 0)
	{
		PX4_WARN("ElevationInitializer: failed to open %s: %d", path, errno);
		return res;
	}

	res = lookupInFile(fd, path, latitude, longitude);
	::close(fd);

	return res;
}

ElevationInitializer::LookupResult ElevationInitializer::lookupInFile(const int fd, const char* path, double latitude, double longitude)
{
	ElevationInitializer::LookupResult res;

	struct stat file_stat {};

	if (::fstat(fd, &file_stat) != 0)
	{
		PX4_WARN("ElevationInitializer: fstat failed for %s : %d", path, errno);
		return res;
	}
	if (file_stat.st_size != kHgt1201FileSize)
	{
		PX4_WARN("ElevationInitializer: HGT file %s with unsupported size", path);
		return res;
	}

	const int tile_south = static_cast<int>(floor(latitude));
	const int tile_west = static_cast<int>(floor(longitude));
	const int intervals = kDimension - 1;

	int row = lround(((tile_south + 1.0) - latitude) * intervals);
	row = math::constrain(row, 0, kDimension - 1);

	int col = lround((longitude - static_cast<double>(tile_west)) * intervals);
	col = math::constrain(col, 0, kDimension - 1);

	const uint64_t pixel_index = static_cast<uint64_t>(row)
		* static_cast<uint64_t>(kDimension)
		+ static_cast<uint64_t>(col);

	const uint64_t offset_in_bytes = pixel_index * sizeof(int16_t);

	if (offset_in_bytes > file_stat.st_size - sizeof(int16_t)) {
		PX4_WARN("ElevationInitializer: HGT offset overflow in %s", path);
		return res;
	}

	const off_t offset = static_cast<off_t>(offset_in_bytes);
	uint8_t bytes[2] {};

	const ssize_t bytes_read = ::pread(fd, bytes, sizeof(bytes), offset);

	if (bytes_read != static_cast<ssize_t>(sizeof(bytes)))
	{
		PX4_WARN("ElevationInitializer: failed to read int16_t from %s: %d", path, errno);
		return res;
	}

	// HGT samples are signed, big-endian int16 values.
	// Decode from big-endian to little endian:
	const uint16_t raw = (static_cast<uint16_t>(bytes[0]) << 8)
		| static_cast<uint16_t>(bytes[1]);
	const int16_t elevation_m = static_cast<int16_t>(raw);

	res.success = true;
	res.terrain_elevation_m = static_cast<float>(elevation_m);

	if (elevation_m == kHgtNoData)
	{
		res.nodata = true;
		PX4_WARN("ElevationInitializer: NoData in the %s at row %d, col %d", path, row, col);
	}
	else
	{
		PX4_INFO("ElevationInitializer: found %.1f elevation in %s at row %d, col %d",
			static_cast<double>(res.terrain_elevation_m),
			path,
			row,
			col);
	}

	return res;
}

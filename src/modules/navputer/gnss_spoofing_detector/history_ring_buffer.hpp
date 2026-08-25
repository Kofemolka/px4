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
 * @file history_ring_buffer.hpp
 * Implementation of the HistoryRingBuffer.
 *
 * @author
 */

#ifndef HISTORY_RING_BUFFER_HPP
#define HISTORY_RING_BUFFER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// Fixed-capacity timestamp-ordered history.
// T must contain uint64_t time_us;
// Samples must be pushed with strictly increasing time_us.
template<typename T, size_t Capacity>
class HistoryRingBuffer
{
	static_assert(Capacity > 0, "HistoryRingBuffer capacity must be positive");

public:
	struct BracketIndices
	{
		size_t before;
		size_t after;
	};

	// Push a newer sample
	bool push(const T &sample)
	{
		if (!empty() && sample.time_us <= newest().time_us)
			return false;

		_samples[_next] = sample;
		_next = (_next + 1) % Capacity;

		if (_count < Capacity) {
			_count++;
		}

		return true;
	}

	void reset()
	{
		_next = 0;
		_count = 0;
	}

	bool empty() const { return _count == 0; }
	bool full() const { return _count == Capacity; }
	size_t size() const { return _count; }

	// offset == 0          -> oldest sample
	// offset == size() - 1 -> newest sample
	const T &atOldestOffset(size_t offset) const
	{
		const size_t oldest_index = (_next + Capacity - _count) % Capacity;
		return _samples[(oldest_index + offset) % Capacity];
	}

	const T &oldest() const
	{
		return atOldestOffset(0);
	}

	const T &newest() const
	{
		return atOldestOffset(_count - 1);
	}

	// Binary Search:
	// Find the newest sample whose timestamp is <= requested time.
	std::optional<size_t> findLastAtOrBefore(uint64_t time_us) const
	{
		if (empty() || time_us < oldest().time_us)
			return std::nullopt;

		if (time_us >= newest().time_us)
			return _count - 1;

		size_t low = 0;
		size_t high = _count - 1;

		while (low < high)
		{
			const size_t middle = low + (high - low + 1) / 2;
			if (atOldestOffset(middle).time_us <= time_us)
				low = middle;
			else
				high = middle - 1;
		}

		return low;
	}

	// Find samples s1, s2 
	// If no sample == time_us: return {s1, s2} which: s1 < time_us < s2
	// If there is an sample == time_us: find it and return as {sample, sample}
	std::optional<BracketIndices> findBracket(uint64_t time_us) const
	{
		const auto before_index = findLastAtOrBefore(time_us);

		if (!before_index)
			return std::nullopt;

		const size_t before = *before_index;
		const T &before_sample = atOldestOffset(before);

		if (before_sample.time_us == time_us)
			return BracketIndices{before, before};

		const size_t after = before + 1;

		if (after >= _count)
			return std::nullopt;

		return BracketIndices{before, after};
	}

private:
	std::array<T, Capacity> _samples{};
	size_t _next{0};
	size_t _count{0};
};

#endif // HISTORY_RING_BUFFER_HPP

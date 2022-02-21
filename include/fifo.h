/*
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *
 *  Copyright (C) 2022  The DOSBox Staging Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

// The FIFO class provides a fixed-length FIFO queue without undefined behavior.
//  - Reading an empty fifo simply returns an empty (default) value.
//  - Writing to a full fifo bumps the oldest value.
//  - Popping an empty fifo leaves the fifo unchanged.
//  - Modifying in-place items or std::moving in-place is deliberately blocked.
//  - Clear()ing the fifo is allowed and is efficient.

#ifndef DOSBOX_FIFO_H
#define DOSBOX_FIFO_H

#include <cassert>
#include <cstdint>
#include <queue>

template <typename value_t, int max_length, typename queue_t = std::queue<value_t>>
class fifo : public queue_t {
public:
	// Constructs an empty fifo with compile-time check of the queue length
	fifo() { static_assert(max_length > 0, "max_length must be positive"); }

	// push an item onto the fifo via copy
	void push(const value_t &value) noexcept
	{
		pop_if_full();
		queue_t::push(value);
	}

	// push an item into the fifo via move
	void push(value_t &value) noexcept
	{
		pop_if_full();
		queue_t::push(std::move(value));
	}

	// emplace an item info the fifo
	const value_t &emplace(value_t &&value) noexcept
	{
		pop_if_full();
		queue_t::emplace(std::move(value));
	}

	// get a read-only reference to the first value (or a default value)
	const value_t &first() const noexcept
	{
		return queue_t::size() ? queue_t::front() : empty_value;
	}
	// get a read-only reference to the last value (or a default value)
	const value_t &last() const noexcept
	{
		return queue_t::size() ? queue_t::back() : empty_value;
	}

	// get the number of items in the fifo as a simple integer
	int length() const noexcept
	{
		const auto num_items = queue_t::size();
		assert(num_items <= max_length);
		return static_cast<int>(num_items);
	}

	// pop the oldest item or leave the fifo empty otherwise
	void pop() noexcept
	{
		if (queue_t::size())
			queue_t::pop();
	}

	// clear the fifo, but leave it with the same capacity
	void clear() noexcept
	{
		*this = fifo<value_t, max_length>();
		assert(queue_t::size() == 0);
	}

	// Blocked functions:
	value_t &back() = delete; // prevent moving and in-place modification
	const value_t &emplace_back(value_t &&) = delete; // prefer emplace
	value_t &front() = delete; // prevent moving and in-place modification
	void push_back(const value_t &) = delete; // prefer push
	void push_back(value_t &&) = delete;      // prefer push
	std::size_t size() = delete;              // prefer length() over size()
	void swap(fifo &) = delete;               // prevent length changes

private:
	// pop the oldest item if the fifo is full
	void pop_if_full() noexcept
	{
		if (queue_t::size() == max_length)
			queue_t::pop();
	}
	// default value to return when reading an empty fifo
	constexpr static value_t empty_value = {};
};

#endif

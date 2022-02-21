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

#include "fifo.h"

#include <gtest/gtest.h>

namespace {

TEST(fifo, length_of_1)
{
	fifo<int, 1> f;

	// Reading an empty queue isn't fatal and returns 0
	EXPECT_EQ(f.length(), 0);
	EXPECT_EQ(f.first(), 0);
	EXPECT_EQ(f.last(), 0);

	// Single item
	f.push(1);
	EXPECT_EQ(f.length(), 1);
	EXPECT_EQ(f.first(), 1);
	EXPECT_EQ(f.last(), 1);

	// Push another and read
	f.push(2);
	EXPECT_EQ(f.length(), 1);
	EXPECT_EQ(f.first(), 2);
	EXPECT_EQ(f.last(), 2);

	// Pop
	f.pop();
	EXPECT_EQ(f.length(), 0);
	EXPECT_EQ(f.first(), 0);
	EXPECT_EQ(f.last(), 0);
}

TEST(fifo, length_of_3)
{
	fifo<int, 3> f;

	// Push 5 items and read each
	f.push(1);
	EXPECT_EQ(f.length(), 1);
	EXPECT_EQ(f.first(), 1);
	EXPECT_EQ(f.last(), 1);

	f.push(2);
	EXPECT_EQ(f.length(), 2);
	EXPECT_EQ(f.first(), 1);
	EXPECT_EQ(f.last(), 2);

	f.push(3); // fifo is full
	EXPECT_EQ(f.length(), 3);
	EXPECT_EQ(f.first(), 1);
	EXPECT_EQ(f.last(), 3);

	f.push(4); // bumps 1 out
	EXPECT_EQ(f.length(), 3);
	EXPECT_EQ(f.first(), 2);
	EXPECT_EQ(f.last(), 4);

	f.push(5); // bumps 2 out
	EXPECT_EQ(f.length(), 3);
	EXPECT_EQ(f.first(), 3);
	EXPECT_EQ(f.last(), 5);

	f.pop(); // bumps 3 out
	EXPECT_EQ(f.length(), 2);
	EXPECT_EQ(f.first(), 4);
	EXPECT_EQ(f.last(), 5);

	f.pop(); // bumps 4 out
	EXPECT_EQ(f.length(), 1);
	EXPECT_EQ(f.first(), 5);
	EXPECT_EQ(f.last(), 5);

	f.pop(); // bumps 5 out
	EXPECT_EQ(f.length(), 0);
	EXPECT_EQ(f.first(), 0);
	EXPECT_EQ(f.last(), 0);

	f.pop(); // can we pop even more?
	EXPECT_EQ(f.length(), 0);
	EXPECT_EQ(f.first(), 0);
	EXPECT_EQ(f.last(), 0);
}

} // namespace

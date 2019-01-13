/**
 *  Board management controller (BMC) for a battery powered RPi computer
 *  Copyright (C) 2019  Andreas Stöckel
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <foxen/unittest.h>
#include <bmc/led.hpp>

using namespace rpibmc;

void test_paused() {
	LED led;
	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(0, led.brightness());
	led.brightness(2);
	EXPECT_EQ(2, led.brightness());
	led.step();
	EXPECT_EQ(2, led.brightness());
	for (int i = 0; i < 512; i++) {
		led.step();
	}
	EXPECT_EQ(2, led.brightness());
}

void test_ctrl_simple() {
	LED led;
	led[0] = LED::Ctrl::set(4, 64);
	led[1] = LED::Ctrl::set(8, 30, true);

	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(0, led.brightness());

	led.run();
	ASSERT_EQ(64, led.brightness());

	for (uint8_t i = 0; i < 128; i++) {
		for (uint8_t i = 0; i < 4; i++) {
			led.step();
			ASSERT_TRUE(led.is_running());
			ASSERT_EQ(64, led.brightness());
		}

		for (uint8_t i = 0; i < 8; i++) {
			led.step();
			ASSERT_TRUE(led.is_running());
			ASSERT_EQ(30, led.brightness());
		}
	}
}

void test_ctrl_wrap() {
	LED led;
	led[0] = LED::Ctrl::set(4, 64);
	led[1] = LED::Ctrl::set(8, 30);
	led[2] = LED::Ctrl::set(4, 64);
	led[3] = LED::Ctrl::set(8, 30);
	led[4] = LED::Ctrl::set(4, 64);
	led[5] = LED::Ctrl::set(8, 30);
	led[6] = LED::Ctrl::set(4, 64);
	led[7] = LED::Ctrl::set(8, 30, true);

	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(0, led.brightness());

	led.run();

	for (uint8_t i = 0; i < 128; i++) {
		for (uint8_t i = 0; i < 4; i++) {
			led.step();
			ASSERT_TRUE(led.is_running());
			ASSERT_EQ(64, led.brightness());
		}

		for (uint8_t i = 0; i < 8; i++) {
			led.step();
			ASSERT_TRUE(led.is_running());
			ASSERT_EQ(30, led.brightness());
		}
	}
}

void test_ctrl_ramp() {
	LED led;
	led[0] = LED::Ctrl::ramp(3, 128);
	led[1] = LED::Ctrl::ramp(5, 2);
	led[2] = LED::Ctrl::set(4, 12, true);

	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(0, led.brightness());

	led.run();

	EXPECT_EQ(0, led.brightness());

	for (uint8_t i = 0; i < 43; i++) {
		ASSERT_GT(128, led.brightness());
		led.step();
	}
	EXPECT_EQ(128, led.brightness());

	for (uint8_t i = 0; i < 128; i++) {
		for (uint8_t i = 0; i < 26; i++) {
			ASSERT_LT(2, led.brightness());
			led.step();
			ASSERT_GT(128, led.brightness());
		}
		EXPECT_EQ(2, led.brightness());

		for (uint8_t i = 0; i < 4; i++) {
			led.step();
			ASSERT_EQ(12, led.brightness());
		}

		for (uint8_t i = 0; i < 39; i++) {
			ASSERT_GT(128, led.brightness());
			led.step();
			ASSERT_LT(12, led.brightness());
		}
		EXPECT_EQ(128, led.brightness());
	}
}

void test_ctrl_ramp_saturated() {
	LED led;
	led[0] = LED::Ctrl::ramp(3, 254);
	led[1] = LED::Ctrl::ramp(5, 0, true);

	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(0, led.brightness());

	led.run();

	EXPECT_EQ(0, led.brightness());

	for (uint8_t i = 0; i < 128; i++) {

		for (uint8_t i = 0; i < 85; i++) {
			ASSERT_GT(254, led.brightness());
			led.step();
		}
		ASSERT_EQ(254, led.brightness());

		for (uint8_t i = 0; i < 51; i++) {
			ASSERT_LT(0, led.brightness());
			led.step();
		}
		ASSERT_EQ(0, led.brightness());
	}
}


int main() {
	RUN(test_paused);
	RUN(test_ctrl_simple);
	RUN(test_ctrl_wrap);
	RUN(test_ctrl_ramp);
	RUN(test_ctrl_ramp_saturated);
}

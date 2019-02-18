/**
 *  Aravir -- Linux compatiable virtual I2C devices -- LED Controller
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

#include <foxen/unittest.h>
#include <aravir/leds.hpp>

using namespace aravir::I2C;

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
	led[1] = LED::Ctrl::set(8, 31, true);

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
			ASSERT_EQ(31, led.brightness());
		}
	}
}

void test_ctrl_wrap() {
	LED led;
	led[0] = LED::Ctrl::set(4, 64);
	led[1] = LED::Ctrl::set(8, 31);
	led[2] = LED::Ctrl::set(4, 64);
	led[3] = LED::Ctrl::set(8, 31);
	led[4] = LED::Ctrl::set(4, 64);
	led[5] = LED::Ctrl::set(8, 31);
	led[6] = LED::Ctrl::set(4, 64);
	led[7] = LED::Ctrl::set(8, 31, true);

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
			ASSERT_EQ(31, led.brightness());
		}
	}
}

void test_ctrl_ramp() {
	LED led;
	led[0] = LED::Ctrl::ramp(3, 128);
	led[1] = LED::Ctrl::ramp(5, 3);
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
		for (uint8_t i = 0; i < 25; i++) {
			ASSERT_LT(3, led.brightness());
			led.step();
			ASSERT_GT(128, led.brightness());
		}
		EXPECT_EQ(3, led.brightness());

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
	led[0] = LED::Ctrl::ramp(3, 255);
	led[1] = LED::Ctrl::ramp(5, 0, true);

	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(0, led.brightness());

	led.run();

	EXPECT_EQ(0, led.brightness());

	for (uint8_t i = 0; i < 128; i++) {

		for (uint8_t i = 0; i < 85; i++) {
			ASSERT_GT(255, led.brightness());
			led.step();
		}
		ASSERT_EQ(255, led.brightness());

		for (uint8_t i = 0; i < 51; i++) {
			ASSERT_LT(0, led.brightness());
			led.step();
		}
		ASSERT_EQ(0, led.brightness());
	}
}


void test_blink() {
	LED led;
	led.brightness(63);
	led.i2c_write(LED::REG_STATUS, LED::BIT_STATUS_BLINK);
	led[0] = LED::Ctrl::set(4, 0xFF);
	led[1] = LED::Ctrl::set(8, 0x00, true);

	EXPECT_FALSE(led.is_running());
	EXPECT_EQ(63, led.brightness());

	led.run();
	ASSERT_EQ(63, led.brightness());

	for (uint8_t i = 0; i < 128; i++) {
		for (uint8_t i = 0; i < 4; i++) {
			led.step();
			ASSERT_TRUE(led.is_running());
			ASSERT_EQ(63, led.brightness());
		}

		for (uint8_t i = 0; i < 8; i++) {
			led.step();
			ASSERT_TRUE(led.is_running());
			ASSERT_EQ(0, led.brightness());
		}
	}
}

template<uint8_t N_LEDS>
void test_multi_led() {
	MultiLED<N_LEDS> leds;
	EXPECT_EQ(N_LEDS, leds.i2c_read(leds.REG_N_LEDS));
	EXPECT_EQ(LED::N_REGS, leds.i2c_read(leds.REG_LED_SIZE));
	EXPECT_EQ(64, leds.i2c_read(leds.REG_TICKS_PER_SECOND));
	uint8_t addr = leds.ADDR_BASE;
	for (uint8_t i = 0; i < leds.size(); i++) {
		for (uint8_t j = 0; j < LED::N_REGS; j++) {
			ASSERT_EQ(i, leds.i2c_addr_to_led_idx(addr));
			addr++;
		}
	}
}

int main() {
	RUN(test_paused);
	RUN(test_ctrl_simple);
	RUN(test_ctrl_wrap);
	RUN(test_ctrl_ramp);
	RUN(test_ctrl_ramp_saturated);
	RUN(test_blink);
	RUN(test_multi_led<0>);
	RUN(test_multi_led<1>);
	RUN(test_multi_led<2>);
	RUN(test_multi_led<3>);
	RUN(test_multi_led<8>);
}

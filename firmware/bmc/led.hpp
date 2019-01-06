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

/**
 * @file led.hpp
 *
 * Implements the programmable LED statemachine. Allows LEDs to blink or fade
 * according to a sequence of control words (instructions).
 *
 * @author Andreas Stöckel
 */

class rpikb {
/**
 * Con
 */
class LED {
private:
	/**
	 *
	 */
	struct Ctrl {
		static constexpr uint8_t MASK_DELAY = 0x7F;
		static constexpr uint8_t MASK_BRIGHTNESS = 0x7F;
		static constexpr uint8_t DELAY_IS_RAMP = 0x80;
		static constexpr uint8_t RESET_SEQ = 0x80;

		uint8_t delay;
		uint8_t brightness;
	};

	/**
	 * Number of control words, must be a power of two.
	 */
	static constexpr uint8_t NUM_CTRL_WORDS = 8;

	union Registers {
		Ctrl ctrl[NUM_CTRL_WORDS];
		uint8_t mem[NUM_CTRL_WORDS * sizeof(Ctrl)];
	};

	Registers m_regs;

	uint8_t m_ctrl_ptr;

	uint8_t m_delay;

	uint8_t m_brightness;

	static uint8_t sub_saturated(uint8_t a, uint8_t b) {
		return (b > a) ? 0 : (a - b);
	}

	static uint8_t delta(uint8_t a, uint8_t b) {
		return (a > b) ? (a - b) : (b - a);
	}

public:
	/**
	 * Create and initialises a new LED instance. Creates a control sequence
	 * that just clamps the LED brightness to zero.
	 */
	LED() : m_ctrl_ptr(0), m_delay(0), m_brightness(0) {
		m_regs.ctrl[0].delay = 0x7F; // Max delay
		m_regs.ctrl[0].brightness = RESET_SEQ; // Zero brightness, reset
	}

	/**
	 * Advances the LED state machine and computes the next value.
	 */
	void step() {
		// If we're at the end of the current control word, go to the next
		// control word
		if (m_delay == 0) {
			// Decide whether to go to the next control word or just reset to
			// the beginning of the sequence
			const Ctrl &c_old = m_regs.ctrl[m_ctrl_ptr];
			if (c_old.brightness & RESET_SEQ) {
				m_ctrl_ptr = 0;
			} else {
				m_ctrl_ptr = (m_ctrl_ptr + 1) & (NUM_CTRL_WORDS - 1);
			}

			// Load m_delay depending on whether the control word indicates a
			// ramp or a delay
			const Ctrl &c_new = m_regs.ctrl[m_ctrl_ptr];
			if (c_new.delay & DELAY_IS_RAMP) {
				m_delay = delta(c.brightness & MASK_BRIGHTNESS, m_brightness);
			} else {
				m_delay= c.delay & MASK_DELAY;
			}
		}

		// Either do a ramp or just wait
		const Ctrl &c = m_regs.ctrl[m_ctrl_ptr];
		if (c.delay & DELAY_IS_RAMP) {
			// Compute the offset from the target value (stored in m_delay)
			m_delay = sub_saturated(m_delay, c.delay & MASK_DELAY);

			// Fetch the target value and apply the offset
			const uint8_t tar = c.brightness & MASK_BRIGHTNESS;
			if (tar > m_brightness) {
				m_brightness = tar - m_delay;
			} else {
				m_brightness = tar + m_delay;
			}
		} else {
			m_delay--;
		}
	}

	/**
	 * Returns the current LED brightness value.
	 */
	uint8_t brightness() const {
		return m_brightness;
	}

	/**
	 * Sets the brightness value to the given value.
	 */
	void brightness(uint8_t value) const {
		m_brightness = value;
	}
};

};

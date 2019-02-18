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

/**
 * @file leds.hpp
 *
 * Implements a programmable LED controller statemachine. Allows LEDs to blink
 * or fade according to a programmable sequence.
 *
 * @author Andreas Stöckel
 */

#pragma once

#ifndef ARAVIR_LEDS_HPP
#define ARAVIR_LEDS_HPP

#include <stddef.h>
#include <stdint.h>

namespace aravir {
namespace I2C {
#pragma pack(push, 1)
/**
 * LED controller state machine. The state machine can either be programmed
 * using I2C or using the provided function calls.
 */
class LED {
public:
	/**
	 * Mask that must be used to extract the actual delay bits from the "delay"
	 * field of the Ctrl structure.
	 */
	static constexpr uint8_t MASK_CTRL_DELAY = 0x7F;

	/**
	 * Mask that must be used to extract the brightness bits from the
	 * "brightness" field of the Ctrl structure.
	 */
	static constexpr uint8_t MASK_CTRL_BRIGHTNESS = 0x7F;

	/**
	 * Bit mask that can be used to extract the current execution pointer from
	 * the status register.
	 */
	static constexpr uint8_t MASK_STATUS_PTR = 0x07;

	/**
	 * Used to extract the "IS_RAMP" bit encoded in the Ctrl::delay field. If
	 * set, the delay value is treated as the speed of a ramp from the current
	 * brightness value to the specified brightness value. A ramp speed of one
	 * means that the brightness will linearly cover the 8-bit space within
	 * 256 calls to the ""
	 */
	static constexpr uint8_t BIT_CTRL_DELAY_IS_RAMP = 0x80;

	/**
	 * Used to extract the "RESET_SEQ" bit encoded in the Ctrl::brightness
	 * field. If set, the instruction processor returns to the first instruction
	 * after this instruction has been processed.
	 */
	static constexpr uint8_t BIT_CTRL_RESET_SEQ = 0x80;

	/**
	 * If set in the "status" register, the LED controller will continue to
	 * execute the current instruction chain. Otherwise the LED controller will
	 * pause.
	 */
	static constexpr uint8_t BIT_STATUS_RUN = 0x80;

	/**
	 * If set in the "status" register, the control word sequence will not
	 * directly modify the brightness register, but instead write to the "mask"
	 * register which will be combined via "AND" with the brightness register
	 * when setting the LED brightness. This facilitates implementation of
	 * blinking in the Linux driver model.
	 *
	 * From an implementation perspective, this bit determined whether the
	 * "pattern_buf" register will be transfered to the "mask" or the
	 * "brightness" register.
	 */
	static constexpr uint8_t BIT_STATUS_BLINK = 0x40;

	/**
	 * Number of control words, must be a power of two.
	 */
	static constexpr uint8_t N_CTRL_WORDS = 8;

	/**
	 * Control word. For convenience, you may use the two static functions
	 * delay() and ramp() to create a new Ctrl instance.
	 */
	struct Ctrl {
		/**
		 * This field either encodes the delay until the next control word is
		 * executed, or a ramp speed if CTRL_DELAY_IS_RAMP is set.
		 */
		uint8_t delay;

		/**
		 * Encodes the brightness value the LED should be set to/ramp towards.
		 * If the CTRL_RESET_SEQ bit is set, execution will continue with the
		 * first control word. Otherwise the next control word will be executed.
		 * Executed wraps around to the first control value.
		 */
		uint8_t brightness;

		/**
		 * Creates a new "set" control word.
		 *
		 * @param delay is the number of calls to tick() until the next
		 * instruction is executed. May be a value between 0 and 127.
		 * @param brightness is the brightness the LED should be set to during
		 * this period. May be a value between 0 and 255; however the resolution
		 * will be reduced to 7bit.
		 * @param eos if true, indicates that this is the end of the sequence.
		 */
		static constexpr Ctrl set(uint8_t delay, uint8_t brightness,
		                          bool eos = false)
		{
			return Ctrl{
			    uint8_t((delay & MASK_CTRL_DELAY)),
			    uint8_t(((brightness >> 1U) & MASK_CTRL_BRIGHTNESS) |
			            (eos ? BIT_CTRL_RESET_SEQ : (uint8_t)0U)),
			};
		}

		/**
		 * Creates a new "ramp" control word.
		 *
		 * @param speed is the linear change in brighness each tick. The
		 * brightness is interpolated in 8bit space.
		 * @param target is the brightness the LED should ramp up/down to. The
		 * next instruction is executed once the target brightness is reached.
		 * @param eos if true, indicates that this is the end of the sequence.
		 */
		static constexpr Ctrl ramp(uint8_t speed, uint8_t target,
		                           bool eos = false)
		{
			return Ctrl{
			    uint8_t((speed & MASK_CTRL_DELAY) | BIT_CTRL_DELAY_IS_RAMP),
			    uint8_t(((target >> 1U) & MASK_CTRL_BRIGHTNESS) |
			            (eos ? BIT_CTRL_RESET_SEQ : (uint8_t)0U)),
			};
		}
	};

private:
	/**************************************************************************
	 * Static helper functions                                                *
	 **************************************************************************/

	/**
	 * Computes max{0, a - b}.
	 */
	static uint8_t sub_saturated(uint8_t a, uint8_t b)
	{
		return (b > a) ? 0 : (a - b);
	}

	/**
	 * Computes | a - b |.
	 */
	static uint8_t delta(uint8_t a, uint8_t b)
	{
		return (a > b) ? (a - b) : (b - a);
	}

	/**************************************************************************
	 * Internal datastructures                                                *
	 **************************************************************************/

	/**
	 * Structure holding all the virtual register values of the LED device.
	 */
	struct Registers {
		/**
		 * Encodes the status of the LED controller. The status contains the
		 * STATUS_RUN bit as well as the execution pointer. When setting the
		 * "run" bit from zero to one when writing to this register, the
		 * specified instruction will be re-loaded.
		 */
		uint8_t status;

		/**
		 * The current brightness of the LED as a value between 0 and 255. Note
		 * that the control words only support 7 bit resolution. Writing to this
		 * value will directly affect the LED brightness. However, this will be
		 * overriden if the LED controller is not paused at the moment and the
		 * BIT_STATUS_BLINK flag is not set.
		 */
		uint8_t brightness;

		/**
		 * This mask will be combined via "AND" with the status brightness
		 * value when reading out the brightness. This is initialized to 0xFF.
		 */
		uint8_t mask;

		/**
		 * Stores the current phase of the instruction decoder. This value is
		 * decreasing with each call to step(). Whenever this value hits
		 */
		uint8_t phase;

		/**
		 * Program memory defining the LED blink pattern.
		 */
		Ctrl ctrl[N_CTRL_WORDS];
	};

	/**
	 * Union holding all registers, as well as a byte-wise representation of the
	 * register contents.
	 */
	union {
		Registers regs;
		uint8_t mem[sizeof(Registers)];
	} m_regs;

	/**
	 * The pattern buffer will always write the current target brightness to
	 * this register. If BIT_STATUS_BLINK is not set, it will be transfered
	 * to the brightness register. If BIT_STATUS_BLINK is set, it will be
	 * transfered to the "mask" register. This register is not exposed via I2C.
	 */
	uint8_t m_pattern_buf;

	/**************************************************************************
	 * Helper functions                                                       *
	 **************************************************************************/

	/**
	 * Returns the current instruction pointer.
	 */
	uint8_t ptr() const { return m_regs.regs.status & MASK_STATUS_PTR; }

	/**
	 * Sets the current instruction pointer to zero.
	 */
	void ptr_zero() { m_regs.regs.status &= ~MASK_STATUS_PTR; }

	/**
	 * Increments the instruction pointer. Wraps around if the end of the buffer
	 * is reached.
	 */
	void ptr_incr()
	{
		m_regs.regs.status =
		    (m_regs.regs.status + 1) & (~(MASK_STATUS_PTR + 1U));
	}

	/**
	 * Unpacks a seven bit brightness value into an 8-bit brightness value.
	 * Ensures that "x == 0" is mapped onto zero and "x = 0x7F" is mapped onto
	 * "0xFF", i.e. the entire dynamic range is used.
	 */
	static constexpr uint8_t unpack_7bit_brightness(uint8_t x)
	{
		return ((x & MASK_CTRL_BRIGHTNESS) << 1) | (x & 0x01);
	}

	/**
	 * Returns a reference at the current control word.
	 */
	Ctrl &ctrl() { return m_regs.regs.ctrl[ptr()]; }

	/**
	 * Transfers the pattern buffer to the output.
	 */
	void transfer() {
		Registers &r = m_regs.regs;

		// Transfer the pattern buffer to either the "mask" or the "brightness"
		// register
		if (r.status & BIT_STATUS_BLINK) {
			r.mask = m_pattern_buf;
		} else {
			r.brightness = m_pattern_buf;
		}
	}

	/**
	 * Goes to the next control word; if BIT_CTRL_RESET_SEQ is set, goes back to
	 * the beginning of the sequence, otherwise advances the instruction
	 * pointer. Wraps around once the end of the instruction buffer is reached.
	 */
	void next()
	{
		if (ctrl().brightness & BIT_CTRL_RESET_SEQ) {
			ptr_zero();
		}
		else {
			ptr_incr();
		}
	}

	/**
	 * Loads the current instruction into memory. In particular, loads "phase"
	 * depending on whether the control word indicates a ramp or a delay, as
	 * well as "brightness" if the instruction encodes a delay.
	 */
	void load()
	{
		// Helper variables
		const Ctrl &c = ctrl();
		Registers &r = m_regs.regs;

		// Expand the encoded brightness value to 8 bit
		const uint8_t brightness = unpack_7bit_brightness(c.brightness);

		if (c.delay & BIT_CTRL_DELAY_IS_RAMP) {
			// In ramp mode, phase encodes the difference between the current
			// brightness value and the target brightness
			r.phase = delta(brightness, m_pattern_buf);
		}
		else {
			// In delay mode, phase encodes the remaining delay, brightness
			// is constant
			r.phase = c.delay & MASK_CTRL_DELAY;
			m_pattern_buf = brightness;
		}

		// Transfer the pattern_buf to the output registers
		transfer();
	}

public:
	/**
	 * Relative address of the status register.
	 */
	static constexpr uint8_t REG_STATUS = offsetof(Registers, status);

	/**
	 * Relative address of the brightness register.
	 */
	static constexpr uint8_t REG_BRIGHTNESS = offsetof(Registers, brightness);

	/**
	 * Relative address of the mask register.
	 */
	static constexpr uint8_t REG_MASK = offsetof(Registers, mask);

	/**
	 * Relative address of the phase register.
	 */
	static constexpr uint8_t REG_PHASE = offsetof(Registers, phase);

	/**
	 * Total number of register bytes.
	 */
	static constexpr uint8_t N_REGS = sizeof(Registers);

	/**
	 * Create and initialises a new LED instance. Initially the controller is
	 * not running, the phase is zero, and the brightness is set to zero.
	 */
	LED()
	{
		m_regs.regs.status = 0;
		m_regs.regs.brightness = 0;
		m_regs.regs.mask = 0xFF;
		m_regs.regs.phase = 0;
		m_pattern_buf = 0;
	}

	/**
	 * Advances the LED state machine and computes the next value.
	 */
	void step()
	{
		// Helper variables
		Registers &r = m_regs.regs;

		// Do nothing if the state machine is not active
		if (!is_running()) {
			return;
		}

		// If we're at the end of the current control word, go to the next
		// control word
		if (r.phase == 0) {
			// Advance the instruction pointer
			next();

			// Load the next instruction
			load();
		}

		// Either do a ramp or just wait
		const Ctrl &c = ctrl();
		if (c.delay & BIT_CTRL_DELAY_IS_RAMP) {
			// Compute the offset from the target value (stored in "phase")
			r.phase = sub_saturated(r.phase, c.delay & MASK_CTRL_DELAY);

			// Fetch the target value and apply the offset
			const uint8_t tar = unpack_7bit_brightness(c.brightness);
			if (tar > m_pattern_buf) {
				m_pattern_buf = tar - r.phase;
			}
			else {
				m_pattern_buf = tar + r.phase;
			}
		}
		else {
			r.phase--;
		}

		// Write the pattern_buf register to the output
		transfer();
	}

	/**
	 * Returns the current LED brightness value.
	 */
	uint8_t brightness() const
	{
		const Registers &r = m_regs.regs;
		return r.brightness & r.mask;
	}

	/**
	 * Sets the brightness value to the given value.
	 */
	void brightness(uint8_t value) { m_regs.regs.brightness = value; }

	/**
	 * Returns true if the LED controller is active at the moment.
	 */
	bool is_running() const { return m_regs.regs.status & BIT_STATUS_RUN; }

	/**
	 * Return a reference at the given control register index. The number of
	 * control words is stored in the constant N_CTRL_WORDS.
	 */
	Ctrl &operator[](uint8_t i) { return m_regs.regs.ctrl[i]; }

	/**
	 * Number of control words.
	 */
	static constexpr uint8_t size() { return N_CTRL_WORDS; }

	/**
	 * Transitions the controller to the "running" state. In particular, this
	 * function will reload the current instruction. Does nothing if the
	 * controller is already running.
	 */
	void run()
	{
		// Do nothing if we are already running
		if (is_running()) {
			return;
		}

		// Load the current instruction
		load();

		// Set the "running" bit
		m_regs.regs.status |= BIT_STATUS_RUN;
	}

	/**
	 * Stops the controller.
	 */
	void stop() { m_regs.regs.status &= ~BIT_STATUS_RUN; }

	/**************************************************************************
	 * I2C Interface                                                          *
	 **************************************************************************/

	/**
	 * Reads the byte stored at the given address.
	 *
	 * @param addr is the address that should be read. Returns zero if the
	 * address is out of bounds.
	 */
	uint8_t i2c_read(uint8_t addr) const
	{
		// Make sure the read is not out of bounds
		if (addr >= sizeof(Registers)) {
			return 0U;
		}

		// Return the memory content at the given address
		return m_regs.mem[addr];
	}

	/**
	 * Writes to the given address. Note that setting the "run" bit in the
	 * status register from zero to one will re-load the current instruction.
	 */
	void i2c_write(uint8_t addr, uint8_t value)
	{
		switch (addr) {
			case REG_STATUS:
				if ((value & BIT_STATUS_RUN)) {
					run();
				}
				m_regs.mem[addr] = value & (BIT_STATUS_RUN | BIT_STATUS_BLINK | MASK_STATUS_PTR);
				break;
			default:
				if (addr < sizeof(Registers)) {
					m_regs.mem[addr] = value;
				}
				break;
		}
	}

	/**
	 * Returns the next I2C address.
	 */
	uint8_t i2c_next_addr(uint8_t addr)
	{
		addr++;
		if (addr >= sizeof(Registers)) {
			addr = 0;
		}
		return addr;
	}
};

/**
 * A set of multiple LED instances that act as a single I2C device.
 */
template <uint8_t N_LEDS, uint8_t N_TICKS_PER_SECOND = 64>
class MultiLED {
private:
	LED m_leds[N_LEDS];

public:
	/**************************************************************************
	 * Constants                                                              *
	 **************************************************************************/

	/**
	 * Read-only register containing the number of LEDs.
	 */
	static constexpr uint8_t REG_N_LEDS = 0;

	/**
	 * Read-only register returning the number of registers per LED.
	 */
	static constexpr uint8_t REG_LED_SIZE = 1;

	/**
	 * Number of ticks per second.
	 */
	static constexpr uint8_t REG_TICKS_PER_SECOND = 2;

	/**
	 * Address offset of the first LED.
	 */
	static constexpr uint8_t ADDR_BASE = 3;

	/**
	 * Maximum address.
	 */
	static constexpr uint8_t N_REGS = ADDR_BASE + LED::N_REGS * N_LEDS;

	/**
	 * Address offset of the LED with the specified index.
	 */
	static constexpr uint8_t led_addr_base(uint8_t led_idx)
	{
		return ADDR_BASE + LED::N_REGS * led_idx;
	}

	/**************************************************************************
	 * MultiLED specific functionality                                        *
	 **************************************************************************/

	/**
	 * Returns the current ticks_per_second as they are reported per I2C.
	 */
	static constexpr uint8_t ticks_per_second() { return N_TICKS_PER_SECOND; }

	/**
	 * Returns a const reference at the i-th LED.
	 */
	const LED &operator[](uint8_t i) const { return m_leds[i]; }

	/**
	 * Returns a reference at the i-th LED.
	 */
	LED &operator[](uint8_t i) { return m_leds[i]; }

	/**
	 * Returns the number of LEDs represented by the MultiLED instance.
	 */
	static constexpr uint8_t size() { return N_LEDS; }

	/**************************************************************************
	 * LED interface                                                          *
	 **************************************************************************/

	/**
	 * Forwards the state of all leds in the multi LED group.
	 */
	void step()
	{
		for (uint8_t i = 0; i < N_LEDS; i++) {
			m_leds[i].step();
		}
	}

	/**
	 * Executes the state machine for all LEDs.
	 */
	void run()
	{
		for (uint8_t i = 0; i < N_LEDS; i++) {
			m_leds[i].run();
		}
	}

	/**
	 * Stops the state machine for all LEDs.
	 */
	void stop()
	{
		for (uint8_t i = 0; i < N_LEDS; i++) {
			m_leds[i].stop();
		}
	}

	/**************************************************************************
	 * I2C Interface                                                          *
	 **************************************************************************/

	/**
	 * Converts an i2c address to an LED index without division by using binary
	 * search. This should just be compiled into a tree of if/then/else
	 * statements. The given address must be greater or equal to ADDR_BASE and
	 * smaller than N_REGS.
	 */
	template <uint8_t MIN_LED = 0, uint8_t MAX_LED = N_LEDS>
	static constexpr uint8_t i2c_addr_to_led_idx(uint8_t addr)
	{
		constexpr uint8_t MID_LED = MIN_LED + (MAX_LED - MIN_LED) / 2;
		if (MIN_LED > 0U && MIN_LED == MAX_LED) {
			return uint8_t(MIN_LED - 1U); /* Always >= 0 */
		}
		else if (addr - ADDR_BASE < MID_LED * LED::N_REGS) {
			return i2c_addr_to_led_idx<MIN_LED, MID_LED>(addr);
		}
		else {
			return i2c_addr_to_led_idx<MID_LED + 1U, MAX_LED>(addr);
		}
	}

	/**
	 * Reads the byte stored at the given address.
	 */
	uint8_t i2c_read(uint8_t addr) const
	{
		if (addr == REG_N_LEDS) {
			return N_LEDS;
		}
		else if (addr == REG_LED_SIZE) {
			return LED::N_REGS;
		}
		else if (addr == REG_TICKS_PER_SECOND) {
			return N_TICKS_PER_SECOND;
		}
		else if (addr >= ADDR_BASE && addr < N_REGS) {
			const uint8_t led_idx = i2c_addr_to_led_idx(addr);
			return m_leds[led_idx].i2c_read(addr - led_addr_base(led_idx));
		}
		else {
			return 0;
		}
	}

	/**
	 * Writes to the given address.
	 */
	void i2c_write(uint8_t addr, uint8_t value)
	{
		if (addr >= ADDR_BASE && addr < N_REGS) {
			const uint8_t led_idx = i2c_addr_to_led_idx(addr);
			m_leds[led_idx].i2c_write(addr - led_addr_base(led_idx), value);
		}
	}

	/**
	 * Returns the next I2C address.
	 */
	uint8_t i2c_next_addr(uint8_t addr)
	{
		addr++;
		if (addr >= N_REGS) {
			addr = 0;
		}
		return addr;
	}
};

};  // namespace I2C
};  // namespace aravir
#pragma pack(pop)
#endif /* ARAVIR_LEDS_HPP */

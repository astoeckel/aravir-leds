// SPDX-License-Identifier: GPL-2.0-or-later

/**
 * Driver for the LED controller on the RPi Board Management Controller
 * Copyright (C) 2019  Andreas Stöckel <andreas.stoeckel@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

/* This code is loosely based on the lp3944 I2C driver */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/regmap.h>

/******************************************************************************
 * CONSTANTS AND DATASTRUCTURES                                               *
 ******************************************************************************/

#define RPIBMC_LEDS_REG_NUM_LEDS 0x00 /* Number of LEDS (RO) */
#define RPIBMC_LEDS_REG_NUM_BYTES 0x01 /* Bytes per LED (RO) */
#define RPIBMC_LEDS_REG_TICKS_PER_SECOND 0x02 /* Ticks per second (RO) */
#define RPIBMC_LEDS_REG_LED_BASE 0x03 /* Offset address for the first LED */

#define RPIBMC_LEDS_REG_LED_STATUS 0x00 /* LED status register */
#define RPIBMC_LEDS_REG_LED_BRIGHTNESS 0x01 /* Current brightness register */
#define RPIBMC_LEDS_REG_LED_MASK 0x02 /* Brightness mask register */
#define RPIBMC_LEDS_REG_LED_PHASE 0x03 /* Current pattern phase */
#define RPIBMC_LEDS_REG_LED_CTRL_BASE 0x04 /* Control words base offset */

/* Maximum number of LEDS that can be managed by the RPIBMC */
#define RPIBMC_LEDS_MAX 8U

/* Maximum number of control words */
#define RPIBMC_LEDS_MAX_NUM_CTRL_WORDS 8U

/* Maximum number of bytes used by a single LED */
#define RPIBMC_LEDS_MAX_NUM_BYTES                                              \
	(RPIBMC_LEDS_REG_LED_CTRL_BASE + 2U * RPIBMC_LEDS_MAX_NUM_CTRL_WORDS)

#define RPIBMC_LEDS_STATUS_RUN 0x80
#define RPIBMC_LEDS_STATUS_BLINK 0x40

#define RPIBMC_LEDS_CTRL_MAX_DELAY 0x7F
#define RPIBMC_LEDS_CTRL_MAX_BRIGHTNESS 0x7F
#define RPIBMC_LEDS_CTRL_IS_RAMP 0x80
#define RPIBMC_LEDS_CTRL_RETURN 0x80
#define RPIBMC_LEDS_CTRL_MASK 0x7F

#define ldev_to_led(c) container_of(c, struct rpibmc_leds_led_data, ldev)

/* Control word registers */
struct rpibmc_leds_ctrl {
	u8 delay;
	u8 brightness;
};

/* Information about an individual LED */
struct rpibmc_leds_led_data {
	/* Parent classdev */
	struct led_classdev ldev;

	/* Register map used to access the device */
	struct regmap *regmap;

	/* Pointer at the I2C device TODO: Get rid of this? */
	struct i2c_client *client;

	/* Number of microseconds per LED tick */
	unsigned long us_per_tick;

	/* I2C register offset for this led */
	u8 addr;

	/* Number of control words available for blink patterns */
	u8 num_ctrl_words;
};

/* Information about all LEDs managed by the device */
struct rpibmc_leds_data {
	struct mutex lock;
	struct rpibmc_leds_led_data leds[RPIBMC_LEDS_MAX];
	u8 leds_size;
};

/* Possible LED names */
static const char *rpibmc_leds_names[RPIBMC_LEDS_MAX] = {
	"rpibmc:led:1", "rpibmc:led:2", "rpibmc:led:3", "rpibmc:led:4",
	"rpibmc:led:5", "rpibmc:led:6", "rpibmc:led:7", "rpibmc:led:8",
};

/******************************************************************************
 * I2C HELPER FUNCTIONS                                                       *
 ******************************************************************************/

static void rpibmc_leds_append_stream(u8 *regs, u8 *mask, int *ptr, u8 value,
				      u8 do_mask)
{
	if (do_mask) {
		regs[*ptr] = value;
		mask[*ptr] = 0;
	} else {
		mask[*ptr] = 1;
	}
	(*ptr)++;
}

/**
 * Updates a single RPIBMC LED. Setting one of the int values to a negative
 * value indicates that this particular register shouldn't be updated.
 */
static int rpibmc_leds_write(struct rpibmc_leds_led_data *led, int status_reg,
			     int brightness_reg, int mask_reg, int phase_reg,
			     struct rpibmc_leds_ctrl *ctrl, int ctrl_size)
{
	/* Fetch the controller data for access to the mutex lock */
	struct rpibmc_leds_data *data = i2c_get_clientdata(led->client);

	/* Temporary buffers containing the data that should be written to the I2C
	   device in as few transactions as possible. */
	u8 regs[RPIBMC_LEDS_MAX_NUM_BYTES];
	u8 mask[RPIBMC_LEDS_MAX_NUM_BYTES + 1];
	int i, offs = 0, len = 0, end = 0, err = 0;

	/* Assemble the masked stream */
	rpibmc_leds_append_stream(regs, mask, &end, status_reg, status_reg >= 0);
	rpibmc_leds_append_stream(regs, mask, &end, brightness_reg,
				  brightness_reg >= 0);
	rpibmc_leds_append_stream(regs, mask, &end, mask_reg, mask_reg >= 0);
	rpibmc_leds_append_stream(regs, mask, &end, phase_reg, phase_reg >= 0);
	for (i = 0; i < ctrl_size; i++) {
		rpibmc_leds_append_stream(regs, mask, &end, ctrl[i].delay, 1);
		rpibmc_leds_append_stream(regs, mask, &end, ctrl[i].brightness,
					  1);
	}

	/* Mark the last byte as masked/invalid. This ensures that the below loop
	   will exit. */
	mask[end] = 1;

	/* Prevent concurrent access to the I2C bus */
	mutex_lock(&data->lock);

	/* Send the registers to the client in as few transactions as possible */
	while (offs <= end) {
		/* If the given register value is masked, try to perform a transaction.
		   Limit the maximum transaction size to 32 for SMBus compatibility. */
		if (mask[offs + len] || (len == 32)) {
			if (len > 0) {
				err = regmap_bulk_write(led->regmap,
							led->addr + offs,
							&regs[offs], len);
				if (err != 0) {
					goto exit;
				}

				offs += len;
				len = 0;
			} else {
				/* Skip masked bytes */
				offs++;
			}
		} else {
			len++;
		}
	}

exit:
	mutex_unlock(&data->lock);
	return err;
}

/******************************************************************************
 * I2C I/O                                                                    *
 ******************************************************************************/

static int rpibmc_leds_set_brightness(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct rpibmc_leds_led_data *led = ldev_to_led(led_cdev);

	/* If the brightness is set to zero, reset the pattern engine */
	const int status = (brightness == LED_OFF) ? 0 : -1;
	const int mask = (brightness == LED_OFF) ? 0xFF : -1;

	/* Update the brightness register, don't touch anything else */
	return rpibmc_leds_write(led, status, brightness, mask, -1, NULL, -1);
}

static enum led_brightness
rpibmc_leds_get_brightness(struct led_classdev *led_cdev)
{
	struct rpibmc_leds_led_data *led = ldev_to_led(led_cdev);
	int err = 0, brightness = 0;

	err = regmap_read(led->regmap,
			  led->addr + RPIBMC_LEDS_REG_LED_BRIGHTNESS,
			  &brightness);
	if (err) {
		dev_err(&led->client->dev, "couldn't read brightness value\n");
	}

	return brightness;
}

static u8 rpibmc_leds_blink_ticks(unsigned long max_us, unsigned long us_p_t,
				  unsigned long *delay_us)
{
	unsigned long us = min(max_us, *delay_us);
	*delay_us -= us;
	return (us / us_p_t) & RPIBMC_LEDS_CTRL_MAX_DELAY;
}

static int rpibmc_leds_blink_set(struct led_classdev *led_cdev,
				 unsigned long *delay_on,
				 unsigned long *delay_off)
{
	struct rpibmc_leds_led_data *led = ldev_to_led(led_cdev);
	struct rpibmc_leds_ctrl ctrl[RPIBMC_LEDS_MAX_NUM_CTRL_WORDS];
	unsigned long d_on_rem, d_off_rem;
	int i;

	/* Make sure the given delay values are valid. */
	const unsigned long num_ctrl_words = led->num_ctrl_words;
	const unsigned long us_p_t = led->us_per_tick;
	const unsigned long max_us = us_p_t * RPIBMC_LEDS_CTRL_MAX_DELAY;
	const unsigned long max_delay = (max_us * num_ctrl_words);
	if ((*delay_on > max_delay) || (*delay_off > max_delay) ||
	    ((*delay_on + *delay_off) > max_delay)) {
		return -EINVAL;
	}

	/* Use a sensible default value in case delay_on and delay_off are both
	   zero. */
	if ((*delay_on == 0) && (*delay_off == 0)) {
		*delay_on = 500;
		*delay_off = 500;
	}

	/* Compile the control word. We're using the "blink" mode where the pattern
	   does not control the brightness directly. */
	d_on_rem = *delay_on * 1000; // ms to us
	d_off_rem = *delay_off * 1000;
	for (i = 0; i < (int)(num_ctrl_words); i++) {
		if (d_on_rem > led->us_per_tick) {
			ctrl[i].delay = rpibmc_leds_blink_ticks(max_us, us_p_t,
								&d_on_rem);
			ctrl[i].brightness = RPIBMC_LEDS_CTRL_MAX_BRIGHTNESS;
		} else if (d_off_rem > led->us_per_tick) {
			ctrl[i].delay = rpibmc_leds_blink_ticks(max_us, us_p_t,
								&d_off_rem);
			ctrl[i].brightness = 0x00;
		} else {
			break;
		}
	}

	/* Abort if the delay couldn't be written successfully */
	if ((i == 0) || (d_on_rem > us_p_t) || (d_off_rem > us_p_t)) {
		return -EINVAL;
	}

	/* Mark the last instruction as "return" instruction */
	ctrl[i - 1].brightness |= RPIBMC_LEDS_CTRL_RETURN;

	/* Tell the class driver the actual blink pattern duration that was used */
	*delay_on = *delay_on - d_on_rem / 1000;
	*delay_off = *delay_off - d_off_rem / 1000;

	/* Send the pattern to the LED */
	return rpibmc_leds_write(
		led, RPIBMC_LEDS_STATUS_RUN | RPIBMC_LEDS_STATUS_BLINK, -1, -1, -1,
		ctrl, i);
}

/******************************************************************************
 * DRIVER INITIALISATION AND FINALISATION                                     *
 ******************************************************************************/

static void rpibmc_leds_unregister_devices(struct rpibmc_leds_data *data)
{
	int i;
	for (i = 0; i < data->leds_size; i++) {
		led_classdev_unregister(&data->leds[i].ldev);
	}
	data->leds_size = 0;
}

static int rpibmc_leds_configure(struct i2c_client *client,
				 struct regmap *regmap,
				 struct rpibmc_leds_data *data)
{
	int i, err, num_leds, num_bytes, num_ctrl_words, tps, us_per_tick, addr;

	/* Read the number of leds from the attached device */
	err = regmap_read(regmap, RPIBMC_LEDS_REG_NUM_LEDS, &num_leds);
	if (err) {
		return err;
	}
	if (num_leds > RPIBMC_LEDS_MAX) {
		return -EINVAL;
	}

	/* Read the number of bytes per LED */
	err = regmap_read(regmap, RPIBMC_LEDS_REG_NUM_BYTES, &num_bytes);
	if (err) {
		return err;
	}
	if ((num_bytes < RPIBMC_LEDS_REG_LED_CTRL_BASE) ||
	    (num_bytes * num_leds + RPIBMC_LEDS_REG_LED_BASE > 255)) {
		return -EINVAL;
	}

	/* Compute the number of control words */
	num_ctrl_words = (num_bytes - RPIBMC_LEDS_REG_LED_CTRL_BASE) / 2;
	if (num_ctrl_words > RPIBMC_LEDS_MAX_NUM_CTRL_WORDS) {
		return -EINVAL;
	}

	/* Read the ticks per second*/
	err = regmap_read(regmap, RPIBMC_LEDS_REG_TICKS_PER_SECOND, &tps);
	if (err) {
		return err;
	}
	if (tps == 0) {
		return -EINVAL;
	}

	/* Compute the number of micro-seconds per tick */
	us_per_tick = (1000UL * 1000UL) / tps;

	/* Allocate the devices */
	addr = RPIBMC_LEDS_REG_LED_BASE;
	for (i = 0; i < num_leds; i++) {
		/* Populate the rpibmc_leds_led_data structure */
		struct rpibmc_leds_led_data *led = &data->leds[i];
		led->client = client;
		led->regmap = regmap;
		led->addr = addr;
		led->num_ctrl_words = num_ctrl_words;
		led->us_per_tick = us_per_tick;

		/* Populate the led_classdev structure */
		led->ldev.name = rpibmc_leds_names[i];
		led->ldev.brightness = 0;
		led->ldev.max_brightness = 255;
		led->ldev.brightness_set_blocking = rpibmc_leds_set_brightness;
		led->ldev.brightness_get = rpibmc_leds_get_brightness;
		led->ldev.blink_set = rpibmc_leds_blink_set;
		led->ldev.flags = LED_CORE_SUSPENDRESUME;

		/* Try to register the LED */
		err = led_classdev_register(&led->client->dev, &led->ldev);
		if (err < 0) {
			dev_err(&client->dev, "couldn't register LED %s\n",
				led->ldev.name);
			goto exit;
		}

		/* Increment the number of leds in the driver structure -- this
		   corresponds to the number of actually registered leds and is used
		   by rpibmc_leds_unregister_devices() */
		data->leds_size++;

		/* Compute the start address of the next LED */
		addr += num_bytes;
	}
	return 0;

exit:
	rpibmc_leds_unregister_devices(data);
	return err;
}

/**
 * Initializes all datastructures used by this driver and reads the number of
 * leds and address offsets from the device.
 */
static int rpibmc_leds_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct regmap *regmap;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0xFF,
	};
	struct rpibmc_leds_data *data;
	int err;

	/* Setup the regmap */
	regmap = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "%s: regmap allocation failed: %ld\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	/* Allocate memory for the rpibmc_leds_data struct */
	data = devm_kzalloc(&client->dev, sizeof(struct rpibmc_leds_data),
			    GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

	/* Initialize rpibmc_leds_data and associate the struct with the I2C
	   client. */
	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	/* Read information about the LEDs from the device */
	err = rpibmc_leds_configure(client, regmap, data);
	if (err < 0) {
		return err;
	}

	/* Add a log message informing about the successful initialisation */
	dev_info(&client->dev, "RPIBMC LEDS enabled\n");
	return 0;
}

/**
 * Removes the kernel module, unregisters all previously registered LED devices.
 */
static int rpibmc_leds_remove(struct i2c_client *client)
{
	/* Unregister all registered LED devices. */
	struct rpibmc_leds_data *data = i2c_get_clientdata(client);
	rpibmc_leds_unregister_devices(data);
	return 0;
}

/******************************************************************************
 * MODULE REGISTRATION                                                        *
 ******************************************************************************/

static const struct i2c_device_id rpibmc_leds_id[] = { { "rpibmc-leds", 0 },
						       {} };

MODULE_DEVICE_TABLE(i2c, rpibmc_leds_id);

static struct i2c_driver rpibmc_driver = {
    .driver =
        {
            .name = "rpibmc-leds",
        },
    .probe = rpibmc_leds_probe,
    .remove = rpibmc_leds_remove,
    .id_table = rpibmc_leds_id,
};

module_i2c_driver(rpibmc_driver);

MODULE_AUTHOR("Andreas Stoeckel <andreas.stoeckel@googlemail.com>");
MODULE_DESCRIPTION("RPIBMC LED Driver");
MODULE_LICENSE("GPL");

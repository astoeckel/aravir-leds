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
#include <linux/regmap.h>

/******************************************************************************
 * CONSTANTS AND DATASTRUCTURES                                               *
 ******************************************************************************/

#define RPIBMC_LEDS_REG_NUM_LEDS 0x00  /* Number of LEDS (Read Only) */
#define RPIBMC_LEDS_REG_NUM_BYTES 0x01 /* Bytes per LED (Read Only) */
#define RPIBMC_LEDS_REG_LED_BASE 0x02  /* Offset address for the first LED */

#define RPIBMC_LEDS_REG_LED_STATUS 0x00     /* LED status register */
#define RPIBMC_LEDS_REG_LED_BRIGHTNESS 0x01 /* Current brightness register */
#define RPIBMC_LEDS_REG_LED_PHASE 0x02      /* Current phase */
#define RPIBMC_LEDS_REG_LED_CTRL_BASE 0x03  /* Control words base offset */

/* Maximum number of LEDS that can be managed by the RPIBMC */
#define RPIBMC_LEDS_MAX 8U

/* Maximum number of control words */
#define RPIBMC_LEDS_MAX_NUM_CTRL_WORDS 8U

/* Maximum number of bytes used by a single LED */
#define RPIBMC_LEDS_MAX_NUM_BYTES \
	(RPIBMC_LEDS_REG_LED_CTRL_BASE + 2U * RPIBMC_LEDS_MAX_NUM_CTRL_WORDS)

#define RPIBMC_LEDS_STATUS_RUN 0x80

#define RPIBMC_LEDS_CTRL_MAX_DELAY 0x7F
#define RPIBMC_LEDS_CTRL_IS_RAMP 0x80
#define RPIBMC_LEDS_CTRL_RETURN 0x80
#define RPIBMC_LEDS_CTRL_MASK 0x7F

#define ldev_to_led(c) container_of(c, struct rpibmc_leds_led_data, ldev)

/* Information about an individual LED */
struct rpibmc_leds_led_data {
	u8 addr;
	u8 num_ctrl_words;
	struct led_classdev ldev;
	struct i2c_client *client;
	struct regmap *regmap;
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
                                      u8 valid)
{
	if (valid) {
		regs[*ptr] = value;
		mask[*ptr] = 0;
	}
	else {
		mask[*ptr] = 1;
	}
	(*ptr)++;
}

/**
 * Updates a single RPIBMC LED. Setting one of the int values to a negative
 * value indicates that this particular register shouldn't be updated.
 */
static int rpibmc_leds_write(struct rpibmc_leds_led_data *led, int status,
                             int brightness, int phase, u8 *ctrl, int ctrl_size)
{
	/* Fetch the controller data for access to the mutex lock */
	struct rpibmc_leds_data *data = i2c_get_clientdata(led->client);

	/* Temporary buffers containing the data that should be written to the I2C
	   device in as few transactions as possible. */
	u8 regs[RPIBMC_LEDS_MAX_NUM_BYTES];
	u8 mask[RPIBMC_LEDS_MAX_NUM_BYTES + 1];
	int i, offs = 0, len = 0, end = 0, err = 0;

	/* Assemble the masked stream */
	rpibmc_leds_append_stream(regs, mask, &end, status, status >= 0);
	rpibmc_leds_append_stream(regs, mask, &end, brightness, brightness >= 0);
	rpibmc_leds_append_stream(regs, mask, &end, phase, phase >= 0);
	for (i = 0; i < ctrl_size; i++) {
		rpibmc_leds_append_stream(regs, mask, &end, ctrl[i], 1);
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
				err = regmap_bulk_write(led->regmap, led->addr + offs,
				                            &regs[offs], len);
				if (err != 0) {
					goto exit;
				}

				offs += len;
				len = 0;
			}
			else {
				/* Skip masked bytes */
				offs++;
			}
		}
		else {
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
	/* Fetch the LED */
	struct rpibmc_leds_led_data *led = ldev_to_led(led_cdev);

	/* Pause the pattern generator and just set the brightness */
	return rpibmc_leds_write(led, 0, brightness, -1, NULL, 0);
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

static int rpibmc_leds_configure(struct i2c_client *client, struct regmap *regmap,
                                 struct rpibmc_leds_data *data)
{
	int i, err, num_leds, num_bytes, num_ctrl_words, addr;

	/* Read the number of leds from the attached device */
	err = regmap_read(regmap, RPIBMC_LEDS_REG_NUM_LEDS, &num_leds);
	if (err) {
		return err;
	}
	if (num_leds > RPIBMC_LEDS_MAX) {
		return EINVAL;
	}

	/* Read the number of bytes per LED */
	err = regmap_read(regmap, RPIBMC_LEDS_REG_NUM_BYTES, &num_bytes);
	if (err) {
		return err;
	}
	if ((num_bytes < RPIBMC_LEDS_REG_LED_CTRL_BASE) ||
	         (num_bytes * num_leds + RPIBMC_LEDS_REG_LED_BASE > 255)) {
		return EINVAL;
	}

	/* Compute the number of control words */
	num_ctrl_words = (num_bytes - RPIBMC_LEDS_REG_LED_CTRL_BASE) / 2;
	if (num_ctrl_words > RPIBMC_LEDS_MAX_NUM_CTRL_WORDS) {
		return EINVAL;
	}

	/* Allocate the devices */
	addr = RPIBMC_LEDS_REG_LED_BASE;
	for (i = 0; i < num_leds; i++) {
		/* Populate the rpibmc_leds_led_data structure */
		struct rpibmc_leds_led_data *led = &data->leds[i];
		led->client = client;
		led->regmap = regmap;
		led->addr = addr;
		led->num_ctrl_words = num_ctrl_words;

		/* Populate the led_classdev structure */
		led->ldev.name = rpibmc_leds_names[i];
		led->ldev.brightness = 0;
		led->ldev.max_brightness = 255;
		led->ldev.brightness_set_blocking = rpibmc_leds_set_brightness;
		led->ldev.flags = LED_CORE_SUSPENDRESUME;

		/* Try to register the LED */
		err = led_classdev_register(&led->client->dev, &led->ldev);
		if (err < 0) {
			dev_err(&client->dev, "couldn't register LED %s\n", led->ldev.name);
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
		dev_err(&client->dev, "%s: regmap allocation failed: %ld\n", __func__,
		        PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	/* Allocate memory for the rpibmc_leds_data struct */
	data =
	    devm_kzalloc(&client->dev, sizeof(struct rpibmc_leds_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Initialize rpibmc_leds_data and associate the struct with the I2C
	   client. */
	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	/* Read information about the LEDs from the device */
	err = rpibmc_leds_configure(client, regmap, data);
	if (err < 0)
		return err;

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

static const struct i2c_device_id rpibmc_leds_id[] = {{"rpibmc-leds", 0}, {}};

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

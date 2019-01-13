// SPDX-License-Identifier: GPL-2.0

/*#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/regmap.h>*/
#include <linux/module.h>
/*#include <linux/mutex.h>
#include <linux/of.h>*/

/*static int rpibmc_leds_probe(struct i2c_client *client,*/
/*                             const struct i2c_device_id *id)*/
/*{*/
/*	return 0;*/
/*}*/

/*static int rpibmc_leds_remove(struct i2c_client *client) { return 0; }*/

/*static const struct i2c_device_id rpibmc_leds_id[] = {{"rpibmc-leds", 0}, {}};*/
/*MODULE_DEVICE_TABLE(i2c, rpibmc_leds_id);*/

/*static const struct of_device_id of_rpibmc_leds_match[] = {*/
/*    {*/
/*        .compatible = "stkl,rpibmc-leds",*/
/*    },*/
/*    {},*/
/*};*/
/*MODULE_DEVICE_TABLE(of, of_rpibmc_leds_match);*/

/*static struct i2c_driver rpibmc_driver = {*/
/*    .driver =*/
/*        {*/
/*            .name = "rpibmc-leds",*/
/*            .of_match_table = of_rpibmc_leds_match,*/
/*        },*/
/*    .probe = rpibmc_leds_probe,*/
/*    .remove = rpibmc_leds_remove,*/
/*    .id_table = rpibmc_leds_id,*/
/*};*/
/*module_i2c_driver(rpibmc_driver);*/

MODULE_DESCRIPTION("RPIBMC LED Driver");
MODULE_AUTHOR("Andreas Stoeckel <andreas.stoeckel@googlemail.com>");
MODULE_LICENSE("GPL v2");

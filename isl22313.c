/*
 *  isl22313.c - Linux kernel module for
 * 	Intersil ISL22313 precision single digitally controlled potentiometer.
 *
 *  Copyright (c) 2014 Peter Milne <peter.milne@d-tacq.com>
 *  Copyright (c) 2012 Andrey Kuyan <kuyan_a@mcst.ru>
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#define ISL22313_DRV_NAME	"isl22313"
#define DRIVER_VERSION		"0.91"

#define ISL22313_WIPER_REG	0x00 /* if (ACR.VOL) { WR accesible }
					else { IVR accessible } */
#define ISL22313_MSR_REG	0x01
#define ISL22313_ACR_REG	0x10

#define ISL22313_VIRT_IVR_REG	0x11

/* WR (Wiper Register) bits: */
#define ISL22313_WR_MASK	0xff
#define ISL22313_WR_RL		0x00
#define ISL22313_WR_RH		0xff

/* IVR (Initial Value Register) bits: */
#define ISL22313_IVR_MASK	0xff
#define ISL22313_IVR_DEFAULT	0x7f

/* ACR (Access Control Register) bits: */
#define ISL22313_ACR_VOL_SHIFT		7
#define ISL22313_ACR_VOL_MASK		(1 << ISL22313_ACR_VOL_SHIFT)
#define ISL22313_ACR_SHDN_SHIFT		6
#define ISL22313_ACR_SHDN_MASK		(1 <<  ISL22313_ACR_SHDN_SHIFT)
#define ISL22313_ACR_WIP_SHIFT		5
#define ISL22313_ACR_WIP_MASK		(1 << ISL22313_ACR_WIP_SHIFT)


/* Common */
#define ISL22313_WIP_ATTEMPTS		100
#define ISL22313_NUM_CACHABLE_REGS	0x12

struct isl22313_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[ISL22313_NUM_CACHABLE_REGS];
};

/*
 * register access helpers
 */

static int __isl22313_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct isl22313_data *data = i2c_get_clientdata(client);
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __isl22313_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct isl22313_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= ISL22313_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	if (reg == ISL22313_VIRT_IVR_REG){
		ret = i2c_smbus_write_byte_data(client, ISL22313_WIPER_REG, tmp);
	}else{
		ret = i2c_smbus_write_byte_data(client, reg, tmp);
	}

	if (!ret){

		data->reg_cache[reg] = tmp;
	}

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* wip */
static u8 isl22313_get_wip(struct i2c_client *client)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, ISL22313_ACR_REG);
	if (ret < 0)
		return -ENODEV;

	return (((u8)ret) & ISL22313_ACR_WIP_MASK) >> ISL22313_ACR_WIP_SHIFT;
}

static int isl22313_check_wip(struct i2c_client *client)
{
	int count = 0;
retry:
	if (isl22313_get_wip(client)) {
		count++;
		printk("get_wip %d\n", count);
		if (count >= ISL22313_WIP_ATTEMPTS)
			return -EAGAIN;
		goto retry;
	}
	return 0;
}

/* vol */
static u8 isl22313_get_vol(struct i2c_client *client)
{
	return __isl22313_read_reg(client, ISL22313_ACR_REG,
		ISL22313_ACR_VOL_MASK, ISL22313_ACR_VOL_SHIFT);
}

static int isl22313_set_vol(struct i2c_client *client, u8 vol)
{
	return __isl22313_write_reg(client, ISL22313_ACR_REG,
		ISL22313_ACR_VOL_MASK, ISL22313_ACR_VOL_SHIFT, vol);
}

/* power_state */
static int isl22313_set_power_state(struct i2c_client *client, u8 state)
{
	if (isl22313_check_wip(client))
		return -EAGAIN;
	
	return __isl22313_write_reg(client, ISL22313_ACR_REG,
				ISL22313_ACR_SHDN_MASK, ISL22313_ACR_SHDN_SHIFT,
				state ? 1 : 0);
}

static u8 isl22313_get_power_state(struct i2c_client *client)
{
	struct isl22313_data *data = i2c_get_clientdata(client);
	u8 cmdreg = data->reg_cache[ISL22313_ACR_REG];
	return ((cmdreg & ISL22313_ACR_SHDN_MASK) >> ISL22313_ACR_SHDN_SHIFT);
}

/* wiper position */
static int isl22313_get_wiper(struct i2c_client *client)
{
	if (!isl22313_get_vol(client)) {
		if (!isl22313_check_wip(client)) {
			if (0 > isl22313_set_vol(client, 1))
				return -EINVAL;
		} else {
			return -EAGAIN;
		}
	}
	return __isl22313_read_reg(client, ISL22313_WIPER_REG,
		ISL22313_WR_MASK, 0);
}

static int isl22313_set_wiper(struct i2c_client *client, u8 wiper)
{
	if (!isl22313_get_vol(client)) {
		if (!isl22313_check_wip(client)) {
			if (0 > isl22313_set_vol(client, 1))
				return -EINVAL;
		} else {
			return -EAGAIN;
		}		
	}
	return __isl22313_write_reg(client, ISL22313_WIPER_REG,
		ISL22313_WR_MASK, 0, wiper);
}

/* initial value */
static int isl22313_get_ivalue(struct i2c_client *client)
{
	if (isl22313_get_vol(client)) {
		if (!isl22313_check_wip(client)) {
			if (0 > isl22313_set_vol(client, 0))
				return -EINVAL;
		} else {
			return -EAGAIN;
		}
	}
	return __isl22313_read_reg(client, ISL22313_VIRT_IVR_REG,
		ISL22313_IVR_MASK, 0);
}

static int isl22313_set_ivalue(struct i2c_client *client, u8 ivalue)
{
	if (isl22313_get_vol(client)) {
		if (!isl22313_check_wip(client)) {
			if (0 > isl22313_set_vol(client, 0))
				return -EINVAL;
		} else {
			return -EAGAIN;
		}
	}
	return __isl22313_write_reg(client, ISL22313_VIRT_IVR_REG,
		ISL22313_IVR_MASK, 0, ivalue);
}


/*
 * sysfs layer
 */

/* wiper */
static ssize_t isl22313_show_wiper(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl22313_get_wiper(client));
}

static ssize_t isl22313_store_wiper(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((kstrtoul(buf, 10, &val) < ISL22313_WR_RL) ||
						(val > ISL22313_WR_RH))
		return -EINVAL;

	ret = isl22313_set_wiper(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(wiper, S_IWUSR | S_IRUGO,
		   isl22313_show_wiper, isl22313_store_wiper);


/* ivalue */
static ssize_t isl22313_show_ivalue(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl22313_get_ivalue(client));
}

static ssize_t isl22313_store_ivalue(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((kstrtoul(buf, 10, &val) < ISL22313_WR_RL) ||
						(val > ISL22313_WR_RH))
		return -EINVAL;

	ret = isl22313_set_ivalue(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ivalue, S_IWUSR | S_IRUGO,
		isl22313_show_ivalue, isl22313_store_ivalue);


/* power state */
static ssize_t isl22313_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", (int)isl22313_get_power_state(client));
}

static ssize_t isl22313_store_power_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((kstrtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;

	ret = isl22313_set_power_state(client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   isl22313_show_power_state, isl22313_store_power_state);

static int isl22313_init_client(struct i2c_client *client)
{
	int v;
	struct isl22313_data *data = i2c_get_clientdata(client);

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */


	v = i2c_smbus_read_byte_data(client, ISL22313_ACR_REG);
	if (v < 0)
		return -ENODEV;

	data->reg_cache[ISL22313_ACR_REG] = (u8)v;

	if (isl22313_get_vol(client)) {
		if (!isl22313_check_wip(client)) {
			if (0 > isl22313_set_vol(client, 0))
				return -EINVAL;
		} else {
			return -EAGAIN;
		}
	}

	v = i2c_smbus_read_byte_data(client, ISL22313_WIPER_REG);
	if (v < 0)
		return -ENODEV;

	data->reg_cache[ISL22313_VIRT_IVR_REG] = (u8)v;

	if (!isl22313_get_vol(client)) {
		if (!isl22313_check_wip(client)) {
			if (0 > isl22313_set_vol(client, 1))
				return -EINVAL;
		} else {
			return -EAGAIN;
		}
	}
	
	v = i2c_smbus_read_byte_data(client, ISL22313_WIPER_REG);
	if (v < 0)
		return -ENODEV;

	data->reg_cache[ISL22313_WIPER_REG] = (u8)v;

	/* set defaults */
	isl22313_set_wiper(client, ISL22313_IVR_DEFAULT);
	isl22313_set_power_state(client, 1);

	return 0;
}

static ssize_t isl22313_store_reinit_chip(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = isl22313_init_client(client);

	return ret ? ret : count;
}

static DEVICE_ATTR(reinit_chip, S_IWUSR,
			NULL, isl22313_store_reinit_chip);

static struct attribute *isl22313_attributes[] = {
	&dev_attr_wiper.attr,
	&dev_attr_ivalue.attr,
	&dev_attr_power_state.attr,
	&dev_attr_reinit_chip.attr,
	NULL
};

static const struct attribute_group isl22313_attr_group = {
	.attrs = isl22313_attributes,
};

/*
 * I2C layer
 */

static int isl22313_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct isl22313_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct isl22313_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	/* initialize the ISL22313 chip */
	err = isl22313_init_client(client);
	if (err)
		goto exit_kfree;

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &isl22313_attr_group);
	if (err)
		goto exit_kfree;

	dev_info(&client->dev, "isl22313 added device %s\n", id->name);
	return 0;

exit_kfree:
	kfree(data);
	return err;
}

static int isl22313_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &isl22313_attr_group);
	isl22313_set_power_state(client, 0);
	kfree(i2c_get_clientdata(client));
	return 0;
}

#if 0
#ifdef CONFIG_PM
static int isl22313_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return isl22313_set_power_state(client, 0);
}

static int isl22313_resume(struct i2c_client *client)
{
	return isl22313_set_power_state(client, 1);
}

#else
#define isl22313_suspend	NULL
#define isl22313_resume		NULL
#endif /* CONFIG_PM */
#endif

static const struct i2c_device_id isl22313_id[] = {
	{ "isl22313", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isl22313_id);


static const struct of_device_id isl22313_dt_ids[] = {
	{ .compatible = "isl,isl22313", },
	{}
};
static struct i2c_driver isl22313_driver = {
	.driver = {
		.name	= ISL22313_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = isl22313_dt_ids,
	},
/*
	.suspend = isl22313_suspend,
	.resume	= isl22313_resume,
*/
	.probe	= isl22313_probe,
	.remove	= isl22313_remove,
	.id_table = isl22313_id,
};

static int __init isl22313_init(void)
{
	printk( "isl22313 driver version %s\n", DRIVER_VERSION);
	return i2c_add_driver(&isl22313_driver);
}

static void __exit isl22313_exit(void)
{
	i2c_del_driver(&isl22313_driver);
}

MODULE_AUTHOR("Peter Milne peter.milne@d-tacq.com");
MODULE_DESCRIPTION("ISL22313 digitally controlled potentiometer driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl22313_init);
module_exit(isl22313_exit);


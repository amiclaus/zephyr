/*
 * Copyright (c) 2023 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/__assert.h>

#define DELAY_WITH_TRIGGER K_SECONDS(5)
#define DELAY_WITHOUT_TRIGGER K_SECONDS(1)

#define UCEL_PER_CEL 1000000
#define UCEL_PER_MCEL 1000
#define TEMP_INITIAL_CEL 21
#define TEMP_WINDOW_HALF_UCEL 500000

K_SEM_DEFINE(sem, 0, 1);	/* starts off "not available" */

#ifdef CONFIG_ADXL367_TRIGGER
static struct sensor_trigger sensor_trig;

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trigger)
{

	struct sensor_value x, y, z, temp;

	sensor_sample_fetch(dev);

	sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

	printk("temp: %d.%06d; x: %d.%06d; y: %d.%06d; z: %d.%06d\n",
		      temp.val1, temp.val2, x.val1, x.val2,
		      y.val1, y.val2,
		      z.val1, z.val2);

	k_sem_give(&sem);
}
#endif

int main(void)
{
	int ret;

	const struct device *const dev = DEVICE_DT_GET_ONE(adi_adxl367);

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	printf("device is %p, name is %s\n", dev, dev->name);

#ifdef CONFIG_ADXL367_TRIGGER
	sensor_trig.type = SENSOR_TRIG_DATA_READY;

	ret = sensor_trigger_set(dev, &sensor_trig, trigger_handler);
#else
	while (1) {

		struct sensor_value x, y, z, temp;

		sensor_sample_fetch(dev);

		sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

		printk("temp: %d.%06d; x: %d.%06d; y: %d.%06d; z: %d.%06d\n",
		      temp.val1, temp.val2, x.val1, x.val2,
		      y.val1, y.val2,
		      z.val1, z.val2);

		k_sleep(K_MSEC(1000));

	}
#endif
	return 0;
}

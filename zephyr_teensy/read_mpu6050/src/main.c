

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#define SLEEP_TIME_MS   200
#define LED0_NODE DT_NODELABEL(board_led)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

static int process_mpu6050(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
					&temperature);
	}
	if (rc == 0) {
		printk("[%s]:%g Cel\n"
		       "  accel %f %f %f m/s/s\n"
		       "  gyro  %f %f %f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		printk("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

void main(void)
{

	
    usb_enable(NULL);

    int ret;

	const struct device *dev = DEVICE_DT_GET_ONE(invensense_mpu6050);

    if (dev != NULL)
    {
        printk("\nDevice found on Teensy DTS");
    }

    if (device_is_ready(dev))
    {
        ret = gpio_pin_toggle_dt(&led);
        printk("\nDevice %s is ready\n", dev->name);
    }

	

    while (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
		int rc = process_mpu6050(dev);

		if (rc != 0) {
			break;
		}
		k_sleep(K_SECONDS(2));
	}

    // while (1) {

    //     if (dev != NULL)
    //     {
    //         printk("\nDevice found on Teensy DTS");
    //     }

    //     if (device_is_ready(dev))
    //     {
    //         ret = gpio_pin_toggle_dt(&led);
    //         printk("\nDevice %s is ready\n", dev->name);
    //     }
    //     // else{
    //     //     printk("Device %s is not ready\n", dev->name);
    //     //     ret = gpio_pin_toggle_dt(&led);

    //     // }
	// 	// ret = gpio_pin_toggle_dt(&led);
    //     // printk("Device %s is not ready\n", dev->name);
	// 	if (ret < 0) {
	// 		return;
	// 	}
	// 	k_msleep(SLEEP_TIME_MS);
	// }
    // return;
    // LOG_INF("Found device %s. Reading sensor data\n", dev->name);

	// printk("Found device %s. Reading sensor data\n", dev->name);

	
}
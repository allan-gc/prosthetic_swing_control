#include "mpu_6050.h"

const char *now_str(void)
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

int process_mpu6050(const struct device *dev, struct mpu_6050_data *mpu_sensor_data)
{

	int rc = sensor_sample_fetch(dev);

    //// FOR FIFO  ////
	// if (rc == 0) {
	// 	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
	// 				tx->accelerometer);
	// }
	// if (rc == 0) {
	// 	rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
	// 				tx->gyro);
	// }

    extern struct k_mutex mpu_sensor_mutex; 

    k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);


    if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					mpu_sensor_data->accelerometer);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					mpu_sensor_data->gyro);
	}

    k_mutex_unlock(&mpu_sensor_mutex);

    // extern struct k_fifo printk_fifo;

    // size_t size = sizeof(struct printk_data_t);
    // char *mem_ptr = k_malloc(size);
    // __ASSERT_NO_MSG(mem_ptr != 0);

    // memcpy(mem_ptr, tx, size);
    

    // k_fifo_put(&printk_fifo, mem_ptr);

	if (rc == 0) {
	} else {
		printk("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}
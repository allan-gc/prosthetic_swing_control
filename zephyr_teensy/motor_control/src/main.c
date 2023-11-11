
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include<string.h>
#include "mpu_6050.h"
#include "ina_219.h"

#define STACKSIZE 1024
#define PRIORITY 7
// #define SPEED PWM_MSEC(23)
#define SPEED PWM_KHZ(3)

#define WINDOW_SIZE 25

static const struct pwm_dt_spec pwm_28 = PWM_DT_SPEC_GET(DT_NODELABEL(pin28));
static uint32_t min_pulse_28 = DT_PROP(DT_NODELABEL(pin28), min_pulse);
// static uint32_t max_pulse_28 = DT_PROP(DT_NODELABEL(pin28), max_pulse);


static const struct pwm_dt_spec pwm_29 = PWM_DT_SPEC_GET(DT_NODELABEL(pin29));
static uint32_t min_pulse_29 = DT_PROP(DT_NODELABEL(pin29), min_pulse);
// static uint32_t max_pulse_29 = DT_PROP(DT_NODELABEL(pin29), max_pulse);


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

enum direction {
	DOWN,
	UP,
};


K_FIFO_DEFINE(printk_fifo);

K_MUTEX_DEFINE(mpu_sensor_mutex);
K_MUTEX_DEFINE(ina_sensor_mutex);
K_MUTEX_DEFINE(pot_adc_mutex);


struct mpu_6050_data mpu_data;
struct ina_219_data ina_data;
const struct device *mpu_dev, *ina_dev;

volatile static float angle = 170.0;
volatile static float commanded_current = 0.0;
static volatile float Kp = 0.13, Ki = 0.025;
static volatile float Kp_pos = 1.3, Ki_pos = 0.09, Kd_pos = 1.0;
// static volatile float Kp = 0.15, Ki = 0.02; ///// WORKED WELl
// static volatile float Kp_pos = 0.9, Ki_pos = 0.09, Kd_pos = 0.5;  WORKED FOR POSITION ONLY CONTROLLER

unsigned int pwm_val = 0;
int dir_flag = 1;
int data_count = 0;

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

struct adc_sequence sequence;
uint16_t buf;
float filteredEncoderReading = 0.0;



const struct device * sensors_init(const char* sensor_compat)
{
    
    const struct device *dev;
    if (strcmp(sensor_compat, "mpu") == 0)
    {
        dev = DEVICE_DT_GET_ONE(MPU_COMPAT);
    }
    else if (strcmp(sensor_compat, "ina") == 0)
    {
        dev = DEVICE_DT_GET_ONE(INA_COMPAT);
    }

    if (dev != NULL)
    {
        printk("\nDevice found on Teensy DTS\n");
    }

    if (device_is_ready(dev))
    {
        printk("\nDevice %s is ready\n", dev->name);
    }
    else{
        printf("Device %s is not ready\n", dev->name);
    }

    return dev;
}

void mpu_sensor_read(void)
{
    const char *sensor_name = "mpu";
    mpu_dev = sensors_init(sensor_name);
    
    while (1) { 
		int rc = process_mpu6050(mpu_dev, &mpu_data);
		
		if (rc != 0) {
			break;
		}
		// k_msleep(500);
	}
}


void ina_sensor_read(void)
{
    const char *sensor_name = "ina";
    ina_dev = sensors_init(sensor_name);
    while (1) { 
		int rc = process_ina219(ina_dev, &ina_data);
		
		if (rc != 0) {
			break;
		}
        k_sleep(K_MSEC(1));  
	}
}

void uart_out(void)
{
    struct printk_data_t *rx_data;
	while (1) {
		rx_data = k_fifo_get(&printk_fifo,
							   K_FOREVER);
        printf("[%s]:\n"
            "accel %f %f %f m/s/s\n"
            "  gyro  %f %f %f rad/s\n",
            now_str(),
        //    sensor_value_to_double(&temperature),
            sensor_value_to_double(rx_data->accelerometer),
            sensor_value_to_double(rx_data->accelerometer+1),
            sensor_value_to_double(rx_data->accelerometer+2),
            sensor_value_to_double(rx_data->gyro),
            sensor_value_to_double(rx_data->gyro+1),
            sensor_value_to_double(rx_data->gyro+2));
		k_free(rx_data);
	}
}

void motor_control(void)
{

	enum direction dir = UP;
	int ret;

	if (!device_is_ready(pwm_28.dev)) {
		printf("Error: PWM device %s is not ready\n", pwm_28.dev->name);
	}

	if (!device_is_ready(pwm_28.dev)) {
		printf("Error: PWM device %s is not ready\n", pwm_29.dev->name);
	}

	ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
	if (ret < 0) {
		printf("Error %d: failed to set pulse width\n", ret);
	}

	ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
	if (ret < 0) {
		printf("Error %d: failed to set pulse width\n", ret);
	}
    
    double accel_z;
    // k_sleep(K_SECONDS(5));

	while (1) {

        // k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);
        // k_mutex_lock(&ina_sensor_mutex, K_FOREVER);



        // printf("[%s]:\n"
        // "accel %f m/s/s\n",
        // now_str(),
        // sensor_value_to_double(&mpu_data.accelerometer[2]));


        // printf("Current: %f [A]\n",sensor_value_to_double(&ina_data.current));
        
        // accel_z = sensor_value_to_double(&mpu_data.accelerometer[2]);

        // printk("%"PRId32 "\n", (int32_t)buf);


        // k_mutex_unlock(&mpu_sensor_mutex);
        // k_mutex_unlock(&ina_sensor_mutex);

    
    
        if(dir_flag != 1)
        {
            ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
            if (ret < 0) {
                printk("Error %d: failed to set 0 pulse width for 28\n", ret);
                break;
            }

            k_msleep(50);
            
            ret = pwm_set_pulse_dt(&pwm_29, pwm_val);
            if (ret < 0) {
                printk("Error %d: failed to set pulse width for pin 29\n", ret);
                break;
		    }
    
        }
        else
        {
            ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
            if (ret < 0) {
                printk("Error %d: failed to set 0 pulse width for 29\n", ret);
                break;
            }

            k_msleep(50);
            // /// 28 CLOCKWISE (+) CURRENT
            ret = pwm_set_pulse_dt(&pwm_28, pwm_val);
            if (ret < 0) {
                printk("Error %d: failed to set pulse width for pin 28\n", ret);
                break;
		    }
        }

        k_sleep(K_MSEC(1));  
    }
}


void read_pot_adc(void)
{
    int err;
	uint32_t count = 0;

    sequence.buffer = &buf;
    sequence.buffer_size = sizeof(buf);
    

    float rawBuffer[WINDOW_SIZE] = {0}; // Initialize buffer with zeros
    int currentIndex = 0; // Index for circular buffer

    k_sleep(K_SECONDS(4));

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device %s not ready\n",adc_channels[i].dev->name);
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
		}
	}

	while (1) {

        (void)adc_sequence_init_dt(&adc_channels[0], &sequence);

        k_mutex_lock(&pot_adc_mutex, K_FOREVER);

        err = adc_read(adc_channels[0].dev, &sequence);
        if (err < 0) {
            printk("Could not read (%d)\n", err);
            continue;
        }
        k_mutex_unlock(&pot_adc_mutex);
  
       float sum = 0;

        // Update circular buffer with new raw reading
        rawBuffer[currentIndex] = buf;
        currentIndex = (currentIndex + 1) % WINDOW_SIZE;

        // Calculate the average of the values in the circular buffer
        for (int i = 0; i < WINDOW_SIZE; ++i) {
            sum += rawBuffer[i];
        }

        filteredEncoderReading = sum / WINDOW_SIZE;

		k_sleep(K_MSEC(1));
	}
}


void current_control(void)
{
    static volatile float eint = 0;
    k_sleep(K_SECONDS(5));
    while(1)
    {
        // static volatile float eint = 0;
        float ref_current = commanded_current;

    
        k_mutex_lock(&ina_sensor_mutex, K_FOREVER);

        float adcval = sensor_value_to_double(&ina_data.current) * 1000;
        // printk("REF %f  ADC %f \n", ref_current, adcval);

        k_mutex_unlock(&ina_sensor_mutex);

        float error = -(ref_current - adcval);
        // printk("ERROR:    %f\n", error);
       

        eint = eint + error;

        if (eint > 500)
        {
            eint = 500;
        }
        if(eint < -500)
        {
            eint = -500;
        }

        float u = Kp*error + Ki*eint;

        if (u > 100.0)
        {
          u = 100.0;
        }
        if (u < -100.0)
        {
          u = -100.0;
        }

        if (u > 0.0) {
            dir_flag = 1;
            pwm_val = ((u)/100 * SPEED);
        } else {
            dir_flag = -1;
            pwm_val = ((u)/-100 * SPEED);
        }

        k_sleep(K_MSEC(1));  
    }
}

void position_control(void)
{
    // static int counter = 0;
    static volatile float eint_pos = 0.0;
    static volatile float ed_pos = 0.0;
    static volatile float eprevious_pos = 0.0; 


    while(1)
    {

        k_mutex_lock(&pot_adc_mutex, K_FOREVER);

       
        float encoder = filteredEncoderReading; // get the encoder value


        float actual_angle = encoder * (220.0/4095.0);
        printk("ACTUAL ANGLE: %f\n", actual_angle);

        k_mutex_unlock(&pot_adc_mutex);

        float pos_error = (angle - actual_angle);

        eint_pos = eint_pos + pos_error;

        if (eint_pos > 500)
        {
            eint_pos = 500;
        }
        if(eint_pos < -500)
        {
            eint_pos = -500;
        }

        ed_pos = (pos_error - eprevious_pos) / 0.005;
        
        float u = Kp_pos*pos_error + Ki_pos*eint_pos + Kd_pos*ed_pos; 

        commanded_current = u;
        eprevious_pos = pos_error;


        // if (commanded_current > 100.0)
        // {
        //   commanded_current = 100.0;
        // }
        // if (commanded_current < -100.0)
        // {
        //   commanded_current = -100.0;
        // }

        
        // if (commanded_current < 0.0) {
        //     dir_flag = 1;
        //     pwm_val = ((commanded_current/-100)*SPEED);
        // } else if (commanded_current > 0.0){
        //     dir_flag = -1;
        //     pwm_val = ((commanded_current/100)*SPEED);
        // }


        // printk("POS ERR: %f ED ERROR: %f COMMAND %f\n", pos_error, ed_pos, u );

        k_sleep(K_MSEC(5));
    }
}



K_THREAD_DEFINE(motor_id, STACKSIZE, motor_control, NULL, NULL, NULL,
PRIORITY, 0, 0);

K_THREAD_DEFINE(ina219_id, STACKSIZE, ina_sensor_read, NULL, NULL, NULL,
PRIORITY, 0, 0);


// K_THREAD_DEFINE(mpu6050_id, STACKSIZE, mpu_sensor_read, NULL, NULL, NULL,
// PRIORITY, 0, 0);

K_THREAD_DEFINE(pot_id, STACKSIZE, read_pot_adc, NULL, NULL, NULL,
PRIORITY, 0, 0);

K_THREAD_DEFINE(pos_control_id, STACKSIZE, position_control, NULL, NULL, NULL,
PRIORITY, 0, 0);

K_THREAD_DEFINE(current_control_id, STACKSIZE, current_control, NULL, NULL, NULL,
PRIORITY, 0, 0);



void main(void)
{

    // General app startups //
    usb_enable(NULL);

    // // Sensor threads //
    // k_thread_start(mpu6050_id);
    // // k_thread_start(ina219_id);

    // // // Motor control thread //
    // k_thread_start(motor_id);

    // // k_thread_start(pot_id);
}



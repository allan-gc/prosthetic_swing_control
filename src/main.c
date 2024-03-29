#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>
#include "mpu_6050.h"
#include "ina_219.h"
#include "data_profile.h"

#define STACKSIZE 1024
#define PRIORITY 7
#define INA_WINDOW 1000
#define IMU_WINDOW 1500
#define SPEED PWM_KHZ(20)   // Sets period to 3kHz with 100% duty cycle being 333,333 ns. 
#define WINDOW_SIZE 35

#define RPM 68
#define V_NOM 7.4
#define PI 3.14159265358979323846
#define KV  RPM/V_NOM
#define KV_SI  (KV * ((2*PI)/60))  // radians/second/volt
#define KT  (1/KV_SI)

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif


float InaSamples[INA_WINDOW], ImuSamples[IMU_WINDOW], OutputCurrent[DATA_SAMPLES]; 

static const struct pwm_dt_spec pwm_28 = PWM_DT_SPEC_GET(DT_NODELABEL(pin28));
static uint32_t min_pulse_28 = DT_PROP(DT_NODELABEL(pin28), min_pulse);
// static uint32_t max_pulse_28 = DT_PROP(DT_NODELABEL(pin28), max_pulse);


static const struct pwm_dt_spec pwm_29 = PWM_DT_SPEC_GET(DT_NODELABEL(pin29));
static uint32_t min_pulse_29 = DT_PROP(DT_NODELABEL(pin29), min_pulse);
// static uint32_t max_pulse_29 = DT_PROP(DT_NODELABEL(pin29), max_pulse);


struct mpu_6050_data mpu_data;
struct ina_219_data ina_data;
const struct device *mpu_dev, *ina_dev;

volatile static float angle = 130.0;
volatile static float commanded_current = 0.0;
volatile static float accel_profile;

// static volatile float Kp = 0.010, Ki = 0.010; // Worked with 20 Kz PWM  
static volatile float Kp = 0.02, Ki = 0.018; // Worked with 20 Kz PWM with 0.005 scale worked with 08ms

static volatile float Kp_pos = 1.4, Ki_pos = 0.08, Kd_pos = 1.00;

unsigned int pwm_val;
int dir_flag = 1;
int data_count = 0;
static int counter = 0;
bool motion_detect = false;
int ret;
float torque_scale;
float max_imu_value = 0.0; float min_imu_value = 0.0;
int steps;


/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

struct adc_sequence sequence;
uint16_t buf;
float filteredEncoderReading = 0.0;


// Define mutex for sensor readings
K_FIFO_DEFINE(printk_fifo);
K_SEM_DEFINE(start_sem, 0, 1);
K_MUTEX_DEFINE(mpu_sensor_mutex);
K_MUTEX_DEFINE(ina_sensor_mutex);
K_MUTEX_DEFINE(pot_adc_mutex);



float mean_val(float arr[], int len)
{
    float sum = 0.0, mean = 0.0;

    for(int i=0; i<len; ++i) {
        sum += arr[i];
    }

    mean = sum/len;
    return mean;
}

float max_val(float arr[], int len)
{
    float max = arr[0]; 

    for (int i = 1; i < len; i++) {
        if (arr[i] > max) {
            max = arr[i]; 
        }
    }
    return max;
}

float min_val(float arr[], int len)
{
    float min = arr[0]; 

    for (int i = 1; i < len; i++) {
        if (arr[i] <  min) {
            min = arr[i]; 
        }
    }
    return min;
}

float min_val_ranged(float arr[], int len, int start)
{
    float min = arr[start]; 

    for (int i = start; i < len; i++) {
        if (arr[i] <  min) {
            min = arr[i]; 
        }
    }
    return min;
}


int base_swing(void)
{
    ret = pwm_set_pulse_dt(&pwm_28,min_pulse_28);
    if (ret < 0) {
        printk("Error %d: failed to set pulse width for pin 28\n", ret);
    }

    k_msleep(700);

      ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
    if (ret < 0) {
        printk("Error %d: failed to set pulse width for pin 28\n", ret);
    }

    k_msleep(50);


    ret = pwm_set_pulse_dt(&pwm_29,min_pulse_28);
    if (ret < 0) {
        printk("Error %d: failed to set pulse width for pin 28\n", ret);
    }

    k_msleep(700);


    ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
    if (ret < 0) {
        printk("Error %d: failed to set pulse width for pin 28\n", ret);
    }

    k_msleep(50);

    return ret;
}

int controlled_swing(void)
{
    if(dir_flag == -1)
    {
        ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
        if (ret < 0) {
            printk("Error %d: failed to set 0 pulse width for 28\n", ret);
        }

        k_msleep(50);
        
        ret = pwm_set_pulse_dt(&pwm_29, pwm_val);
        if (ret < 0) {
            printk("Error %d: failed to set pulse width for pin 29\n", ret);
        }

    }
    else if (dir_flag ==1)
    {
        ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
        if (ret < 0) {
            printk("Error %d: failed to set 0 pulse width for 29\n", ret);
        }

        k_msleep(50);
        // /// 28 CLOCKWISE (+) CURRENT
        ret = pwm_set_pulse_dt(&pwm_28, pwm_val);
        if (ret < 0) {
            printk("Error %d: failed to set pulse width for pin 28\n", ret);
        }
    }
    else
    {
        ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
        if (ret < 0) {
            printf("Error %d: failed to set pulse width\n", ret);
        }
        k_msleep(50);

        ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
        if (ret < 0) {
            printf("Error %d: failed to set pulse width\n", ret);
        }
        k_msleep(50);
    }

    return ret;
}

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

/// Thread Definitions

void mpu_sensor_read(void)
{
    const char *sensor_name = "mpu";
    mpu_dev = sensors_init(sensor_name);    
    float accel_mag, accel_x, accel_y, accel_z;
    float smooth_data = 0.0;
    float LPF_Beta = 0.010; // 0.010
    float prev,avg;
    float point_arr[3];

    k_sleep(K_SECONDS(5));
    while (1) { 
		int rc = process_mpu6050(mpu_dev, &mpu_data);
		
		if (rc != 0) {
			break;
		}

        k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);
        accel_x = sensor_value_to_double(&mpu_data.accelerometer[0])- 0.6;
        accel_y = sensor_value_to_double(&mpu_data.accelerometer[1]) - 9.8;
        accel_z = sensor_value_to_double(&mpu_data.accelerometer[2]) + 2.0;
        accel_mag = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
        k_mutex_unlock(&mpu_sensor_mutex);

        if(data_count < IMU_WINDOW)
        {

            smooth_data = smooth_data - (LPF_Beta * (smooth_data - ((accel_mag))));
            ImuSamples[data_count] = smooth_data;
            data_count++;
            // printf("%f \n",smooth_data);
        
        }
        else if (data_count == IMU_WINDOW)
        {
            prev = ImuSamples[0];
            int direction = 1;
            int peak_freq = 0;
            int peak_loc = 0;
            steps = 0;
            max_imu_value = max_val(ImuSamples, IMU_WINDOW);
            min_imu_value = min_val_ranged(ImuSamples, IMU_WINDOW, 500);

            for(int i=1; i < IMU_WINDOW -1; i++)
            {
                if (ImuSamples[i] < prev && direction == 1)
                {
                    if ((i -peak_loc) > 50)
                    {
                        point_arr[0] = ImuSamples[i-1];    
                        point_arr[1] = ImuSamples[i];
                        point_arr[2] = ImuSamples[i+1];
                        avg = mean_val(point_arr, 3);
                        if (avg > 0.9*max_imu_value)
                        {
                            steps++;
                            peak_loc = i;
                        }
                    }
                    direction = 0;
                }
                else if (ImuSamples[i] > prev && direction == 0)
                {
                    direction = 1;
                }
                prev = ImuSamples[i];
            }
            data_count = 0;
        }

		k_sleep(K_MSEC(1)); 
	}
}

void ina_sensor_read(void)
{
    // Runs at 200 Hz
    const char *sensor_name = "ina";
    ina_dev = sensors_init(sensor_name);

    while (1) { 

		int rc = process_ina219(ina_dev, &ina_data);
		
		if (rc != 0) {
			break;
		}

	}
}



void motor_control(void)
{
	int ret;
    float output_torque;
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
    
    int res;
	while (1) {

        if (! motion_detect)
        {
            res = base_swing();
            if (res < 0.0)
            {
                break;
            }
        }

        else
        {
            res = controlled_swing();
             if (res < 0.0)
            {
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
    

    float rawBuffer[WINDOW_SIZE] = {0}; 
    int currentIndex = 0; 

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

        rawBuffer[currentIndex] = buf;
        currentIndex = (currentIndex + 1) % WINDOW_SIZE;

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
    float int_control = 0;
    float output_torque, input_torque;

    k_sleep(K_SECONDS(5));
    while(1)
    {
       if(counter == 1)
       {
            pwm_val = 0.0;
            break;
       }

        float ref_current = commanded_current * 1000.0;

        k_mutex_lock(&ina_sensor_mutex, K_FOREVER);
        float adcval = sensor_value_to_double(&ina_data.current) * 1000.0;
        k_mutex_unlock(&ina_sensor_mutex);

        k_mutex_lock(&pot_adc_mutex, K_FOREVER);

        float encoder = filteredEncoderReading; // get the encoder value

        float actual_angle = encoder * (180.0/4095.0);

        k_mutex_unlock(&pot_adc_mutex);

        float error = (ref_current - adcval);

        eint = eint + error;
        int_control = Ki*eint;

        if(int_control > 10.0)
        {
            int_control = 10.0;
        }
        else if (int_control < -10.0)
        {
            int_control = -10.0;
        }
        

        float u = Kp*error + int_control;

        if (accel_profile > 0.0)
        {
            dir_flag = 1;
        }
        else
        {
            dir_flag = -1;
            torque_scale*=-1;
        }

        u = u  + torque_scale;

        if (u > 100.0)
        {
          u = 100.0;
        }
        if (u < -100.0)
        {
          u = -100.0;
        }

        if (u >= 0.0) {
            pwm_val = (u/100* SPEED);

        } else {
            pwm_val = (-u/100 * SPEED);
        }

        // printk("%f %f\n", accel_profile, actual_angle);  

        k_sleep(K_MSEC(1));  
    }
}

void position_control(void)
{
    static volatile float eint_pos = 0.0;
    static volatile float ed_pos = 0.0;
    static volatile float eprevious_pos = 0.0; 


    while(1)
    {
        k_mutex_lock(&pot_adc_mutex, K_FOREVER);

        float encoder = filteredEncoderReading; 

        float actual_angle = encoder * (220.0/4095.0);

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

        float pwm_pos_i = Ki_pos*eint_pos;



        ed_pos = (pos_error - eprevious_pos) / 0.005;
        
        float u = Kp_pos*pos_error + pwm_pos_i + Kd_pos*ed_pos; 

        commanded_current = u;
        eprevious_pos = pos_error;

        
        k_sleep(K_MSEC(5));
    }
}

void torque_profile_signals(void)
{
    k_sleep(K_SECONDS(5));
    float max_torque;
    float torque;

    while(1)
    {
        if ((max_imu_value - min_imu_value) > 0.2)
        {
            motion_detect = true;
            for(int i=0; i < DATA_SAMPLES-2; i++)
            {
                if ((max_imu_value - min_imu_value) < 0.2)
                {
                    motion_detect =false;
                    break;
                }
                torque_scale = 0.0;
                
                
                if(steps > 1)
                {
                    torque = Torque_Profile_05[i];
                    accel_profile = Accel_Profile_05[i];

                }
                else if (steps > 9 && steps < 20)
                {
                    torque = Torque_Profile_08[i];
                    accel_profile = Accel_Profile_08[i];

                    max_torque = min_val(Torque_Profile_08, DATA_SAMPLES);
                    torque_scale = (max_torque * 0.005)*1000.0;
                }
                commanded_current = torque / KT;
                // printk("%f\n", torque)
                k_sleep(K_MSEC(10));
            }
        }
        else
        {
            // printk("MOTION NOT DETECTED\n");
            motion_detect = false;
        }
        k_sleep(K_MSEC(10));
    }
}


K_THREAD_DEFINE(motor_id, STACKSIZE, motor_control, NULL, NULL, NULL,
PRIORITY, 0, 0);

K_THREAD_DEFINE(ina219_id, STACKSIZE, ina_sensor_read, NULL, NULL, NULL,
PRIORITY, 0, 0);

K_THREAD_DEFINE(mpu6050_id, STACKSIZE, mpu_sensor_read, NULL, NULL, NULL,
PRIORITY, 0, 0);


K_THREAD_DEFINE(pot_id, STACKSIZE, read_pot_adc, NULL, NULL, NULL,
PRIORITY, 0, 0);

// // K_THREAD_DEFINE(pos_control_id, STACKSIZE, position_control, NULL, NULL, NULL,
// // PRIORITY, 0, 0);

K_THREAD_DEFINE(current_control_id, STACKSIZE, current_control, NULL, NULL, NULL,
PRIORITY, 0, 0);

K_THREAD_DEFINE(torque_profile, STACKSIZE, torque_profile_signals, NULL, NULL, NULL,
PRIORITY, 0, 0);


int main(void)
{
    usb_enable(NULL);
    return 0;
}



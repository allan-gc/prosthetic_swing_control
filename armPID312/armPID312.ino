//#include <WireKinetis.h>
//#include <WireIMXRT.h>
//#include <Wire.h>




// Gait-Simulating Prosthetic Arm Elbow: Basic Functionality Code
// --------------------------------------------------------------
// this code demonstrates the functionality of all components of the prototype including a positional
// PID control, a current sensor, a gyroscope/accelerometer, and a rudimentary battery monitoring method.
// Edit the printing booleans under the USER INPUT heading to print different data to the serial monitor.
//
// This code was cobbled together by copy-pasting and modifying from these sources:
// PID control: Arduino PID library and its documentation on Brett Beauregard's blog, brettbeauregard.com
// Adafruit INA219 Current Sensor: example code in library
// MPU 6050 Accelerometer/Gyroscope: Michael Shoeffler's blog, http://www.mschoeffler.de

#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

//------------------  PINS  ------------------
#define MOTOR_23 23
#define MOTOR_22 22
#define POT_IN 38
#define BAT_IN 27
#define LED 13



//------------------  USER INPUT  ------------------
// these variables characterize the sine wave of the setpoint trajetory for PID control
double u_amplitude = 10; // set amplitude of sine wave in degrees
double u_offset = -85; // set offset from centerline of sine wave in degrees
double u_frequency = 3; // set frequency in arbitrary time units!!

// printing choices
bool printPID = 0;
bool printBattery = 0;
bool printCurrent = 0;
bool printAccelGyro = 1;

// running choices
bool runPID = 0;
bool runBattery = 0;
bool runCurrent  =0;
bool runAccelGyro = 1;
float R1 = 30000.0; //  
float R2 = 7500.0; // 

//------------------  SETUP  ------------------
// sine wave: converting user input values to code (deg -> pot value)
double amplitude = u_amplitude * (1023 / 180);
double offset = (u_offset + 90 + 28) * (1023 / 180);
int divisions = (1 / u_frequency) * 100; // more divisions is slower so divisions are inversely related to frequency

// Define PID Variables
double Setpoint_p, Input_p, Output_p;
double Kp_p = 10, Ki_p = 0, Kd_p = 0; // TUNE PID CONTROL HERE
PID PID_p(&Input_p, &Output_p, &Setpoint_p, Kp_p, Ki_p, Kd_p, DIRECT);

// initialize current sense variables
float shuntvoltage, busvoltage, current_mA, loadvoltage, power_mW;

// other variables
int i; // for generating sine wave
int motor_speed;

//------------------  CURRENT SENSE  ------------------
Adafruit_INA219 ina219;

//------------------  ACCEL  ------------------
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  Serial.println("Serial is alive!");
//
//  if (! ina219.begin()) {
//    Serial.println("Failed to find INA219 chip");
//    while (1) {
//      delay(10);
//    }
//  }

  //------------------  PIN MODES  ------------------
  //pinMode(SWITCH_PIN, INPUT); // digital
  pinMode(MOTOR_23, OUTPUT); // PWM
  pinMode(MOTOR_22, OUTPUT); // PWM
//  pinMode(23, OUTPUT); // PWM
  pinMode(POT_IN, INPUT); // analog
  pinMode(BAT_IN, INPUT); // analog
  pinMode(LED, OUTPUT); // digital

  //------------------  PID  ------------------
  Input_p = analogRead(POT_IN); Setpoint_p = 0; // initialize the variables we're linked to
  i = 0; // used in generating setpoint along sine wave

  PID_p.SetOutputLimits(-255, 255); // set outputs to between -255 and 255 for full range of motor voltage
  PID_p.SetSampleTime(20); // sample time in ms
  PID_p.SetMode(AUTOMATIC); // turn PID on

  ina219.begin();

  //------------------  ACCEL  ------------------
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
void loop()
{
  //------------------  PAUSING  ------------------
//  bool pausing = (digitalRead(SWITCH_PIN) == LOW); //check go state
//  while (pausing) { // pause code so that nothing happens if switch is off
//    digitalWrite(MOTOR_1, LOW);
//    digitalWrite(MOTOR_2, LOW);
//    delay(10); // just do nothing in here and make sure the motor is off
//    pausing = (digitalRead(SWITCH_PIN) == LOW); // check if still pausing
//  }

  //------------------  PID  ------------------
  if (runPID) {
    Input_p = 1023 - analogRead(POT_IN);

    int k = 20;
    digitalWrite(MOTOR_22, LOW);
//    analogWrite(MOTOR_25, 64);
//    analogWrite(23, 64);
    Serial.print("Motor Speed: "); Serial.print(k, 2); Serial.print(" ");
    for (int k=0; k<100; k++) {
      analogWrite(MOTOR_23, k);
      Serial.print("Motor Speed: "); Serial.print(k, 2); Serial.print(" ");
      Serial.println("");
//      delay(2);
    }
  
//    PID_p.Compute();
//    motor_speed = -1.0 *20;
//
//    // communicate speed to motor
//    if (motor_speed > 0) {
//      digitalWrite(MOTOR_1, LOW);
//      analogWrite(MOTOR_2, motor_speed);
//    } else {
//      digitalWrite(MOTOR_2, LOW);
//      analogWrite(MOTOR_1, -1.0 * motor_speed);
//    }
//
//    // update Setpoint
//    i = i + 1;
//    if (i >= divisions) {
//      i = 0;
//    }
//    Setpoint_p = amplitude * sin(i * 2 * 3.1415 / divisions) + offset;
    if (printPID) {
//      Serial.print("set_p: "); Serial.print(Setpoint_p, 2); Serial.print(" ");
      Serial.print("in_p: "); Serial.print(Input_p, 2); Serial.print(" ");
//      Serial.print("Output P: "); Serial.print(Output_p, 2); Serial.print(" ");
      
//      Serial.print("out_p: "); Serial.print(Output_p, 2); Serial.print(" ");
      Serial.println("");
    }
  }
  //------------------  CURRENT SENSE  ------------------
  if (runCurrent) {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    if (printCurrent) {
      Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
      Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
      Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
      Serial.print("Motor Speed:   "); Serial.print(motor_speed); Serial.println("");
      Serial.println("");
    }
  }
  //------------------  GYROSCOPE/ACCELEROMETER  ------------------
  if (runAccelGyro) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    //temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

    if (printAccelGyro) {
      Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
      Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
      Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
      // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
      Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
      Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
      Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
      Serial.println();
    }
  }

//  ------------------  BATTERY MONITOR  ------------------
  if (runBattery) {
    int batSense = analogRead(BAT_IN);
   
    float batVolt1 =  batSense * (3.3 / 1024.0);
    float batVolt2 = batVolt1/ (R2/(R1+R2));
    Serial.println(batVolt2);
    if (batVolt2 < 5.0) {
   
      digitalWrite(LED, HIGH); //turn on built in LED
    }
    if (printBattery) {
//      Serial.print("Battery Voltage: "); Serial.println(batVolt);
    }
  }

}

#include "Wire.h"
#include "LSM303.h"
#include "L3G.h"

// Pins for the two status LEDs
#define GREEN_LED A2
#define RED_LED -1


// Pin definitions for the four DRV8833 PWM motor channels. 
#define MOTOR_1 5
#define MOTOR_2 3
#define MOTOR_3 6
#define MOTOR_4 9

// Time in milliseconds to wait between flashing the default LED when blinkLED() is called.
#define DEFAULT_BLINK_INTERVAL 500

// Time in seconds the accelerometer data will be smoothed over, and amount of time gyro is trusted (approx.)
#define FILTER_TIME_CONSTANT .99f

#define AX_CORRECTION 12
#define AY_CORRECTION 25
#define AZ_CORRECTION 25

// Proportional Integral Derivative algorithm gains
#define ANGLE_PID_P_GAIN 2.9f
#define ANGLE_PID_I_GAIN 0.0f//.0002f
#define ANGLE_PID_D_GAIN 0.03f//.2f
#define RATE_PID_P_GAIN 0.05f
#define RATE_PID_I_GAIN 0.0001f
#define RATE_PID_D_GAIN 0.0f

#define MIN_LIFTOFF_THROTTLE 50

L3G gyro;
LSM303 compass;

float target_pitch = 0, target_roll = 0, target_yaw = 0;
float current_pitch = 0, current_roll = 0, current_yaw = 0;
float pitch_error_integrator = 0, roll_error_integrator = 0;
float pitch_rate_error_integrator = 0, roll_rate_error_integrator = 0;

float target_pitch_rate = 0, target_roll_rate = 0, target_yaw_rate = 0;

float ax, ay, az, gx, gy, gz;
float last_gx = 0, last_gy = 0, last_gz = 0;

// Offsets used to steer the quad. Increasing pitch bias will cause
// positive torque around the pitch axis
int pitch_motor_bias = 0, roll_motor_bias = 0, yaw_motor_bias = 0;

// Motor outputs will be set to throttle value +/- the bias values above
int throttle = 0;

long dT = 0;
long lastMicros;

int printCounter = 0;
int throttleCounter = 0;
int pauseTimer = 0;
// Stores the current PWM values for each motor starting with 1 and ending with 4
// Any time analogWrite is called on one of the motor pins, this array should be updated
// so that a call to get the current motor output value will return the correct value.
int MOTOR_OUTPUTS[4] = { 0, 0, 0, 0 };

boolean motors_enabled = false;


void setup()
{
	pinMode(GREEN_LED, OUTPUT);
	if (RED_LED > 0)
	{
		pinMode(RED_LED, OUTPUT);
	}
	
	pinMode(MOTOR_1, OUTPUT);
	pinMode(MOTOR_2, OUTPUT);
	pinMode(MOTOR_3, OUTPUT);
	pinMode(MOTOR_4, OUTPUT);

	LedTest();
	Wire.begin();
	gyro.init();
	//compass.init();
	Serial.begin(9600);
	
	//MotorTest(100);
	gyro.enableDefault();
	//compass.enableDefault();
	enableMotors();
	Thrust(60);
	setMotor(2, 100);
	lastMicros = micros();
}

void loop()
{
	
	
	// Read data from the gyro and accelerometer
	gyro.read();
	//compass.read();
	/*
	long currentMicros = micros();
	dT = currentMicros - lastMicros;
	lastMicros = currentMicros;

	calculateOrientation();
	doPID();

	if (current_pitch < 1 && current_pitch > -1 && current_roll > -1 && current_roll < 1)
	{
		digitalWrite(GREEN_LED, HIGH);
		digitalWrite(RED_LED, LOW);
	}
	else if (current_pitch > 20 || current_pitch < -20)
	{
		digitalWrite(GREEN_LED, LOW);
		digitalWrite(RED_LED, HIGH);
		motors_enabled = false;
	}
	else
	{
		digitalWrite(GREEN_LED, LOW);
		digitalWrite(RED_LED, LOW);
	}
	*/
	printGyroAndCompassData();
	blinkLed(1, 200);
}

void doPID()
{
	int pitch_error = current_pitch - target_pitch;
	int roll_error = current_roll - target_roll;

	pitch_error_integrator += pitch_error;
	roll_error_integrator += roll_error;

	int pitchCorrection = (pitch_error * ANGLE_PID_P_GAIN) + (pitch_error_integrator * ANGLE_PID_I_GAIN) + (ax * ANGLE_PID_D_GAIN);
	int rollCorrection = (roll_error * ANGLE_PID_P_GAIN) + (roll_error_integrator * ANGLE_PID_I_GAIN) + (ay * ANGLE_PID_D_GAIN);

	target_pitch_rate = pitchCorrection;
	target_roll_rate = -rollCorrection;

	int pitch_rate_error =  target_pitch_rate - gy;
	int roll_rate_error =   gx - target_roll_rate;
	int yaw_rate_error = target_yaw_rate - gz;

	// Prevent integration of error if the copter is not airborn, otherwise
	// if the quad is not perfectly level when landed it will veer off course 
	// once minimum takeoff throttle is reached
	if (throttle >= MIN_LIFTOFF_THROTTLE)
	{
		pitch_rate_error_integrator += pitch_rate_error;
		roll_rate_error_integrator += roll_rate_error;
	}
	

	int dgx = (gx - last_gx) / dT;
	int dgy = (gy - last_gy) / dT;
	int dgz = (gz - last_gz) / dT;

	int pitchRateCorrection = (pitch_rate_error * RATE_PID_P_GAIN) + (pitch_rate_error_integrator * RATE_PID_D_GAIN) + (dgx - RATE_PID_D_GAIN);
	int rollRateCorrection = (roll_rate_error * RATE_PID_P_GAIN) + (roll_rate_error_integrator * RATE_PID_D_GAIN) + (dgy - RATE_PID_D_GAIN);
	int yawRateCorrection = (yaw_rate_error * RATE_PID_P_GAIN);
	setMotorBias(pitchRateCorrection, rollRateCorrection, yawRateCorrection);
}

void calculateOrientation()
{
	

	// The gyro and accelerometer data in raw form has 4 unused bits, dividing by 16 yeilds 
	// the value in milli-degrees and milli-g's.
	 ax = (compass.a.x / 16) - AX_CORRECTION;
	 ay = (compass.a.y / 16) - AY_CORRECTION;
	 az = (compass.a.z / 16) - AZ_CORRECTION;
	 gx = gyro.g.x / 16;
	 gy = gyro.g.y / 16;
	 gz = gyro.g.z / 16;


	float accelerometer_pitch_estimate = RAD_TO_DEG * asin(ax / 1000);
	if (isnan(accelerometer_pitch_estimate))
		accelerometer_pitch_estimate = 90;

	float accelerometer_roll_estimate = RAD_TO_DEG * asin(ay / 1000);
	if (isnan(accelerometer_roll_estimate))
		accelerometer_roll_estimate = 90;

	float gyro_pitch_estimate = current_pitch + (gx * dT / 1000000000);
	float gyro_roll_estimate = current_roll + (gy * dT /   1000000000);

	current_roll = FILTER_TIME_CONSTANT * gyro_roll_estimate;
	current_roll += (1.0f - FILTER_TIME_CONSTANT) * accelerometer_roll_estimate;

	current_pitch = FILTER_TIME_CONSTANT * gyro_pitch_estimate;
	current_pitch += (1.0f - FILTER_TIME_CONSTANT) * accelerometer_pitch_estimate;

	bool print = false;
	if (printCounter > 100 && print)
	{
		Serial.print("Pitch: ");
		Serial.print(current_pitch);
		Serial.print("Roll: ");
		Serial.println(current_roll);
		printCounter = 0;
	}
	else printCounter++;
	
}

void setMotorBias(int pitchBias, int rollBias, int yawBias)
{
	if (throttle > MIN_LIFTOFF_THROTTLE)
	{
		pitch_motor_bias = pitchBias;
		roll_motor_bias = rollBias;

		int motor1out = throttle - pitchBias + rollBias + yawBias;
		int motor2out = throttle + pitchBias + rollBias - yawBias;
		int motor3out = throttle + pitchBias - rollBias + yawBias;
		int motor4out = throttle - pitchBias - rollBias - yawBias;

		motor1out = max(min(motor1out, 254), 0);
		motor2out = max(min(motor2out, 254), 0);
		motor3out = max(min(motor3out, 254), 0);
		motor4out = max(min(motor4out, 254), 0);

		setMotor(1, motor1out);
		setMotor(2, motor2out);
		setMotor(3, motor3out);
		setMotor(4, motor4out);
	}
}


int getIntInRange(int integer, int lower, int upper)
{
	return max(min(integer, upper), lower);
}

void printGyroAndCompassData()
{
	Serial.print("AX: ");
	Serial.print((compass.a.x / 16) - AX_CORRECTION);
	Serial.print("\t\tAY: ");
	Serial.print((compass.a.y / 16) - AY_CORRECTION);
	Serial.print("\t\t  AZ: ");
	Serial.print((compass.a.z / 16)- AZ_CORRECTION);
	Serial.print("\tGX: ");
	Serial.print(gyro.g.x / 16);
	Serial.print("\tGY: ");
	Serial.print(gyro.g.y / 16);
	Serial.print("\tGZ: ");
	Serial.println(gyro.g.z / 16);
}

// This method should be called, rather than analogWrite, when
// the current PWM value of one of the motors is to be set.
void setMotor(int motor, int value)
{
	// check to make sure method is being called on one of the 4 motors
	if (motor > 4 || motor < 1)
		return;

	MOTOR_OUTPUTS[motor - 1] = value;

	switch (motor)
	{
		case 1:
			analogWrite(MOTOR_1, value);
			break;
		case 2:
			analogWrite(MOTOR_2, value);
			break;
		case 3: 
			analogWrite(MOTOR_3, value);
			break;
		case 4:
			analogWrite(MOTOR_4, value);
			break;
	}

}

void setTargetPitch(float degrees)
{
	target_pitch = degrees;
}

void setTargetRoll(float degrees)
{
	target_roll = degrees;
}

void setTargetYaw(float degrees)
{
	target_yaw = degrees;
}

void LedTest()
{
	digitalWrite(GREEN_LED, HIGH);
	delay(1000);
	digitalWrite(GREEN_LED, LOW);
}

void MotorTest(int throttle)
{
	int delayInterval = 1000;
	int blinkDelay = 200;
	delay(delayInterval);
	blinkLed(1, blinkDelay);
	analogWrite(MOTOR_1, throttle);
	delay(delayInterval);
	analogWrite(MOTOR_1, 0);
	blinkLed(2, blinkDelay);
	analogWrite(MOTOR_2, throttle);
	delay(delayInterval);
	analogWrite(MOTOR_2, 0);
	blinkLed(3, blinkDelay);
	analogWrite(MOTOR_3, throttle);
	delay(delayInterval);
	analogWrite(MOTOR_3, 0);
	blinkLed(4, blinkDelay);
	analogWrite(MOTOR_4, throttle);
	delay(delayInterval);
	analogWrite(MOTOR_4, 0);
}

void enableMotors()
{
	motors_enabled = true;
}

void disableMotors()
{
	motors_enabled = false;
}

void Thrust(int thrust)
{
	if (thrust < 0 || thrust > 254)
		return;

	int output = 0;
	throttle = thrust;

	if (!motors_enabled)
	{
		output = 0;
	}
	else
	{
		output = throttle;
	}

	setMotor(1, output);
	setMotor(2, output);
	setMotor(3, output);
	setMotor(4, output);
}

void blinkLed(int times, int del)
{
	for (int i = 0; i < times; i++)
	{
		digitalWrite(GREEN_LED, HIGH);
		delay(del/4);
		digitalWrite(GREEN_LED, LOW);
		delay(del);
	}
}

#ifndef CAMERA_SERVO_STAB
#define CAMERA_SERVO_STAB
#include <Servo.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <EEWrap.h>
#include <ResponsiveAnalogRead.h>

class Axis {
public:
	double accel;
	double aimAccel;
	double servoStep;
	byte first_init;
	float servoAngle;
	double pid_kp;
	double pid_ki;
	double pid_kd;
	PID *pid;
	byte threshold;
	Servo servo;
	ResponsiveAnalogRead *input_accel;
	Axis() {
		pid = new PID(&accel, &servoStep, &aimAccel, pid_kp, pid_ki, pid_kd,
		REVERSE);
		pid->SetMode(AUTOMATIC);
		pid->SetOutputLimits(-90, 90);
		pid->SetSampleTime(20);
		input_accel = new ResponsiveAnalogRead(false);
		input_accel->setAverageAmount(10);
	}

	void update_pid_tun() {
		pid->SetTunings(pid_kp, pid_ki, pid_kd);
	}

	void update_data(int data) {
		input_accel->update(data);
		accel = input_accel->getValue();
	}

	void process_PID(bool servo_enable) {
		if (pid->Compute()) {
			if (servo_enable) {
				if (abs(accel - aimAccel) > threshold) {
					servoAngle += (float) servoStep;
					servoAngle = checkServoAngle(servoAngle);
					servo.write(round(servoAngle));
				}
			}
		}
	}
	float checkServoAngle(float servoAngle) {
		if (servoAngle > 180)
			servoAngle = 180;
		if (servoAngle < 0)
			servoAngle = 0;
		return servoAngle;
	}
};
#endif

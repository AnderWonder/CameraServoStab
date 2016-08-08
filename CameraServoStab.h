#ifndef CAMERA_SERVO_STAB
#define CAMERA_SERVO_STAB
#include <Servo.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <EEWrap.h>
#include <ResponsiveAnalogRead.h>

struct Axis_eemem {
	uint8_e first_init;
	float_e servoAngle;
	double_e pid_kp;
	double_e pid_ki;
	double_e pid_kd;
	double_e aim;
};

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
	Axis_eemem *eemem_data;
	Axis(Axis_eemem *eemem_data,byte threshold) {
		this->eemem_data=eemem_data;
		if (eemem_data->first_init != 5) {
			eemem_data->first_init = 5;
			eemem_data->servoAngle = 90;
			eemem_data->pid_kp = .015;
			eemem_data->pid_ki = 0;
			eemem_data->pid_kd = 0;
			eemem_data->aim = 0;
		}
		pid = new PID(&accel, &servoStep, &aimAccel, pid_kp, pid_ki, pid_kd,
		REVERSE);
		pid->SetMode(AUTOMATIC);
		pid->SetOutputLimits(-90, 90);
		pid->SetSampleTime(20);

		pid_kp = eemem_data->pid_kp;
		pid_ki = eemem_data->pid_ki;
		pid_kd = eemem_data->pid_kd;
		update_pid_tun();

		input_accel = new ResponsiveAnalogRead(false);
		input_accel->setAverageAmount(10);

		aimAccel = eemem_data->aim;
		servoAngle = eemem_data->servoAngle;



		this->threshold = threshold;
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

	void save_pid_tun(){
		eemem_data->pid_kp = pid_kp;
		eemem_data->pid_ki = pid_ki;
		eemem_data->pid_kd = pid_kd;
	}
};
#endif

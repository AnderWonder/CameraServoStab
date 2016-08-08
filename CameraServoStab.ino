#include <Adafruit_ADXL345_U.h>
#include <Servo.h>
#include <PID_v1.h>
#include <se8r01.h>
#include <CmdParser.hpp>
#include <Thread.h>
#include <ResponsiveAnalogRead.h>
#include <PinChangeInterrupt.h>
#include <EEPROM.h>
#include <EEWrap.h>
#include "CameraServoStab.h"

#define AIM_FOR_X 0
#define AIM_FOR_Z -15
#define THRESHOLD_Z 0
#define THRESHOLD_X 1
#define START_STATE GO_XZ
#define LONG_PRESS_DELAY 2000
#define Y_SPEED_FAST 5
#define Y_SPEED_MIDLE 10
#define Y_SPEED_SLOW 50

bool show_info1 = false;
bool show_info2 = false;
bool show_info3 = false;
bool pid_frontend = false;

Thread serial_cmd = Thread();

Thread get_accel_data = Thread();

Thread get_angle_Y = Thread();

Thread serial_info = Thread();

Thread radio_cmd = Thread();

Thread Y_moving = Thread();

Thread X_moving = Thread();

Thread pid_frontend_processing = Thread();

ResponsiveAnalogRead accel_Z(false, .01);

CmdParser cmdParser;

Servo servoZ, servoY;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double accelZ;
double aimAccelZ;
int16_e aimZ EEMEM;
int16_e servoAngleZ EEMEM;
double servoStepZ;
PID Z_Pid(&accelZ, &servoStepZ, &aimAccelZ, .01, 0, 0.0005, REVERSE);

Axis_eemem axis_x_eemem EEMEM;
Axis axis_x(&axis_x_eemem,THRESHOLD_X);

byte thrX = THRESHOLD_X, thrZ = THRESHOLD_Z;

int Y_moving_speed;
int Y_moving_direction;
int aimAngleY;
bool Y_moving_start = true;
uint8_e eeprom_writen EEMEM;
int16_e servoAngleY EEMEM;
uint8_e Y_preset_1 EEMEM;
uint8_e Y_preset_2 EEMEM;
uint8_e Y_preset_3 EEMEM;

int X_moving_speed = 0;
int X_moving_direction = 0;
bool X_moving_start = true;

enum STATE {
	GO_XZ, GO_Z, GO_X, STOP
};
int state = START_STATE;
int prev_state = START_STATE;

int kf = 100;

byte rx_data[] = { "-10;-10;0;0;0;0;" };

bool got_radio = false;

int button_0_state = 0;
unsigned long button_0_time;
int button_1_state = 0;
unsigned long button_1_time;
int button_2_state = 0;
unsigned long button_2_time;
int button_3_state = 0;
unsigned long button_3_time;

double *Setpoint, *Input, *Output;
Axis *frontend_axis;

//********************************* THREADS
void get_accel_Data() {
	axis_x.update_data(accel.getX());
	accel_Z.update(accel.getZ());
	accelZ = accel_Z.getValue();
}

void serial_Cmd() {

	if (Serial.available()) {

		String buffer_string = Serial.readString();

		pid_frontend_serial_receive(&buffer_string);

		if (cmdParser.parseCmd((char*) buffer_string.c_str()) != CMDPARSER_ERROR) {

			if (cmdParser.equalCommand_P(PSTR("go")))
				state = GO_XZ;

			if (cmdParser.equalCommand_P(PSTR("st")))
				state = STOP;

			if (cmdParser.equalCommand_P(PSTR("pid"))) {
				PID *pid;
				pid = NULL;
				if (cmdParser.equalCmdParam_P(1, PSTR("X")))
					pid = axis_x.pid;
				if (cmdParser.equalCmdParam_P(1, PSTR("Z")))
					pid = &Z_Pid;
				if (cmdParser.equalCmdParam_P(1, PSTR("kf"))) {
					kf =
							String(cmdParser.getValueFromKey(cmdParser.getCmdParam(1))).toInt();
					Serial.println(String("kf=") + kf);
				}
				if (cmdParser.equalCmdParam_P(1, PSTR("fr"))) {
					pid_frontend = !pid_frontend;
				}
				/*
				 if (pid != NULL) {
				 String k(cmdParser.getCmdParam(2));
				 float k_val = String(cmdParser.getValueFromKey(k.c_str())).toFloat();
				 k_val /= kf;
				 if (k == "kp") {
				 pid->SetTunings(k_val, pid->GetKi(), pid->GetKd());
				 }
				 if (k == "kd") {
				 pid->SetTunings(pid->GetKp(), pid->GetKi(), k_val);
				 }
				 Serial.println(
				 String("PID: kf = ") + kf + "; kp = " + pid->GetKp() * kf
				 + "; kd = " + pid->GetKd() * kf);
				 }
				 */
			}

			if (cmdParser.equalCommand_P(PSTR("info1"))) {
				show_info1 = !show_info1;
			}

			if (cmdParser.equalCommand_P(PSTR("info2"))) {
				show_info2 = !show_info2;
			}

			if (cmdParser.equalCommand_P(PSTR("info3"))) {
				show_info3 = !show_info3;
			}

		}

	}

}

void serial_Info() {
	if (show_info1) {
		Serial.println(String("accel_x: ") + axis_x.accel + "; accel_z: " + accelZ);
	}
	if (show_info2) {
	}
	if (show_info3) {
		Serial.print("Received by radio:");
		Serial.println(String((char*) rx_data));
		show_info3 = false;
	}
}

void radio_Cmd() {
	if (got_radio) {

		String radio_cmd = String((char*) rx_data);

		int Y_moving_val = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		Y_moving_speed = abs(Y_moving_val);
		if (Y_moving_speed > 1) {

			aimAngleY = Y_moving_val > 0 ? 180 : 0;

			if (Y_moving_speed > 8)
				Y_moving_speed = Y_SPEED_FAST;
			else if (Y_moving_speed > 4)
				Y_moving_speed = Y_SPEED_MIDLE;
			else if (Y_moving_speed > 1)
				Y_moving_speed = Y_SPEED_SLOW;

			Y_moving.setInterval(Y_moving_speed);
		}
		else {
			aimAngleY = servoAngleY;

		}

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int X_moving_val = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		X_moving_direction = X_moving_val >= 0 ? 1 : -1;
		X_moving_speed = abs(X_moving_val);
		if (X_moving_speed > 7) {
			if (X_moving_speed > 7) {
				X_moving.setInterval(5);
			}
			else
				X_moving.setInterval(25);
			X_moving.enabled = true;
		}
		else {
			X_moving.enabled = false;
		}

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_0_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_1_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_2_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_3_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();

		if (button_0_now != button_0_state) {
			if (button_0_now) { //pressed
				button_0_time = millis();
			}
			else { //released
				if (millis() - button_0_time > LONG_PRESS_DELAY) {

				}
				else {
					switch (state) {
					case GO_XZ:
						state = GO_Z;
						break;
					case GO_Z:
						state = STOP;
						break;
					case STOP:
						axis_x.aimAccel = axis_x.accel;
						if (button_1_now) {
							aimAccelZ = aimZ = accelZ;
							button_1_now = button_1_state = 0;
						}
						state = GO_XZ;
					}
				}
			}
		}
		button_0_state = button_0_now;

		if (button_1_now != button_1_state) {
			if (button_1_now) { //pressed
				button_1_time = millis();
			}
			else { //released
				if (millis() - button_1_time > LONG_PRESS_DELAY) {
					Y_preset_1 = servoAngleY;
				}
				else {
					aimAngleY = Y_preset_1;
					Y_moving.setInterval(Y_SPEED_FAST);
				}
			}
		}
		button_1_state = button_1_now;

		if (button_2_now != button_2_state) {
			if (button_2_now) { //pressed
				button_2_time = millis();
			}
			else { //released
				if (millis() - button_2_time > LONG_PRESS_DELAY) {
					Y_preset_2 = servoAngleY;
				}
				else {
					aimAngleY = Y_preset_2;
					Y_moving.setInterval(Y_SPEED_FAST);
				}
			}
		}
		button_2_state = button_2_now;

		if (button_3_now != button_3_state) {
			if (button_3_now) { //pressed
				button_3_time = millis();
			}
			else { //released
				if (millis() - button_3_time > LONG_PRESS_DELAY) {
					Y_preset_3 = servoAngleY;
				}
				else {
					aimAngleY = Y_preset_3;
					Y_moving.setInterval(Y_SPEED_FAST);
				}
			}
		}
		button_3_state = button_3_now;

		got_radio = false;

	}
}

void Y_Moving() {
	char Y_moving_direction = 0;
	if (servoAngleY != aimAngleY) {
		if (Y_moving_start) {
			prev_state = state;
			state = STOP;
			Y_moving_start = false;
		}

		if (aimAngleY > servoAngleY)
			Y_moving_direction = 1;
		else if (aimAngleY < servoAngleY)
			Y_moving_direction = -1;
		servoAngleY += Y_moving_direction;
		servoAngleY = checkServoAngle(servoAngleY);
		servoY.write(servoAngleY);
	}
	else {
		if (!Y_moving_start) {
			state = prev_state;
			Y_moving_start = true;
		}
	}
}

void X_Moving() {
	if (state == GO_XZ || state == GO_X) {
		axis_x.aimAccel += X_moving_direction;
		if (axis_x.aimAccel > 250)
			axis_x.aimAccel = 250;
		if (axis_x.aimAccel < -250)
			axis_x.aimAccel = -250;
	}
	else {
		axis_x.servoAngle -= X_moving_direction;
		axis_x.servoAngle = checkServoAngle(axis_x.servoAngle);
		axis_x.servo.write(axis_x.servoAngle);
		axis_x.aimAccel = axis_x.accel;
	}
}

void radio_ISR() {
	if (digitalRead(IRQ_pin) == LOW) {
		getRxData(rx_data);
		got_radio = true;
	}
}

void pid_frontend_Processing() {
	if (pid_frontend)
		pid_frontend_serial_send();
}
//********************************* THREADS

void init_radio() {
	if (!init_rf(10, 9, 8, sizeof(rx_data))) {
		Serial.println("Radio chip not found!");
	}
	else {
		Serial.println("Radio connected");
		changeMode(RX_MODE);
		setChannel(110);
	}
}

void initThreads() {

	serial_info.onRun(serial_Info);
	serial_info.setInterval(100);

	serial_cmd.onRun(serial_Cmd);
	serial_cmd.setInterval(200);

	radio_cmd.onRun(radio_Cmd);
	radio_cmd.setInterval(100);

	get_accel_data.onRun(get_accel_Data);
	get_accel_data.setInterval(1);

	Y_moving.onRun(Y_Moving);
	Y_moving.setInterval(50);

	X_moving.onRun(X_Moving);
	X_moving.setInterval(100);
	X_moving.enabled = false;

	pid_frontend_processing.onRun(pid_frontend_Processing);
	pid_frontend_processing.setInterval(50);
}

void connect_PID_frontend(Axis *axis) {
	frontend_axis = axis;
	Setpoint = &axis->aimAccel;
	Input = &axis->accel;
	Output = &axis->servoStep;
}

void setup() {

	Serial.begin(9600);
	Serial.println("Program start");

	if (eeprom_writen != 9) {
		servoAngleZ = 90;
		Y_preset_1 = 0;
		Y_preset_2 = 90;
		Y_preset_3 = 180;
		servoAngleY = 90;
		aimAccelZ = aimZ = AIM_FOR_Z;
		eeprom_writen = 9;
		Serial.println("Init eeprom");
	}
	aimAngleY = servoAngleY;
	aimAccelZ = aimZ;

/*
if (axis_x_eemem.first_init != 2) {
		axis_x_eemem.first_init = 2;
		axis_x_eemem.servoAngle = 90;
		axis_x_eemem.pid_kp = .015;
		axis_x_eemem.pid_ki = 0;
		axis_x_eemem.pid_kd = 0;
	}
	axis_x.servoAngle = axis_x_eemem.servoAngle;
	axis_x.pid_kp = axis_x_eemem.pid_kp;
	axis_x.pid_ki = axis_x_eemem.pid_ki;
	axis_x.pid_kd = axis_x_eemem.pid_kd;
	axis_x.update_pid_tun();
	*/
	axis_x.servo.attach(5);
	axis_x.servo.write(axis_x.servoAngle);
	//axis_x.threshold = THRESHOLD_X;


	servoZ.attach(6);
	servoZ.write(servoAngleZ);

	servoY.attach(7);
	servoY.write(servoAngleY);

	Z_Pid.SetMode(AUTOMATIC);
	Z_Pid.SetOutputLimits(-90, 90);
	Z_Pid.SetSampleTime(20);

	accel_Z.setAverageAmount(10);

	delay(500);

	if (!accel.begin()) {
		Serial.println("No ADXL345 detected ...");
		while (1)
			;
	}
	accel.setRange(ADXL345_RANGE_2_G);
	accel.setDataRate(ADXL345_DATARATE_3200_HZ);

	initThreads();

	cmdParser.setOptKeyValue(true);

	init_radio();
	attachPCINT(digitalPinToPCINT(IRQ_pin), radio_ISR, CHANGE);

	connect_PID_frontend(&axis_x);
}

void loop() {

	if (serial_cmd.shouldRun())
		serial_cmd.run();

	if (get_accel_data.shouldRun())
		get_accel_data.run();

	if (Z_Pid.Compute()) {
		if (state == GO_XZ || state == GO_Z) {
			if (abs(accelZ-aimAccelZ) > thrZ) {
				servoAngleZ += servoStepZ;
				servoAngleZ = checkServoAngle(servoAngleZ);
				servoZ.write(servoAngleZ);
			}
		}
	}

	axis_x.process_PID(state == GO_XZ || state == GO_X);

	if (serial_info.shouldRun())
		serial_info.run();

	if (Y_moving.shouldRun())
		Y_moving.run();

	if (X_moving.shouldRun())
		X_moving.run();

	if (radio_cmd.shouldRun())
		radio_cmd.run();

	if (pid_frontend_processing.shouldRun())
		pid_frontend_processing.run();
}

float checkServoAngle(float servoAngle) {
	if (servoAngle > 180)
		servoAngle = 180;
	if (servoAngle < 0)
		servoAngle = 0;
	return servoAngle;
}

//*********************************************************PID FRONTEND
/********************************************
 * Serial Communication functions / helpers
 ********************************************/

union {                // This Data structure lets
	byte asBytes[24];    // us take the byte array
	float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1-4: float setpoint
//  5-8: float input
//  9-12: float output
//  13-16: float P_Param
//  17-20: float I_Param
//  21-24: float D_Param
void pid_frontend_serial_receive(String *buffer) {
	if (buffer->length() == 25) {
		//buffer->getBytes((unsigned char*)(foo.asBytes),24,1);
		for (byte i = 1; i < 25; i++)
			foo.asBytes[i - 1] = byte((*buffer)[i]);
		// if the information we got was in the correct format,
		// read it into the system
		byte Auto_Man = (*buffer)[0];
		if (Auto_Man == 0 || Auto_Man == 1) {
			*Setpoint = double(foo.asFloat[0]);
			//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
			//   value of "Input"  in most cases (as
			//   in this one) this is not needed.
			if (Auto_Man == 0)                // * only change the output if we are in
					{                            //   manual mode.  otherwise we'll get an
				*Output = double(foo.asFloat[2]); //   output blip, then the controller will
			}                                     //   overwrite.

			frontend_axis->pid_kp = double(foo.asFloat[3]) / kf;           //
			frontend_axis->pid_ki = double(foo.asFloat[4]) / kf;           //
			frontend_axis->pid_kd = double(foo.asFloat[5]) / kf;

			frontend_axis->save_pid_tun();
			//axis_x_eemem.pid_kp = frontend_axis->pid_kp;
			//axis_x_eemem.pid_ki = frontend_axis->pid_ki;
			//axis_x_eemem.pid_kd = frontend_axis->pid_kd;

			frontend_axis->update_pid_tun();

			if (Auto_Man == 0)
				frontend_axis->pid->SetMode(MANUAL);        // * set the controller mode
			else
				frontend_axis->pid->SetMode(AUTOMATIC);             //
		}

	}

}

void pid_frontend_serial_send() {
	Serial.print("PID ");
	Serial.print(*Setpoint);
	Serial.print(" ");
	Serial.print(*Input);
	Serial.print(" ");
	Serial.print(*Output);
	Serial.print(" ");
	Serial.print(frontend_axis->pid->GetKp() * kf);
	Serial.print(" ");
	Serial.print(frontend_axis->pid->GetKi() * kf);
	Serial.print(" ");
	Serial.print(frontend_axis->pid->GetKd() * kf);
	Serial.print(" ");
	if (frontend_axis->pid->GetMode() == AUTOMATIC)
		Serial.println("Automatic");
	else
		Serial.println("Manual");
}

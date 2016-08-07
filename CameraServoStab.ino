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

Thread serial_cmd = Thread();

Thread get_accel_data = Thread();

Thread get_angle_Y = Thread();

Thread serial_info = Thread();

Thread radio_cmd = Thread();

Thread Y_moving = Thread();

Thread X_moving = Thread();

Thread pid_frontend_processing = Thread();

ResponsiveAnalogRead accel_X(false, .01);
ResponsiveAnalogRead accel_Z(false, .01);

CmdParser cmdParser;

Servo servoZ, servoY, servoX;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double accelZ;
int16_e aimZ EEMEM;
double aimAccelZ;
int16_e servoAngleZ EEMEM;
double servoStepZ;
PID Z_Pid(&accelZ, &servoStepZ, &aimAccelZ, .023, 0, 0.0005, REVERSE);



Axis axis_x;

double accelX;
double aimAccelX = AIM_FOR_X;
float_e servoAngleX EEMEM;
float servoAngleX_f;
double servoStepX;
PID X_Pid(&accelX, &servoStepX, &aimAccelX, .015, 0.0008, 0.002, REVERSE);

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

//**************** PID FRONTEND PROCESSING
//Define Variables we'll be connecting to
double *Setpoint, *Input, *Output;

//Specify the links and initial tuning parameters
PID *myPID;
//**************** PID FRONTEND PROCESSING

//********************************* THREADS
void get_accel_Data() {
	accel_X.update(accel.getX());
	accel_Z.update(accel.getZ());
	accelZ = accel_Z.getValue();
	axis_x.accel = accel_Z.getValue();
	//accelX = accel_X.getValue();
}

void serial_Cmd() {

	CmdBuffer<32> cmdBuffer;

	cmdBuffer.setEndChar(CMDBUFFER_CHAR_CR);

	if (cmdBuffer.readFromSerial(&Serial, 1)) {

		if (cmdParser.parseCmd(&cmdBuffer) != CMDPARSER_ERROR) {

			if (cmdParser.equalCommand("go"))
				state = GO_XZ;

			if (cmdParser.equalCommand("st"))
				state = STOP;

			if (cmdParser.equalCommand("pid")) {
				PID *pid;
				pid = NULL;
				if (cmdParser.equalCmdParam(1, "X"))
					pid = &X_Pid;
				if (cmdParser.equalCmdParam(1, "Z"))
					pid = &Z_Pid;
				if (cmdParser.equalCmdParam(1, "kf")) {
					kf =
							String(cmdParser.getValueFromKey(cmdParser.getCmdParam(1))).toInt();
					Serial.println(String("kf=") + kf);
				}
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
			}

			if (cmdParser.equalCommand("info1")) {
				show_info1 = !show_info1;
			}

			if (cmdParser.equalCommand("info2")) {
				show_info2 = !show_info2;
			}

			if (cmdParser.equalCommand("info3")) {
				show_info3 = !show_info3;
			}

			if (cmdParser.equalCommand("aim")) {
				int aim_val = String(
						cmdParser.getValueFromKey(cmdParser.getCmdParam(1))).toInt();
				if (cmdParser.equalCmdParam(1, "X")) {
					aimAccelX = aim_val;
				}
				if (cmdParser.equalCmdParam(1, "Z")) {
					aimAccelZ = aim_val;
				}
			}

			/*
			 if (command == "go") {
			 state = GO;
			 }
			 if (command == "st") {
			 state = STOP;
			 }
			 int k = 10;
			 if (command == "kpz") {
			 kp = Serial.parseFloat() / k;
			 Z_Pid.SetTunings(kp, Z_Pid.GetKi(), Z_Pid.GetKd());
			 command = "infz";
			 }
			 if (command == "kiz") {
			 ki = Serial.parseFloat() / k;
			 Z_Pid.SetTunings(Z_Pid.GetKp(), ki, Z_Pid.GetKd());
			 command = "infz";
			 }
			 if (command == "kdz") {
			 kd = Serial.parseFloat() / k;
			 Z_Pid.SetTunings(Z_Pid.GetKp(), Z_Pid.GetKi(), kd);
			 command = "infz";
			 }
			 if (command == "thz") {
			 thrZ = Serial.parseInt();
			 }
			 if (command == "infz") {
			 Serial.print("Kp=");
			 Serial.print(Z_Pid.GetKp() * k);
			 Serial.print(";");
			 Serial.print("Ki=");
			 Serial.print(Z_Pid.GetKi() * k);
			 Serial.print(";");
			 Serial.print("Kd=");
			 Serial.println(Z_Pid.GetKd() * k);
			 }
			 if (command == "kpx") {
			 kp = Serial.parseFloat() / k;
			 X_Pid.SetTunings(kp, X_Pid.GetKi(), X_Pid.GetKd());
			 command = "infx";
			 }
			 if (command == "kix") {
			 ki = Serial.parseFloat() / k;
			 X_Pid.SetTunings(X_Pid.GetKp(), ki, X_Pid.GetKd());
			 command = "infx";
			 }
			 if (command == "kdx") {
			 kd = Serial.parseFloat() / k;
			 X_Pid.SetTunings(X_Pid.GetKp(), X_Pid.GetKi(), kd);
			 command = "infx";
			 }
			 if (command == "thx") {
			 thrX = Serial.parseInt();
			 }
			 if (command == "infx") {
			 Serial.print("Kp=");
			 Serial.print(X_Pid.GetKp() * k);
			 Serial.print(";");
			 Serial.print("Ki=");
			 Serial.print(X_Pid.GetKi() * k);
			 Serial.print(";");
			 Serial.print("Kd=");
			 Serial.println(X_Pid.GetKd() * k);
			 }
			 */

		}
		else
			Serial.println("Error parsing cmd!");

	}

}

void serial_Info() {
	if (show_info1) {
		Serial.println(String("accel_x: ") + accelX + "; accel_z: " + accelZ);
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
						aimAccelX = accelX;
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
		//aimAccelX += X_moving_direction;
		axis_x.aimAccel += X_moving_direction;
		if (axis_x.aimAccel > 250)
			axis_x.aimAccel = 250;
		if (axis_x.aimAccel < -250)
			axis_x.aimAccel = -250;
	}
	else {
		axis_x.servoAngle -= X_moving_direction;
		axis_x.servoAngle = checkServoAngle(axis_x.servoAngle);
		servoX.write(axis_x.servoAngle);
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
	SerialReceive();
	SerialSend();
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
	myPID = axis->pid;
	Setpoint = &axis->aimAccel;
	Input = &axis->accel;
	Output = &axis->servoStep;
}

void setup() {

	Serial.begin(9600);
	Serial.println("Program start");

	if (eeprom_writen != 1) {
		axis_x.servoAngle = 90;
		//servoAngleX = 90;
		servoAngleZ = 90;
		Y_preset_1 = 0;
		Y_preset_2 = 90;
		Y_preset_3 = 180;
		servoAngleY = 90;
		aimAccelZ = aimZ = AIM_FOR_Z;
		eeprom_writen = 1;
		Serial.println("Init eeprom");
	}
	aimAngleY = servoAngleY;
	aimAccelZ = aimZ;

	//servoX.attach(5);
	//servoX.write(axis_x.servoAngle);
	axis_x.servo.attach(5);
	axis_x.servo.write(axis_x.servoAngle);

	servoZ.attach(6);
	servoZ.write(servoAngleZ);

	servoY.attach(7);
	servoY.write(servoAngleY);

	Z_Pid.SetMode(AUTOMATIC);
	Z_Pid.SetOutputLimits(-90, 90);
	Z_Pid.SetSampleTime(20);

	//X_Pid.SetMode(AUTOMATIC);
	//X_Pid.SetOutputLimits(-90, 90);
	//X_Pid.SetSampleTime(20);

	accel_X.setAverageAmount(10);
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

	/*
	 if (axis_x.pid.Compute()) {
	 if (state == GO_XZ || state == GO_X) {
	 if (abs(axis_x.accel-axis_x.aimAccel) > axis_x.threshold) {
	 axis_x.servoAngle += (float) axis_x.servoStep;
	 axis_x.servoAngle = checkServoAngle(axis_x.servoAngle);
	 axis_x.servo.write(round(axis_x.servoAngle));
	 }
	 }
	 }
	 */

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

//**************** PID FRONTEND PROCESSING
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
void SerialReceive() {

	// read the bytes sent from Processing
	int index = 0;
	byte Auto_Man = -1;
	while (Serial.available() && index < 25) {
		if (index == 0)
			Auto_Man = Serial.read();
		else
			foo.asBytes[index - 1] = Serial.read();
		index++;
	}

	// if the information we got was in the correct format,
	// read it into the system
	if (index == 25 && (Auto_Man == 0 || Auto_Man == 1)) {
		*Setpoint = double(foo.asFloat[0]);
		//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
		//   value of "Input"  in most cases (as
		//   in this one) this is not needed.
		if (Auto_Man == 0)                  // * only change the output if we are in
				{                              //   manual mode.  otherwise we'll get an
			*Output = double(foo.asFloat[2]); //   output blip, then the controller will
		}                                     //   overwrite.

		double p, i, d;                  // * read in and set the controller tunings
		p = double(foo.asFloat[3]);           //
		i = double(foo.asFloat[4]);           //
		d = double(foo.asFloat[5]);           //
		myPID->SetTunings(p / kf, i / kf, d / kf);            //

		if (Auto_Man == 0)
			myPID->SetMode(MANUAL);            // * set the controller mode
		else
			myPID->SetMode(AUTOMATIC);             //
	}
	Serial.flush();              // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend() {
	Serial.print("PID ");
	Serial.print(*Setpoint);
	Serial.print(" ");
	Serial.print(*Input);
	Serial.print(" ");
	Serial.print(*Output);
	Serial.print(" ");
	Serial.print(myPID->GetKp() * kf);
	Serial.print(" ");
	Serial.print(myPID->GetKi() * kf);
	Serial.print(" ");
	Serial.print(myPID->GetKd() * kf);
	Serial.print(" ");
	if (myPID->GetMode() == AUTOMATIC)
		Serial.println("Automatic");
	else
		Serial.println("Manual");
}

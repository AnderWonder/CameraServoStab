#include <Adafruit_ADXL345_U.h>
#include <Servo.h>
#include <PID_v1.h>
#include <se8r01.h>
#include <CmdParser.hpp>
#include <Thread.h>
#include <ResponsiveAnalogRead.h>
#include <PinChangeInterrupt.h>

#define AIM_FOR_X 0
#define AIM_FOR_Z -20
#define THRESHOLD_Z 3
#define THRESHOLD_X 3
#define START_STATE GO
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

ResponsiveAnalogRead accel_X(false, .01);
ResponsiveAnalogRead accel_Z(false, .01);

CmdParser cmdParser;

Servo servoZ, servoY, servoX;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double accelZ;
double aimAccelZ = AIM_FOR_Z;
double servoAngleZ = 90, servoStepZ;
PID Z_Pid(&accelZ, &servoStepZ, &aimAccelZ, .02, 0, 0.0005, REVERSE);

double accelX;
double aimAccelX = AIM_FOR_X;
double servoAngleX = 90, servoStepX;
PID X_Pid(&accelX, &servoStepX, &aimAccelX, .02, 0, 0.0005, REVERSE);

int Y_moving_speed = 0;
int Y_moving_direction = 0;
int servoAngleY = 90;
int aimAngleY = 90;
byte Y_preset_1=0;
byte Y_preset_2=90;
byte Y_preset_3=180;

int X_moving_speed = 0;
int X_moving_direction = 0;

byte thrX = THRESHOLD_X, thrZ = THRESHOLD_Z;

enum STATE {
	GO, STOP
};
int state = START_STATE;

int kf = 1;

byte rx_data[] = { "-10;-10;0;0;0;0;" };

bool got_radio = false;

int button_0_state = 0;
int button_1_state = 0;
unsigned long button_1_time;
int button_2_state = 0;
unsigned long button_2_time;
int button_3_state = 0;
unsigned long button_3_time;

void init_radio() {
	if (!init_rf(10, 7, 8, sizeof(rx_data))) {
		Serial.println("Radio chip not found!");
	}
	else {
		Serial.println("Radio connected");
		changeMode(RX_MODE);
		setChannel(110);
	}
}

//********************************* THREADS
void get_accel_Data() {
	accel_X.update(accel.getX());
	accel_Z.update(accel.getZ());
	accelZ = accel_Z.getValue();
	accelX = accel_X.getValue();
}

void serial_Cmd() {

	CmdBuffer<32> cmdBuffer;

	cmdBuffer.setEndChar(CMDBUFFER_CHAR_CR);

	if (cmdBuffer.readFromSerial(&Serial, 1)) {

		if (cmdParser.parseCmd(&cmdBuffer) != CMDPARSER_ERROR) {

			if (cmdParser.equalCommand("go"))
				state = GO;

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
			aimAngleY = Y_moving_val > 0 ? 0 : 180;

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
		X_moving_direction = X_moving_val >= 0 ? -5 : 5;
		X_moving_speed = abs(X_moving_val);
		if (X_moving_speed > 2) {
			if (X_moving_speed > 7) {
				X_moving.setInterval(20);
			}
			else
				X_moving.setInterval(50);
			X_moving.enabled = true;
		}
		else {
			X_moving.enabled = false;
		}

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_0_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		if (button_0_state && !button_0_now) { //clicked
			if (state == GO)
				state = STOP;
			else {
				aimAccelX = accelX;
				state = GO;
			}
		}
		button_0_state = button_0_now;

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_1_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		if (button_1_now != button_1_state) {
			if (button_1_now) { //pressed
				button_1_time = millis();
			}
			else { //released
				if (millis() - button_1_time > LONG_PRESS_DELAY) {
					Y_preset_1=servoAngleY;
				}
				else {
					aimAngleY=Y_preset_1;
					Y_moving.setInterval(Y_SPEED_FAST);
				}
			}
		}
		button_1_state = button_1_now;

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_2_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		if (button_2_now != button_2_state) {
			if (button_2_now) { //pressed
				button_2_time = millis();
			}
			else { //released
				if (millis() - button_2_time > LONG_PRESS_DELAY) {
					Y_preset_2=servoAngleY;
				}
				else {
					aimAngleY=Y_preset_2;
					Y_moving.setInterval(Y_SPEED_FAST);
				}
			}
		}
		button_2_state = button_2_now;

		radio_cmd = radio_cmd.substring(radio_cmd.indexOf(';') + 1);
		int button_3_now = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		if (button_3_now != button_3_state) {
			if (button_3_now) { //pressed
				button_3_time = millis();
			}
			else { //released
				if (millis() - button_3_time > LONG_PRESS_DELAY) {
					Y_preset_3=servoAngleY;
				}
				else {
					aimAngleY=Y_preset_3;
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
	if (aimAngleY > servoAngleY)
		Y_moving_direction = 1;
	else if (aimAngleY < servoAngleY)
		Y_moving_direction = -1;
	servoAngleY += Y_moving_direction;
	servoAngleY = checkServoAngle(servoAngleY);
	servoY.write(servoAngleY);
}

void X_Moving() {
	aimAccelX += X_moving_direction;
	if (aimAccelX > 250)
		aimAccelX = 250;
	if (aimAccelX < -250)
		aimAccelX = -250;
}

void radio_ISR() {
	if (digitalRead(IRQ_pin) == LOW) {
		getRxData(rx_data);
		got_radio = true;
	}
}

//********************************* THREADS

void setup() {
	Serial.begin(9600);
	Serial.println("Program start");

	servoX.attach(2);
	servoZ.attach(3);
	servoY.attach(4);

	Z_Pid.SetMode(AUTOMATIC);
	Z_Pid.SetOutputLimits(-90, 90);
	Z_Pid.SetSampleTime(20);

	X_Pid.SetMode(AUTOMATIC);
	X_Pid.SetOutputLimits(-90, 90);
	X_Pid.SetSampleTime(20);

	delay(500);

	if (!accel.begin()) {
		Serial.println("No ADXL345 detected ...");
		while (1)
			;
	}
	accel.setRange(ADXL345_RANGE_2_G);
	accel.setDataRate(ADXL345_DATARATE_3200_HZ);

	serial_cmd.onRun(serial_Cmd);
	serial_cmd.setInterval(200);

	get_accel_data.onRun(get_accel_Data);
	get_accel_data.setInterval(1);

	serial_info.onRun(serial_Info);
	serial_info.setInterval(100);

	Y_moving.onRun(Y_Moving);
	Y_moving.setInterval(50);

	X_moving.onRun(X_Moving);
	X_moving.setInterval(100);
	X_moving.enabled = false;

	radio_cmd.onRun(radio_Cmd);
	radio_cmd.setInterval(100);

	cmdParser.setOptKeyValue(true);

	accel_X.setAverageAmount(10);
	accel_Z.setAverageAmount(10);

	init_radio();
	attachPCINT(digitalPinToPCINT(8), radio_ISR, CHANGE);
}

void loop() {

	if (serial_cmd.shouldRun())
		serial_cmd.run();

	if (get_accel_data.shouldRun())
		get_accel_data.run();

	if (Z_Pid.Compute()) {
		if (state == GO) {
			if (abs(accelZ-aimAccelZ) > thrZ) {
				servoAngleZ += servoStepZ;
				servoAngleZ = checkServoAngle(servoAngleZ);
				servoZ.write(servoAngleZ);
			}
		}
	}

	if (X_Pid.Compute()) {
		if (state == GO) {
			if (abs(accelX-aimAccelX) > thrX) {
				servoAngleX += servoStepX;
				servoAngleX = checkServoAngle(servoAngleX);
				servoX.write(servoAngleX);
			}
		}
	}

	if (serial_info.shouldRun())
		serial_info.run();

	if (Y_moving.shouldRun())
		Y_moving.run();

	if (X_moving.shouldRun())
		X_moving.run();

	if (radio_cmd.shouldRun())
		radio_cmd.run();
}

double checkServoAngle(double servoAngle) {
	if (servoAngle > 180)
		servoAngle = 180;
	if (servoAngle < 0)
		servoAngle = 0;
	return servoAngle;
}


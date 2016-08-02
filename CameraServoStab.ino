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

bool show_info1 = false;
bool show_info2 = false;
bool show_info3 = false;

Thread serial_cmd = Thread();

Thread get_accel_data = Thread();

Thread get_angle_Y = Thread();

Thread serial_info = Thread();

Thread radio_cmd = Thread();

Thread Y_moving = Thread();

ResponsiveAnalogRead accel_X(false, .01);
ResponsiveAnalogRead accel_Z(false, .01);
ResponsiveAnalogRead angle_Y(A7, false, .01);

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
PID X_Pid(&accelX, &servoStepX, &aimAccelX, .03, 0, 0.0005, REVERSE);

int angleY = 0;
int Y_moving_speed = 0;
int Y_moving_direction = 0;
int servoAngleY = 90;

byte thrX = THRESHOLD_X, thrZ = THRESHOLD_Z;

enum STATE {
	GO, STOP
};
int state = START_STATE;

int kf = 1;

byte rx_data[] = { "-10;-10;0" };

bool got_radio = false;

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
		Serial.println(String("angle_y: ") + angleY);
	}
	if (show_info3) {
		Serial.print("Received by radio:");
		Serial.println(String((char*) rx_data));
		show_info3 = false;
	}
}

void get_Angle_Y() {
	if (angle_Y.hasChanged()) {
		angleY = angle_Y.getValue();
		servoY.write(map(angleY, 0, 1023, 0, 180));
	}
}

void radio_Cmd() {
	if (got_radio) {
		String radio_cmd = String((char*) rx_data);
		int Y_moving_val = radio_cmd.substring(0, radio_cmd.indexOf(';')).toInt();
		Y_moving_direction = Y_moving_val >= 0 ? -1 : 1;
		Y_moving_speed = abs(Y_moving_val);
		if (Y_moving_speed > 1) {
			Serial.println(
					String("Y dir:") + Y_moving_direction + " Y speed:" + Y_moving_speed);
			if (Y_moving_speed > 8)
				Y_moving_speed = 5;
			else if (Y_moving_speed > 4)
				Y_moving_speed = 10;
			else if (Y_moving_speed > 1)
				Y_moving_speed = 50;

			Y_moving.enabled = true;
			Y_moving.setInterval(Y_moving_speed);
		}
		else {
			Y_moving.enabled = false;
		}
		got_radio = false;
	}
}

void Y_Moving() {
	servoAngleY += Y_moving_direction;
	servoAngleY = checkServoAngle(servoAngleY);
	servoY.write(servoAngleY);
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

	get_angle_Y.onRun(get_Angle_Y);
	get_angle_Y.setInterval(10);

	serial_info.onRun(serial_Info);
	serial_info.setInterval(100);

	Y_moving.onRun(Y_Moving);
	Y_moving.enabled = false;

	radio_cmd.onRun(radio_Cmd);
	radio_cmd.setInterval(100);

	cmdParser.setOptKeyValue(true);

	angle_Y.setAverageAmount(30);
	accel_X.setAverageAmount(10);
	accel_Z.setAverageAmount(10);

	init_radio();
	attachPCINT(digitalPinToPCINT(8), radio_ISR, CHANGE);
}

void loop() {

	angle_Y.update();

	if (serial_cmd.shouldRun())
		serial_cmd.run();

	if (get_accel_data.shouldRun())
		get_accel_data.run();

	//if (get_angle_Y.shouldRun())
	//	get_angle_Y.run();

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
			if (abs(accelZ-aimAccelZ) > thrX) {
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


/*
 * Copyright (c) 2016 Subbaraya Sundeep <sundeep.babi@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This Arduino program receives commands from computer via serial port
 * and moves stepper motors and Pen motor.
 */

#define STEP_X     11
#define DIR_X      10

#define STEP_Y     9
#define DIR_Y      8

#define MAXSTEPS_X  155 //down
#define MAXSTEPS_Y  250 //UP

#define MOT_ENABLE1  5
#define MOT_ENABLE2  7

#define MOT_UP    4 // PEN UP
#define MOT_DOWN  3 // PEN DOWN

#define SCALE 3

String inString = "";

bool calibrate = false;
bool calibrate_x = true;
bool calibrate_y = true;

bool loc_x = false;
bool loc_y = false;
bool rev_x  = false;
bool rev_y = false;
bool pen = false;

int x = 0;
int y = 0;
int p = 0;

int curr_x = 0;
int curr_y = 0;

void setup() {
	Serial.begin(9600);

	pinMode(STEP_X, OUTPUT);
	pinMode(DIR_X, OUTPUT);
	pinMode(EN_X, OUTPUT);

	pinMode(STEP_Y, OUTPUT);
	pinMode(DIR_Y, OUTPUT);
	pinMode(EN_Y, OUTPUT);

	pinMode(MOT_UP, INPUT);
	pinMode(MOT_DOWN, INPUT);

	pinMode(MOT_ENABLE1, OUTPUT);
	pinMode(MOT_ENABLE2, OUTPUT);

	digitalWrite(STEP_X, LOW);
	digitalWrite(DIR_X, LOW);
	digitalWrite(EN_X, LOW);

	digitalWrite(STEP_Y, LOW);
	digitalWrite(DIR_Y, LOW);
	digitalWrite(EN_Y  , LOW);

	digitalWrite(MOT_ENABLE1, LOW);
	digitalWrite(MOT_ENABLE1, LOW);

	gohome();
}

void loop() 
{
	int i;
	char c;
  
	while (Serial.available() < 4);
  
	for (i = 0; i < 4; i++) {
		c = Serial.read();
    	if (c == 'C') {
        	calibrate = true;
        	loc_x = false;
        	loc_y = false;
        	pen  = false;
        	curr_x = 0;
        	curr_y = 0;
		} else if (c == 'H') {
			gohome();
			loc_x = false;
			loc_y = false;
			pen = false;
			curr_x = 0;
			curr_y = 0;
		} else if (c == 'X') {
			loc_x = true;
		} else if (c == 'Y') {
			loc_y = true;
		} else if (c == 'P') {
			pen = true;
		} else if (c == 'R') {
			digitalWrite(DIR_Y, LOW);
			generate_steps(STEP_Y, curr_y, 8, SCALE);
			curr_y = 0;
			digitalWrite(DIR_Y, HIGH);
			loc_y = false;
		} else {
			inString += (char)c;
		}
	}

	if (loc_x) {
		x = inString.toInt();
	if (x != curr_x)
		generate_steps(STEP_X, x - curr_x, 20, SCALE);
	curr_x = x;
	loc_x = false;
	}

	if (loc_y) {
		y = inString.toInt();
	if (y != curr_y)
		generate_steps(STEP_Y, y - curr_y, 20, SCALE);
	curr_y = y;
	loc_y = false;
	}

	if (pen) {
		pendown();
		pen = false;
	}

	Serial.print("Ende");

	if (calibrate) {
		if (inString.toInt() == 111) {
			calibrate_y = false;
		Serial.println("calibrate X");  
	} else if (inString.toInt() == 222) {
		calibrate_x = false; 
		Serial.println("calibrate Y");  
	} else {
		Serial.println("calibrate X and Y");  
	}

	while(1)
		gohome();  
  }

	inString = "";
}

void gohome()
{
	bool flag = true;
	int iterations = 6;
	int i;

	if (calibrate_x) {
		for (i = 0; i < iterations; i++)
		{
			digitalWrite(DIR_X, flag);
			generate_steps(STEP_X, MAXSTEPS_X, 3, 1);
			flag = !flag;
		}
	digitalWrite(DIR_X, HIGH);
	}

	if (calibrate_y) {
		flag = true;
		for (i = 0; i < iterations; i++)
		{
      		digitalWrite(DIR_Y, flag);
      		generate_steps(STEP_Y, MAXSTEPS_Y, 3, 1);
      		flag = !flag;
    	}
	digitalWrite(DIR_Y, HIGH);
	}
}

void generate_steps(int axis, int steps, int interval, int scale)
{
	int i;

	for (int i = 0; i < steps * scale; i++)
	{
		digitalWrite(axis, HIGH);
		delay(interval);
		digitalWrite(axis, LOW);
		delay(interval);
	}
}

void pendown()
{
	digitalWrite(MOT_ENABLE1, HIGH);
	digitalWrite(MOT_ENABLE2, LOW);
	while (digitalRead(MOT_DOWN));

	delay(200);

	digitalWrite(MOT_ENABLE1, LOW);
	digitalWrite(MOT_ENABLE2, HIGH);
	while (digitalRead(MOT_UP));

	digitalWrite(MOT_ENABLE1, LOW);
	digitalWrite(MOT_ENABLE2, LOW);
}

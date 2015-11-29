// 
// Copyright (c) 2014 - Allen Bauer - http://blog.thereadoracleatdelphi.com
// Under MIT License
// 

//#include <MAX31855.h>
#include <stdio.h>
#include <ClickButton.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

#define THERMISTOR_PIN A1

// EPCOS 100K Thermistor #3(B57560G104F)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4036 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 4036
// max adc: 1023
#define NUMTEMPS 20
// {ADC, temp }, // temp
short temptable[NUMTEMPS][2] /* PROGMEM */ = {
   {1, 864}, // 864.165363324 C
   {54, 258}, // 258.53991594 C
   {107, 211}, // 211.310066205 C
   {160, 185}, // 185.861725716 C
   {213, 168}, // 168.31793816 C
   {266, 154}, // 154.754297589 C
   {319, 143}, // 143.52544406 C
   {372, 133}, // 133.784751118 C
   {425, 125}, // 125.033500921 C
   {478, 116}, // 116.945124847 C
   {531, 109}, // 109.283980973 C
   {584, 101}, // 101.861768746 C
   {637, 94}, // 94.5095302806 C
   {690, 87}, // 87.0542728805 C
   {743, 79}, // 79.2915563492 C
   {796, 70}, // 70.9409729952 C
   {849, 61}, // 61.5523326183 C
   {902, 50}, // 50.25271896 C
   {955, 34}, // 34.7815846664 C
   {1008, 2} // 2.86606331838 C
};

typedef enum Mode {mdRun, mdTempSet, mdHeatOff};

int WindowSize = 2000;
int HeaterWindowSize = 5000;
int modeFlash = 0;
unsigned long motorWindowStart;
unsigned long heaterWindowStart;
unsigned long displayUpdateStart;

enum Mode mode = mdRun;

double SetTemp, InTemp, OutControl;

ClickButton downButton(PIN3, LOW, CLICKBTN_PULLUP);
ClickButton upButton(PIN6, LOW, CLICKBTN_PULLUP);
ClickButton modeButton(PIN2, LOW, CLICKBTN_PULLUP);

LiquidCrystal lcd(12, 13, 8, 9, 10, 11);
PID pid(&InTemp, &OutControl, &SetTemp, 100, 0.5, 75, DIRECT);
//MAX31855 Temp(5, 4, 7);

int read_temp(void)
{
   int rawtemp = analogRead(THERMISTOR_PIN);
   int current_celsius = 0;

   byte i;
   for (i=1; i<NUMTEMPS; i++)
   {
      if (temptable[i][0] > rawtemp)
      {
         int realtemp  = temptable[i-1][1] + (rawtemp - temptable[i-1][0]) * (temptable[i][1] - temptable[i-1][1]) / (temptable[i][0] - temptable[i-1][0]);
         if (realtemp > 255)
            realtemp = 255;

         current_celsius = realtemp;

         break;
      }
   }

   // Overflow: We just clamp to 0 degrees celsius
   if (i == NUMTEMPS)
   current_celsius = 0;

   return current_celsius;
}

void print_temp_table()
{
	Serial.println();
	Serial.println("temptable[NUMTEMPS][2] = {");
	for	(byte i = 0; i < NUMTEMPS; i++)
	{
		Serial.print("  {");
		Serial.print(temptable[i][0]);
		Serial.print(", ");
		Serial.print(temptable[i][1]);
		Serial.println("},");
	}
	Serial.println("};");
}

void setup()
{
  Serial.begin(9600);
  lcd.begin(20, 2);
  lcd.print("Tp: ");
  for (int i = 0; i < 5; i++)
  {
	byte pattern[8];
    for (int j = 0; j < 8; j++)
		pattern[j] = (0b11111 << (4 - i)) & 0x1f;
	lcd.createChar(i, pattern);
  }
  motorWindowStart = millis();
  heaterWindowStart = motorWindowStart;
  displayUpdateStart = motorWindowStart;
  pinMode(PIN5, OUTPUT);
  pinMode(PIN4, OUTPUT);
  SetTemp = 295;
  pid.SetOutputLimits(0, HeaterWindowSize);
  pid.SetMode(AUTOMATIC);
  //print_temp_table();
  //read_temp(true);
}

void displayProgress(int value)
{
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  for (int i = 0; i < value / 5; i++)
    lcd.write(4);
  lcd.write(value % 5);
}

void displayTemp(int temp, int ref, bool heater, bool motor)
{
  char buf[17];
  char refBuf[4];
  if (mode == mdTempSet)
  {
	  int dutyCycle = modeFlash % 4;
	  if (dutyCycle == 0 || dutyCycle == 1)
		  itoa(ref, refBuf, 10);
	  else
		  refBuf[0] = '\0';
	  modeFlash++;
  } else
    itoa(ref, refBuf, 10);
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  snprintf(buf, 16, "T: %3d,%3s %1s %1s", temp, refBuf, heater ? "H" : " ", motor ? "M" : " ");
  lcd.print(buf);
/*
  lcd.print("T: ");
  lcd.print(temp);
  lcd.print(",");
  if (mode == mdTempSet)
  {
	  int dutyCycle = modeFlash % 4;
	  if (dutyCycle == 0 || dutyCycle == 1)
		  lcd.print(ref);
	  else
		  lcd.print("   ");
	  modeFlash++;
  } else
    lcd.print(ref);
  lcd.print(" ");
  if (heater)
    lcd.print("ON");
  else
	lcd.print("  ");
  lcd.print(" ");
  if (motor)
	lcd.print("ON");
  else
	lcd.print("  ");
*/
}

void HandleMotor(void)
{
  unsigned long now = millis();
  int potValue = analogRead(A0);
  int scaled = map(potValue, 0, 1023, 0, WindowSize);
  if (now - motorWindowStart > WindowSize)
	  motorWindowStart += WindowSize;
  if (scaled > now - motorWindowStart)
	  digitalWrite(PIN5, HIGH);
  else
  	  digitalWrite(PIN5, LOW);
}

void HandleHeater()
{
  int temp = ((read_temp() * 9) / 5) + 32;
  InTemp = temp;
  pid.Compute();
  unsigned long now = millis();
  if(now - heaterWindowStart > HeaterWindowSize)
  { //time to shift the Relay Window
    heaterWindowStart += HeaterWindowSize;
  }
  if(OutControl > now - heaterWindowStart) 
	digitalWrite(PIN4,HIGH);
  else 
	digitalWrite(PIN4,LOW);
}

void UpdateDisplay()
{
  unsigned int now = millis();
  if (now - displayUpdateStart > 250)
  {
    int potValue = analogRead(A0);
    int progress = map(potValue, 0, 1023, 0, 5 * 16 - 1);
    displayTemp(InTemp, SetTemp, digitalRead(PIN4), digitalRead(PIN5));
    displayProgress(progress);
	displayUpdateStart = now;
  }
}

void HandleButtons()
{
	modeButton.Update();
	downButton.Update();
	upButton.Update();
	switch (mode) {
	case mdRun:
		if (modeButton.click > 0)
			mode = mdTempSet;
		break;
	case mdTempSet:
		if (upButton.click > 0)
			SetTemp += 1.0;
		else if (downButton.click > 0)
			SetTemp -= 1.0;
		else if (modeButton.click > 0)
			mode = mdRun;
		break;
	}
}

void loop()
{
  HandleMotor();
  HandleHeater();
  UpdateDisplay();
  HandleButtons();
}

#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);
#include "LowPower.h"
#include <DS3232RTC.h>
#include <Time.h>
#include <Wire.h>
#include <EEPROMex.h>

#define RADIUS 32.5 //cm
#define MAGNETS 2 //number of magnets
#define CIRCUM 2*RADIUS*3.1415926 //cm
#define MAGNETDISTANCE CIRCUM/MAGNETS //cm
#define RESETSPEED 2500 //ms

//digital pins
#define rpmPin 2 //interrupt 0
#define buttonPin1 3 //interrupt 1
#define buttonPin2 4
#define beepPin 5 //pwm
#define headLightPin 6 //pwm
#define enablePin 7
#define enablePinAux 8

//analog pins
#define lightSensor A1
#define temperatureSensor A2
#define rawBattery A3

#define dailyKmAddress 0
#define sumKmAddress 4

volatile unsigned long lastMagnet;
volatile unsigned long lastUpdate;
volatile byte stoppedState = 0;
volatile boolean needRefresh = true;

// travelled cm * 36 / elapsed ms = km/h

volatile float kmph = 0;
volatile float dailyKm = 9.444; //EEPROM.readFloat(dailyKmAddress); //this should be saved in EEPROM when stop
volatile float sumEEPROM = 12.333; //EEPROM.readFloat(sumKmAddress); //subt. with EEPROM read in KM
volatile unsigned long countedMagnets = 0; //this + EEPROM sumMeter should be saved in EEPROM sumMeter when stop


void setup() {
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(beepPin, OUTPUT);
  pinMode(headLightPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(enablePinAux, OUTPUT);
  pinMode(lightSensor, INPUT_PULLUP);
  pinMode(temperatureSensor, INPUT);
  pinMode(rawBattery, INPUT);
  digitalWrite(beepPin, LOW);
  digitalWrite(headLightPin, LOW);
  digitalWrite(enablePin, LOW);
  digitalWrite(enablePinAux, HIGH);
  u8g.begin(); 
  Serial.begin(9600);
  pinMode(rpmPin, INPUT_PULLUP);
  attachInterrupt(rpmPin-2, magnetDetected, FALLING); //2-2 = 0 means digital pin 2
}

void loop() {   
  if (needRefresh) {
    needRefresh = false;
    Serial.println(countedMagnets);
    UpdateDisplay();
  }
  //Serial.println(millis() - lastMagnet);
  if (millis() - lastMagnet > RESETSPEED) {
    kmph = 0;
    stoppedState = 0;
    UpdateDisplay();
    //time to save datas to EEPROM
    //EEPROM.updateFloat(dailyKmAddress, dailyKm);
    //EEPROM.updateFloat(sumKmAddress, sumEEPROM + (countedMagnets*MAGNETDISTANCE)/100000);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    //test is needed how the sleep ended: interrupt or simply the timer ran out
    if (millis() - lastMagnet > RESETSPEED) {//even if the millis is not upadted during the powerdown, if a magnet interrupt happened the conditon will fail. IF NOT: time to turn off display and clock
      digitalWrite(enablePin, HIGH);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      digitalWrite(enablePin, LOW); //poweroff is over due to rpm interrupt --> we are moving again
      u8g.begin();
      UpdateDisplay();
    }
  }
  while (!(millis() - lastMagnet > RESETSPEED) && !needRefresh) {
    LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);    
    }
}

void magnetDetected()
{//debounce here
  if (millis() - lastMagnet > 20) {
    dailyKm += MAGNETDISTANCE / 100000;
    countedMagnets++;
    if (stoppedState == 0) {
      stoppedState = 1; //stopped 1 means the first reading after a stop
    }
    else if (millis() - lastUpdate > 350 || stoppedState == 1) {
      kmph = MAGNETDISTANCE * 36 / (millis() - lastMagnet);
      lastUpdate = millis();
      stoppedState = 2; //state 2 means countinue cycling
      needRefresh = true;
    }  
    lastMagnet = millis();
  }
}

void UpdateDisplay() {
  // picture loop
  u8g.firstPage(); 
  do {
    drawScreen();
  }
  while(u8g.nextPage());
}

void drawScreen(void) {
  //speedo numbers if using u8g_font_osb35n then 1 number is 28 p wide
  //the dot is 14 p wide, so 1.2 = 70p 12.3 = 98p
  u8g.setFont(u8g_font_osb35n);
  if (kmph < 9.95)
    u8g.setPrintPos(28, 35);
  else
    u8g.setPrintPos(0, 35);
  u8g.print(kmph, 1);  
  
  // km/h label
  u8g.setFont(u8g_font_timB14r);
  u8g.setPrintPos(100, 15);
  u8g.print("km");
  u8g.drawHLine(100,17,25);
  u8g.setPrintPos(109, 33);
  u8g.print("h");
  
  //daily counters  
  u8g.setFont(u8g_font_helvB18r);
  // 13p per number, 6p per space
  char displayStr[5];
  dtostrf(dailyKm, 4, 1, displayStr);
  byte xPosOffset = 0;
  if (displayStr[0] == ' ')
    xPosOffset += 7;
  u8g.setPrintPos(xPosOffset, 63);
  u8g.print(displayStr[0]); u8g.print(displayStr[1]);
  u8g.setFont(u8g_font_helvB14r);  
  u8g.setPrintPos(26, 63);
  u8g.print("."); u8g.print(displayStr[3]);  
  
  //summary
  // 10p per number, 5p per space
  dtostrf(sumEEPROM + (countedMagnets*MAGNETDISTANCE)/100000, 4, 0, displayStr);
  u8g.setPrintPos(108-u8g.getStrWidth(displayStr), 63);
  u8g.print(displayStr);

  //km labels for daily and summary counters  
  u8g.setFont(u8g_font_timB10r);
  u8g.setPrintPos(40, 63);
  u8g.print("km");
  u8g.setPrintPos(108, 63);
  u8g.print("km");  
}

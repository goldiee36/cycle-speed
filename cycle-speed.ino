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
#define RESETSPEED 2500 //ms, reset speed to zero
#define DEBOUNCETIMEINTERVAL (MAGNETDISTANCE*36)/80

//digital pins
#define rpmPin 2 //interrupt 0
#define buttonPin1 3 //interrupt 1
#define buttonPin2 4
#define beepPin 5 //pwm
#define headLightPin 6 //pwm
#define enablePin 7 //to turn of OLED display and clock module
#define enablePinAux 8 //tempsensor, lightsensor

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

unsigned long lastBattMeas;

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
  lastBattMeas = millis();
}

void loop() {   
  if (needRefresh) {
    needRefresh = false;
    //Serial.println(countedMagnets); //for detecting rpm bounce
    UpdateDisplay();
  }
  if (millis() - lastMagnet > RESETSPEED) { //if no magenet detected for RESETSPEED milliseconds we assume bike is stopped.
    kmph = 0;
    stoppedState = 0;
    UpdateDisplay();    
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //complete controller power down for 8 seconds, but interruprt can always wake the controller up. Display remains active
    //test is needed how the sleep ended: interrupt or simply the timer ran out
    if (millis() - lastMagnet > RESETSPEED) {//even if the millis is not upadted during the powerdown, if a magnet interrupt happened the conditon will fail. IF NOT: time to turn off display and clock
      //time to save datas to EEPROM too
      //EEPROM.updateFloat(dailyKmAddress, dailyKm);
      //EEPROM.updateFloat(sumKmAddress, sumEEPROM + (countedMagnets*MAGNETDISTANCE)/100000);
      digitalWrite(enablePin, HIGH); //turn off display and clock module
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //power down controller "forever", only interruprt can wake it up
      //poweroff is over due to rpm interrupt --> we are moving again
      digitalWrite(enablePin, LOW); //turn on display and clock module
      u8g.begin();
      UpdateDisplay(); //***** not sure if needed, because speed is not determined yet
    }
  }
  while (!(millis() - lastMagnet > RESETSPEED) && !needRefresh) { //some ppower saving: if no screen refresh is needed and we do not reach the RESETSPEED time interval (so we are still moving) we can put the controller in idle.
    //Timer0 must left running because of the millis and because of this timer this idle won't take forever. Thats why the while is needed to put the controller back to idle.
    LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);    
    }
}

//called by the digital pin 2 interrupt on FALLING edge
void magnetDetected()
{
  if (millis() - lastMagnet > DEBOUNCETIMEINTERVAL) {//debounce here
    dailyKm += MAGNETDISTANCE / 100000;
    countedMagnets++;
    if (stoppedState == 0) {//stopped 0 means that last state was stopped
      stoppedState = 1; //stopped 1 means the we had the first reading after a stop. We need 2 readings after a stop to be able to calculate the speed.
    }
    else if (millis() - lastUpdate > 350 || stoppedState == 1) { //lastUpdate is used to not refresh the screen to fast in case of the readings coming in fast. If stopped state is 1 then we can refresh immediately though 
      kmph = MAGNETDISTANCE * 36 / (millis() - lastMagnet);
      lastUpdate = millis();
      stoppedState = 2; //state 2 means countinue cycling
      needRefresh = true; // this will be checked in the main loop to trigger the screen refresh
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
  //variables printed: kmph, dailyKm, sumEEPROM
  
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
  // 13p per number (12p + 1 spaceing), 6p per space
  char displayStr[5];
  dtostrf(dailyKm, 4, 1, displayStr);
  byte xPosOffset = 0;
  if (displayStr[0] == ' ')
    xPosOffset += 7;
  u8g.setPrintPos(xPosOffset, 63);
  u8g.print(displayStr[0]); u8g.print(displayStr[1]);
  u8g.setFont(u8g_font_helvB14r);  
  u8g.setPrintPos(26, 63);
  u8g.print("."); u8g.print(displayStr[3]);  //dot is 4p width (3p + 1p spaceing)
  
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

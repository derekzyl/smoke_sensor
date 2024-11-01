
#include <Arduino.h>
#include <SoftwareSerial.h>




#define ALARM_PIN 12
#define TXD 6
#define RXD 5
#define STOP_PIN 8
#define SENSOR_PIN A0
#define SAFETY_LEVEL 100

unsigned long buttonOnePressTime = 0;
bool buttonOneLongPressDetected = false;
const unsigned long longPressDuration = 2000; // 2 seconds for long press detection



SoftwareSerial mySerial(TXD, RXD);


void sendSms(String message) ;
void triggerAlarm();
void overrideAlarm();
void updateSerial();
bool longPress(int buttonPin );

void setup() {

  pinMode(ALARM_PIN, OUTPUT);
  pinMode(STOP_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);

  Serial.begin(9600);
  mySerial.begin(9600);

  // send  sms to indicate working
  sendSms("the system is working");

 
 
}

void loop() {
  int sensorValue = analogRead(SENSOR_PIN);
  Serial.println(sensorValue);
  updateSerial();{
if(longPress(STOP_PIN)){
  overrideAlarm();
}else
  if(sensorValue>SAFETY_LEVEL){
    triggerAlarm();}
else{
  digitalWrite(ALARM_PIN, LOW);
}}
}

void triggerAlarm(){
  digitalWrite(ALARM_PIN, HIGH);
  sendSms("there is gas leak detected");
}

void overrideAlarm(){
  digitalWrite(ALARM_PIN, LOW);
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

void sendSms(String message) {
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+2347059011222\"\r"); 
  delay(1000);
  mySerial.println(message);// The SMS text you want to send
  delay(100);
  mySerial.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}


bool longPress(int buttonPin) {
  if (digitalRead(buttonPin) == LOW) {
    if (buttonOnePressTime == 0) {
      buttonOnePressTime = millis();
    }
    if ((millis() - buttonOnePressTime) > longPressDuration) {
      buttonOnePressTime = 0;
      return true;
    }
  } else {
    buttonOnePressTime = 0;
  }
  return false;
}
#include <Arduino.h>
#include <SoftwareSerial.h>

#define ALARM_PIN 12
#define TXD 6
#define RXD 5
#define STOP_PIN 8
#define SENSOR_PIN 4
#define SAFETY_LEVEL 400

unsigned long buttonOnePressTime = 0;
bool buttonOneLongPressDetected = false;
const unsigned long longPressDuration = 2000; // 2 seconds for long press detection

// Add debounce variables for sensor readings
const int numReadings = 5;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

SoftwareSerial mySerial(TXD, RXD);

void sendSms(String message);
void triggerAlarm();
void overrideAlarm();
void updateSerial();
bool longPress(int buttonPin);

void setup() {
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(STOP_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(SENSOR_PIN, INPUT);

  // Initialize all readings to 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
  digitalWrite(ALARM_PIN, LOW);

  Serial.begin(9600);
  mySerial.begin(9600);

  // Send SMS to indicate working
  sendSms("The system is working");
  delay(60000); // allow the MQ2 to warm up
}

void loop() {
  // Read and average sensor values
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(SENSOR_PIN);
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;

  int digitalVal = digitalRead(SENSOR_PIN);

  // Debug print
  Serial.print("Raw sensor value: ");
  Serial.print(readings[readIndex]);
  Serial.print(" Average value: ");
  Serial.println(average);

    digitalWrite(ALARM_PIN, LOW);
  // Update serial communication
  updateSerial();

  // Check for long press first
  if (longPress(STOP_PIN)) {
    overrideAlarm();
  }
  // If no override, check gas levels
  // else if (average > SAFETY_LEVEL) {
  //   triggerAlarm();
  // }
  else if (digitalVal == LOW) {
    triggerAlarm();
  }
  else {
    digitalWrite(ALARM_PIN, LOW);
  }

  delay(500); // Small delay to prevent too frequent readings
}

void triggerAlarm() {
  static bool alarmSent = false;  // Keep track of whether we've sent an SMS
  
  digitalWrite(ALARM_PIN, HIGH);
  
  // Only send SMS once when leak is detected
  if (!alarmSent) {
    Serial.println("Gas leak detected!");
    sendSms("Gas leak detected!");
    alarmSent = true;
  }
}

void overrideAlarm() {
  static bool overrideSent = false;  // Keep track of override notification
  
  digitalWrite(ALARM_PIN, LOW);
  
  if (!overrideSent) {
    sendSms("Alarm has been manually overridden");
    overrideSent = true;
  }
}

void updateSerial() {
  Serial.println("Updating serial");
  Serial.println(mySerial.available());
  while (Serial.available()) {
    mySerial.write(Serial.read());
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}

void sendSms(String message) {
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);
  mySerial.println("AT+CMGS=\"+2347059011222\"\r"); 
  delay(1000);
  mySerial.println(message);

  delay(100);
  mySerial.println((char)26);  // ASCII code of CTRL+Z
  delay(1000);
}

bool longPress(int buttonPin) {
  if (digitalRead(buttonPin) == LOW) {  // Button is pressed
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
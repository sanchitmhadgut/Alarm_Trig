#include <Wire.h>
#include <ErriezDS1302.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#define DS1302_CLK_PIN      15
#define DS1302_IO_PIN       2
#define DS1302_CE_PIN       4

uint8_t btnSts = 0;
uint8_t btnPin = 19;

uint8_t r_led = 25;
uint8_t o_led = 26;
uint8_t g_led = 27;


// Create RTC object
ErriezDS1302 rtc = ErriezDS1302(DS1302_CLK_PIN, DS1302_IO_PIN, DS1302_CE_PIN);

Servo myservo;
int pos = 0;

BluetoothSerial SerialBT;
char receivedChar;

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 5;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int servoPin = 5;
#else
int servoPin = 5;
#endif

// Variables to store alarm time
int alarmHour = -1;
int alarmMinute = -1;
bool alarmSet = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32_ALarm");
  Serial.println("Your device is connected");

   while (!Serial) {
        ;
    }
    Serial.println(F("\nErriez DS1302 set get time example"));

  pinMode(btnPin, INPUT_PULLUP);
  pinMode(r_led, OUTPUT);
  pinMode(o_led, OUTPUT);
  pinMode(g_led, OUTPUT);

 // Initialize I2C
    Wire.begin();
    Wire.setClock(100000);

     //servo
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 5 to the servo object

    // Initialize RTC
    while (!rtc.begin()) {
        Serial.println(F("RTC not found"));
        delay(3000);
    }

    // // Set new time 12:00:00
    // if (!rtc.setTime(2, 49, 30)) {
    //     Serial.println(F("Set time failed"));
    // }

    while(digitalRead(btnPin) == 1 ){

    }
    analogWrite(r_led, 50); // Code started

    SerialBT.println("Enter alarm time in HH:MM format:");

}

void loop() {

  // put your main code here, to run repeatedly:
  // btnSts = digitalRead(btnPin);
  // Wait for alarm time input from Serial Monitor
  if (SerialBT.available() > 0 && !alarmSet) {
    String input = SerialBT.readStringUntil('\n');
    if (parseAlarmTime(input)) {
      SerialBT.print("Alarm set for ");
      SerialBT.print(alarmHour);
      SerialBT.print(":");
      if (alarmMinute < 10) {
        SerialBT.print("0");
      }
      SerialBT.println(alarmMinute);
      alarmSet = true;
    } else {
      SerialBT.println("Invalid time. Please enter time in HH:MM format.");
    }
  }

    if (alarmSet) {
    // Now that the alarm is set, start showing the real-time clock and check alarm
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    if (rtc.getTime(&hour, &minute, &second)) {
      // Print current time
      SerialBT.print(hour);
      SerialBT.print(":");
      if (minute < 10) {
        SerialBT.print("0");
      }
      SerialBT.print(minute);
      SerialBT.print(":");
      if (second < 10) {
        SerialBT.print("0");
      }
      SerialBT.println(second);

      // If the current time is 1 minute before the alarm, blink orange LED
      if (alarmSet && hour == alarmHour && minute == alarmMinute - 1) {
        analogWrite(o_led, 100);  // Blink orange LED
      }

      // If it's the alarm time, turn on the green LED
      if (alarmSet && hour == alarmHour && minute == alarmMinute) {
        analogWrite(g_led, 100);  // Turn on green LED
        servo();
        delay(10000);  // Keep the green LED on for 10 seconds

        analogWrite(g_led, 0);
        analogWrite(o_led, 0);
        alarmSet = false;  // Reset alarm
        SerialBT.println("Alarm triggered!");
      }
    } else {
      SerialBT.println(F("Failed to get time from RTC"));
    }

    delay(1000);  // Wait for 1 second before checking again
  }

}


// Helper function to parse alarm time input
bool parseAlarmTime(String input) {
  int separatorIndex = input.indexOf(':');
  if (separatorIndex == -1) {
    return false;
  }
  String hourStr = input.substring(0, separatorIndex);
  String minuteStr = input.substring(separatorIndex + 1);

  int tempHour = hourStr.toInt();
  int tempMinute = minuteStr.toInt();

  if (tempHour >= 0 && tempHour < 24 && tempMinute >= 0 && tempMinute < 60) {
    alarmHour = tempHour;
    alarmMinute = tempMinute;
    return true;
  }
  return false;
}

void servo()
{
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);    // tell servo to go to position in variable 'pos'
    delay(100);             // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);    // tell servo to go to position in variable 'pos'
    delay(100);             // waits 15ms for the servo to reach the position
  }
}
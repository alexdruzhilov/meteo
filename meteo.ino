#include <Arduino.h>
#include <TM1637Display.h>
#include "MHZ19.h"
#include <SoftwareSerial.h> 
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Display connection pins (Digital Pins)
#define DISPLAY_CLK SCL
#define DISPLAY_DIO SDA
TM1637Display display(DISPLAY_CLK, DISPLAY_DIO);

// MHZ19 sensor
#define MHZ19_RX_PIN 8                                     // Rx pin which the MHZ19 Tx pin is attached to
#define MHZ19_TX_PIN 7                                     // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(MHZ19_RX_PIN, MHZ19_TX_PIN);       // (Uno example) create device to MH-Z19 serial
unsigned long mhz19DataTimer = 0;

// DHT sensor
#define DHTPIN   A3
#define DHTTYPE  DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t DHTdelayMS;

// Change mode button
const int ModeButtonPin = 2; // Mode button pin
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// LED
const int RED_LED_PIN = 12;
const int BLUE_LED_PIN = 13;
const int RGB_LED_PIN_R = 9;
const int RGB_LED_PIN_G = 11;
const int RGB_LED_PIN_B = 10;

// Global variables
int mode = 0;
int co2 = 0;
double temperature = 0;
double humidity = 0;
unsigned long dataCollectionTimer = 0;

void setup() {
  Serial.begin(9600);                                     // Device to serial monitor feedback

  // MZH19 setup
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 
  myMHZ19.autoCalibration(false);

  // DHT setup
  dht.begin();

  // Change mode button setup
  pinMode(ModeButtonPin, INPUT);

  // LED setup
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RGB_LED_PIN_R, OUTPUT);
  pinMode(RGB_LED_PIN_G, OUTPUT);
  pinMode(RGB_LED_PIN_B, OUTPUT);

  // Display setup
  display.setBrightness(0x0f);  
}

void loop()
{
  readChangeModeButton();
  
  if (millis() - dataCollectionTimer >= 2000) {
    readMHZ19();
    readDHT();
    displayData(false, mode);
    showAlerts();
    dataCollectionTimer = millis();
  }
}

void showAlerts() {
  if (temperature < 18 or temperature > 26) {
    digitalWrite(RED_LED_PIN, HIGH);
  } else {
    digitalWrite(RED_LED_PIN, LOW);
  }

  if (humidity < 30 or humidity > 60) {
    digitalWrite(BLUE_LED_PIN, HIGH);
  } else {
    digitalWrite(BLUE_LED_PIN, LOW);
  }

  if (co2 < 800) {
    RGB_color(0, 0, 0);     // No light
  } else if (co2 >= 800 and co2 < 1000) {
    RGB_color(120, 120, 0); // Yellow
  } else if (co2 >= 1000 and co2 <= 1400) {
    RGB_color(255, 30, 40);  // Orange
  } else if (co2 > 1400) {
    RGB_color(255, 0, 0);   // Red
  }
}

void displayData(bool cleanup, int mode) {
  if (cleanup) {
    display.clear();
  }

  if (mode == 1) {
    displayTemperature(temperature);
  } else if (mode == 2) {
    displayHumidity(humidity);
  } else if (mode == 3) {
    displayCO2(co2);
  }
}

void displayCO2(int value) {
  display.showNumberDec(value, false);
}

void displayTemperature(double value) {
  uint8_t data[] = { 0x00, 0x00, 0x00, SEG_A | SEG_B | SEG_F | SEG_G };
  display.setSegments(data); 
  display.showNumberDec(round(value), false, 2, 1);
}

void displayHumidity(double value) {
  display.showNumberDec(round(value), false);
}

void readChangeModeButton() {
  // read the pushbutton input pin:
  buttonState = digitalRead(ModeButtonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      mode++;
      if (mode > 3) {
        mode = 1;
      }
      Serial.print("change mode to: ");
      Serial.println(mode);
    }
    // Delay a little bit to avoid bouncing
    delay(50);

    lastButtonState = buttonState;

    displayData(true, mode);
  }
}

void readDHT() {
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("DHT Error reading temperature!"));
  }
  else {
    Serial.print(F("DHT Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));

    temperature = event.temperature;
  }
  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("DHT Error reading humidity!"));
  }
  else {
    Serial.print(F("DHT Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));

    humidity = event.relative_humidity;
  }
}

void readMHZ19() {
  co2 = myMHZ19.getCO2();                              // Request CO2 (as ppm)
  Serial.print("MZH19 CO2 (ppm): ");                      
  Serial.println(co2);                          
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(RGB_LED_PIN_R, red_light_value);
  analogWrite(RGB_LED_PIN_G, green_light_value);
  analogWrite(RGB_LED_PIN_B, blue_light_value);
}

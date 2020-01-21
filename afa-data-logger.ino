#include <SPI.h>

#include "ubx.h"

// ZOE-M8Q Specification
//   Max transfer rate: 125 kb/s
//   Max frequency: 5.5 MHz

SPISettings gpsSPISetting(1000000, MSBFIRST, SPI_MODE0);
//SPISettings gpsSPISetting(5500000, MSBFIRST, SPI_MODE0);

// SPI comm pins
const int PIN_MOSI = 11;
const int PIN_MISO = 12;
const int PIN_SCK = 13;
const int PIN_SSN = 10;

// functions

void setup() {
  // Serial comm
  Serial.begin(9600);

  // GPS module SPI comm
  SPI.begin();

  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_SSN, OUTPUT);

  digitalWrite(PIN_SSN, HIGH); // not select slave as default
  digitalWrite(PIN_MOSI, HIGH); // MOSI should be kept HIGH unless there are data to be sent to receiver
  
  SPI.beginTransaction(gpsSPISetting);

  //
  delay(100);
}

unsigned long lastMillis = millis();



void loop() {
  digitalWrite(PIN_SSN, LOW);
  {
    delayMicroseconds(10); // min init time = 10us

    unsigned char c = SPI.transfer(0xff);
    Serial.print((char) c);
  }
  digitalWrite(PIN_SSN, HIGH);
  delay(1); // deselect time = 1ms
}

void _loop() {
  digitalWrite(PIN_SSN, LOW);
  {
    delayMicroseconds(10); // min init time = 10us

    unsigned char buf[16];
    for (int i = 0; i < 16; i++) {
      buf[i] = SPI.transfer(0xff);
    }
    Serial.print("\"");
    for (int i = 0; i < 16; i++) {
      if (buf[i] == '\n') Serial.print("\\n");
      else Serial.print((char) buf[i]);
    }
    Serial.print("\", [");
    for (int i = 0; i < 16; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.print("]");
    Serial.println();
    
    if (millis() - lastMillis > 1000) {
      lastMillis = millis();

      unsigned char buf[1024];
      ubx_nav_posllh msg;
      int len = ubx_make_packet(buf, msg);
      Serial.println();
      Serial.println("#################################");
      for (int i = 0; i < len; i++) {
        Serial.print("0x");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      Serial.println("#################################");

      for (int i = 0; i < len; i++) {
        unsigned char c = SPI.transfer(buf[i]);
        Serial.print((char) c);
      }
      Serial.println();
    }
  }
  digitalWrite(PIN_SSN, HIGH);
  delay(1); // deselect time = 1ms

  delay(1);
}

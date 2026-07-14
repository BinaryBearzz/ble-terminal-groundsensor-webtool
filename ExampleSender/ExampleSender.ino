#include <Arduino.h>
#include "uart_builder.h"
#include "ground_sensor_at.h"

// Example sender that talks to GroundSensor over UART using AT commands.
// This file auto-selects an appropriate UART implementation for AVR, ESP32, ESP8266.
// All AT-command/response handling now lives in ground_sensor_at.h (GroundSensor class).

// GroundSensor expects 115200 8N1
#define GS_BAUD 115200

// Default pins (override in build flags or before including this file)
#ifndef GS_UART_RX_PIN
#define GS_UART_RX_PIN 26  //<--> 46 TX Heltec
#endif
#ifndef GS_UART_TX_PIN
#define GS_UART_TX_PIN 27  //<--> 45 RX Heltec
#endif

#define UNIXTIME_REFERENCE 1783479326
// Enable interactive diagnostic echo between USB Serial and the sensor UART
// set to 1 to type AT commands in the Serial Monitor and see replies.
static int counter = 0;
// Optionally force SoftwareSerial by defining GS_USE_SOFTWARE_SERIAL
#if defined(GS_USE_SOFTWARE_SERIAL) || defined(ESP8266) || defined(ARDUINO_ARCH_AVR)
#include <SoftwareSerial.h>
#endif

#if defined(GS_USE_SOFTWARE_SERIAL) || (defined(ESP8266)) || (defined(ARDUINO_ARCH_AVR) && !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__))
static SoftwareSerial gsSoftSerial(GS_UART_RX_PIN, GS_UART_TX_PIN);
#endif

static GroundSensor groundSensor;
// Pointer to the low-level stream used for diagnostics (Serial1 or SoftwareSerial)
static Stream *debugStream = nullptr;

void setup() {
  // Monitor
  Serial.begin(115200);
  delay(100);

  // Initialize the appropriate UART for the target MCU and bind it to groundSensor
#if defined(ESP32)
  Serial1.begin(GS_BAUD, SERIAL_8N1, GS_UART_RX_PIN, GS_UART_TX_PIN);
  groundSensor.begin(&Serial1);
#elif defined(ESP8266)
  // ESP8266: Serial1 is often TX-only. Prefer SoftwareSerial when GS_USE_SOFTWARE_SERIAL set.
#if defined(GS_USE_SOFTWARE_SERIAL)
  gsSoftSerial.begin(GS_BAUD);
  groundSensor.begin(&gsSoftSerial);
#else
  Serial1.begin();
  groundSensor.begin(&Serial1);  // may be TX-only on some cores
#warning "ESP8266: Serial1 may be TX-only. Define GS_USE_SOFTWARE_SERIAL to use SoftwareSerial for RX+TX."
#endif
#elif defined(ARDUINO_ARCH_AVR)
  // AVR: Mega/2560 has Serial1; smaller Arduinos need SoftwareSerial
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  Serial1.begin(GS_BAUD);
  groundSensor.begin(&Serial1);
#else
  gsSoftSerial.begin(GS_BAUD);
  groundSensor.begin(&gsSoftSerial);
#endif
#else
  // Fallback: try Serial1
  Serial1.begin(GS_BAUD);
  groundSensor.begin(&Serial1);
#endif

  // Select the low-level stream for diagnostics to match the stream
  // we bound into `groundSensor` above.
#if defined(ESP32)
  debugStream = &Serial1;
#elif defined(ESP8266)
#if defined(GS_USE_SOFTWARE_SERIAL)
  debugStream = &gsSoftSerial;
#else
  debugStream = &Serial1;
#endif
#elif defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  debugStream = &Serial1;
#else
  debugStream = &gsSoftSerial;
#endif
#else
  debugStream = &Serial1;
#endif

  delay(300);
  Serial.println("ExampleSender started");

  // Do not send any real commands until the GroundSensor confirms it is
  // alive on the UART with a plain "AT" -> "OK" handshake.
  if (!groundSensor.waitForReady()) {
    // Nothing more we can safely do; loop() will keep retrying the ping.
    return;
  }

  // Build payload: "OK:12.34" as ASCII bytes
  UartBuilder b;
  b.appendStringAsAscii("OK");
  b.appendByte(':');
  b.appendFloatAsBytes(12.34f);  // float 12.34 as raw IEEE-754 bytes (4 bytes)
  // b.appendFloatAsAscii(12.34); // float 12.34 as ASCII characters '1','2','.','3','4' (5bytes)

  String payload = b.toPayload();  // OK:12.34 -> String OK:(3bytes)   Float 12.34(4bytes)

  // Send once and react to the result
  GsResponse sendResp = groundSensor.sendAT_SEND(-1, 123, payload);
  if (sendResp.status == GsStatus::ERROR) {
    Serial.println("AT+SEND failed: " + sendResp.lastLine);
  }

  GsResponse syncResp = groundSensor.sendAT_SYNCTIME(UNIXTIME_REFERENCE);  // Example: send a specific Unix timestamp to synchronize time
  if (syncResp.status != GsStatus::OK) {
    Serial.println("AT+SYNCTIME did not confirm OK (status=" + String((int)syncResp.status) + ")");
  } else {
    Serial.println("AT+SYNCTIME successful: " + syncResp.lastLine);
  }
}

void loop() {
  // If we never got the initial handshake, keep retrying periodically
  // instead of spamming AT+SEND commands the sensor can't process.
  if (!groundSensor.isConnected()) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry < 5000)
      return;
    lastRetry = millis();
    groundSensor.waitForReady();
    return;
  }

  // Periodic example: send an integer as two raw bytes followed by text
  static unsigned long last = 0;
  if (millis() - last < 10000)
    return;
  last = millis();

  UartBuilder b;
  int number = 1000;
  String text = "_OK_";
  b.appendUint16(number);       // 2-byte big-endian representation of 1000
  b.appendStringAsAscii(text);  // Append string "_OK_" as ASCII bytes 4 characters (4 bytes)
  b.appendUint16(counter);      // 2-byte big-endian representation of counter
  // b.appendUint32(counter); // 4-byte big-endian representation of counter

  String payload = b.toPayload();  // 1000_OK_(0-65535)
  Serial.println("=========================");
  Serial.print("Message from: ");
  Serial.print(number);
  Serial.print(text);
  Serial.println(counter);
  Serial.print("Payload(byte decimal): ");
  Serial.println(payload);

  counter++;

  GsResponse resp = groundSensor.sendAT_SEND(-1, 123, payload);  // ch=-1 -> GroundSensor may randomize channel per spec
  switch (resp.status) {
    case GsStatus::OK:
      // e.g. "OK,SEND:5,CH:3,FINISH" - packet accepted and transmitted
      break;
    case GsStatus::ERROR:
      // e.g. "ERROR,DATA:>21" - payload too large, or ERROR,P:0 project not set, etc.
      Serial.println("GroundSensor rejected command: " + resp.lastLine);
      break;
    case GsStatus::TIMEOUT:
      Serial.println("No final response before timeout, partial: " + resp.raw);
      break;
    case GsStatus::NONE:
      Serial.println("GroundSensor did not respond (check wiring/baud).");
      break;
  }
}
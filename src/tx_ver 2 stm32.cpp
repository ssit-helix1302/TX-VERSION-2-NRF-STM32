#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- Analog Joystick Pins ---
#define vrx1 PA3       // Yaw
#define vry1 PA2       // Throttle
#define vrx2 PA1       // Roll
#define vry2 PA0       // Pitch

// --- Switches and LEDs ---
#define sw1 PB10       // AUX1 ARM SWITCH
#define sw2 PB5        // AUX2 CONFIG MODE
#define nrfled PC13    // Built-in LED on STM32 (Active-LOW)
#define armled PB11    // Arm LED

// --- Radio & Timing Variables ---
unsigned long lastSuccessTime = 0;
bool failsafeActive = false;
bool armstate = false;

unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 10; // 10ms = 100 Hz

#define CE_PIN PB0
#define CSN_PIN PB1

RF24 radio(CE_PIN, CSN_PIN);
const byte location[5] = "F450"; 

// --- Data Structures ---
typedef struct __attribute__((packed)) drone {
  int32_t  roll, pitch, yaw, throttle, aux2;
  uint8_t arm, disarm;
} packet;

typedef struct __attribute__((packed)) telemetry {
  int32_t rawDroneVoltage; 
} telemPacket;

packet dataToRX;
telemPacket dataFromRX;

// --- Helper Functions ---
int simpleChannel(int raw) {
  return map(raw, 0, 4095, 1000, 2000); 
}

void setup() {
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  
  pinMode(armled, OUTPUT); 
  pinMode(nrfled, OUTPUT);

  analogReadResolution(12);

  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(location);
}

void loop() {
  unsigned long currentMillis = millis();

  // Run control logic at 100 Hz to match V1
  if (currentMillis - lastLoopTime >= loopInterval) {
    lastLoopTime = currentMillis;

    // 1. Human Input (Read & Map) - Matched to V1 behavior
    dataToRX.throttle = simpleChannel(analogRead(vry1));
    dataToRX.roll     = simpleChannel(analogRead(vrx2));
    dataToRX.pitch    = 3000 - simpleChannel(analogRead(vry2));
    dataToRX.yaw      = simpleChannel(analogRead(vrx1));

    // 2. ARM Switch Logic (SW 1)
    armstate = (digitalRead(sw1) == LOW);
    digitalWrite(armled, armstate ? HIGH : LOW);
    dataToRX.arm = armstate;
    dataToRX.disarm = !armstate;

    // 3. AUX Switch Logic (SW 2)
    if (digitalRead(sw2) == LOW) {
      dataToRX.aux2 = 2000;
    } else {
      dataToRX.aux2 = 1000;
    }

    // 4. Radio Exchange
    radio.stopListening();
    if (radio.write(&dataToRX, sizeof(dataToRX))) {
      lastSuccessTime = millis();
      failsafeActive = false;
      
      // Formal Telemetry Check: Read to clear the buffer, but do nothing with it.
      if (radio.isAckPayloadAvailable()) {
        radio.read(&dataFromRX, sizeof(dataFromRX));
      }
    }

    // 5. Failsafe & Status Checking
    if (millis() - lastSuccessTime > 500) {
      failsafeActive = true;
    }
    
    // STM32 LED indicator (Active-LOW on most STM32 boards)
    digitalWrite(nrfled, !radio.isChipConnected()); 
  }
}
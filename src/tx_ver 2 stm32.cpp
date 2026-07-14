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
#define NRF_LED PC13    // Built-in LED on STM32 (Active-LOW)
#define ARM_LED PB11    // ARM LED

#define LOWBATT_LED PB9
#define BUZZER_PIN PB8

// --- Radio & Timing Variables ---
unsigned long lastSuccessTime = 0;
bool failsafeActive = false;
bool armstate = false;

unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 10; // 10ms = 100 Hz
//const unsigned long lastwritethreshold = 2000; // 2 sec
//unsigned long lastwritetime = 0 ; 


#define CE_PIN PB0
#define CSN_PIN PB1

RF24 radio(CE_PIN, CSN_PIN);
const byte location[10] = "MiniDrone"; 

// Telemetry & Voltage Variables ---
float droneVoltage = 0.0;
float VOLTAGE_CALIBRATION = 0.995;
const float VOLTAGE_MULTIPLIER = (3.3 / 4095.0) * 2.0; 

int ledSagCounter = 0;
int buzzerSagCounter = 0;
bool lowBattLedActive = false;
bool criticalBattBuzzerActive = false;
const int SAG_THRESHOLD = 100;

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
  //Serial.begin(115200);

  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  
  pinMode(ARM_LED, OUTPUT); 
  pinMode(NRF_LED, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LOWBATT_LED, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LOWBATT_LED, LOW);

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
    digitalWrite(ARM_LED, armstate ? HIGH : LOW);
    dataToRX.arm = armstate;
    dataToRX.disarm = !armstate;

    // 3. AUX Switch Logic (SW 2)
    if (digitalRead(sw2) == LOW) {
      dataToRX.aux2 = 2000;
    } else {
      dataToRX.aux2 = 1000;
    }
     
    // If the battery has sagged below 3.5V for 200 ticks, force stop.
    if (criticalBattBuzzerActive) {
      dataToRX.throttle = 1000; 
      dataToRX.arm = 0;         
      dataToRX.disarm = 1;
    }

    // 4. Radio Exchange
    radio.stopListening();
    if (radio.write(&dataToRX, sizeof(dataToRX))) {
      lastSuccessTime = millis();
      failsafeActive = false;
      
     // 5.
      if (radio.isAckPayloadAvailable()) {
        radio.read(&dataFromRX, sizeof(dataFromRX));

        droneVoltage = dataFromRX.rawDroneVoltage * VOLTAGE_MULTIPLIER*VOLTAGE_CALIBRATION;
        
        if (droneVoltage > 3.30) { 
          
          if (droneVoltage < 3.70) {
            ledSagCounter++;
            if (ledSagCounter >= SAG_THRESHOLD) {
              lowBattLedActive = true;
            }
          } else {
            ledSagCounter = 0;
          }

          if (droneVoltage < 3.60) {
            buzzerSagCounter++;
            if (buzzerSagCounter >= SAG_THRESHOLD) {
              criticalBattBuzzerActive = true;
            }
          } else {
            buzzerSagCounter = 0;
          }
        } else {
          lowBattLedActive = true;
        }
      }
    }
    
    /*if (currentMillis - lastwritetime >= lastwritethreshold) {
      Serial.print("Drone Battery Voltage : ");
      Serial.print(droneVoltage);
      Serial.println("");
      lastwritetime = currentMillis;
    }*/


    // 6. Failsafe & Status Checking
    if (millis() - lastSuccessTime > 500) {
      failsafeActive = true;
      droneVoltage = 0.0;
      lowBattLedActive = false;
      criticalBattBuzzerActive = false;
      ledSagCounter = 0;
      buzzerSagCounter = 0;
    }
    
    // STM32 LED indicator (Active-LOW on most STM32 boards)
    digitalWrite(NRF_LED, !radio.isChipConnected());

    digitalWrite(LOWBATT_LED, lowBattLedActive ? HIGH : LOW); 

    if (criticalBattBuzzerActive && armstate && !failsafeActive) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  } 
} 
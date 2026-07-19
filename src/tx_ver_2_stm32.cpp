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
int recoveryCounter = 0; // NEW: Prevents LED flickering on boundary limits
bool lowBattLedActive = false;
bool criticalBattBuzzerActive = false;
bool autoDisarmed = false; // NEW: Prevents violent re-arming mid-air
const int SAG_THRESHOLD = 100;
const int RECOVERY_THRESHOLD = 50; // NEW: Require stable voltage to turn off LED

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
  
  pinMode(ARM_LED, OUTPUT); 
  pinMode(NRF_LED, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LOWBATT_LED, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LOWBATT_LED, LOW);

  analogReadResolution(12);

  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(115);
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

    // 1. Human Input (Read & Map)
    dataToRX.throttle = simpleChannel(analogRead(vry1));
    dataToRX.roll     = simpleChannel(analogRead(vrx2));
    dataToRX.pitch    = 3000 - simpleChannel(analogRead(vry2));
    dataToRX.yaw      = simpleChannel(analogRead(vrx1));

    // 2. ARM Switch Logic (SW 1)
    armstate = (digitalRead(sw1) == LOW);
    
    // NEW: If user physically flips switch to disarm, clear the safety latch
    if (!armstate) {
      autoDisarmed = false; 
    }

    // 3. AUX Switch Logic (SW 2)
    if (digitalRead(sw2) == LOW) {
      dataToRX.aux2 = 2000;
    } else {
      dataToRX.aux2 = 1000;
    }
     
    // NEW: Latching Force Stop Logic 
    // If battery critical, force stop. Latch it until pilot manually disarms.
    if (criticalBattBuzzerActive || autoDisarmed) {
      autoDisarmed = true; 
      dataToRX.throttle = 1000; 
      dataToRX.arm = 0;         
      dataToRX.disarm = 1;
      digitalWrite(ARM_LED, LOW); // Turn off ARM LED to indicate forced disarm
    } else {
      dataToRX.arm = armstate;
      dataToRX.disarm = !armstate;
      digitalWrite(ARM_LED, armstate ? HIGH : LOW);
    }

    // 4. Radio Exchange
    radio.stopListening();
    if (radio.write(&dataToRX, sizeof(dataToRX))) {
      lastSuccessTime = millis();
      failsafeActive = false;
      
     // 5. Telemetry Handling
      if (radio.isAckPayloadAvailable()) {
        radio.read(&dataFromRX, sizeof(dataFromRX));

        droneVoltage = dataFromRX.rawDroneVoltage * VOLTAGE_MULTIPLIER * VOLTAGE_CALIBRATION;
        
        if (droneVoltage > 3.30) { 
          
          if (droneVoltage < 3.70) {
            ledSagCounter++;
            recoveryCounter = 0; 
            if (ledSagCounter >= SAG_THRESHOLD) {
              lowBattLedActive = true;
            }
          } else {
            ledSagCounter = 0;
            // NEW: Only clear LED after voltage is stably recovered
            recoveryCounter++;
            if (recoveryCounter >= RECOVERY_THRESHOLD) {
                lowBattLedActive = false;
            }
          }

          if (droneVoltage < 3.60) {
            buzzerSagCounter++;
            if (buzzerSagCounter >= SAG_THRESHOLD) {
              criticalBattBuzzerActive = true;
            }
          } else {
            buzzerSagCounter = 0;
            // NEW: Clears buzzer if voltage climbs up (but safety latch keeps drone disarmed)
            if (recoveryCounter >= RECOVERY_THRESHOLD) {
                criticalBattBuzzerActive = false; 
            }
          }
        } else {
          lowBattLedActive = true; // Instantly on if extremely low
        }
      }
    }

    // 6. Failsafe & Status Checking
    if (millis() - lastSuccessTime > 500) {
      failsafeActive = true;
      droneVoltage = 0.0;
      lowBattLedActive = false;
      criticalBattBuzzerActive = false;
      ledSagCounter = 0;
      buzzerSagCounter = 0;
      recoveryCounter = 0;
    }
    
    digitalWrite(NRF_LED, !radio.isChipConnected());
    digitalWrite(LOWBATT_LED, lowBattLedActive ? HIGH : LOW); 

    // NEW: Pulsing buzzer instead of solid tone
    if (criticalBattBuzzerActive && armstate && !failsafeActive) {
      bool buzzerPulse = (millis() % 500) < 250; // Beep twice a second
      digitalWrite(BUZZER_PIN, buzzerPulse ? HIGH : LOW);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  } 
}
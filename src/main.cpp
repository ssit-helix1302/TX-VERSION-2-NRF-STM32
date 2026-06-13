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
#define sw1 PB10       // AUX1  ARM SWITCH
#define sw2 PB5        // AUX 2 CONFIG MODE
#define sw3 PB4        // AUX 3 ACRO/ANGLE
#define nrfled PC13    // Built-in LED on STM32 (Active-LOW)
#define armled PB11    // Arm LED

#define CE_PIN PB0
#define CSN_PIN PB1

RF24 radio(CE_PIN, CSN_PIN);
const byte location[6] = "HeliX"; 

typedef struct __attribute__((packed)) drone {
  int32_t  roll, pitch, yaw, throttle, aux2, aux3;
  uint8_t arm, disarm;
} packet;

packet dataToRX;

bool sw3ToggleState = false;        
int sw3ButtonState = HIGH;          
int lastSw3Reading = HIGH;         
unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 50; 

int blinkCount = 0; 
unsigned long lastBlinkTime = 0;
unsigned long blinkInterval = 100; 

// Non-blocking timer variables
unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL = 10; // 100Hz transmission rate

// Filter states for stick smoothing (Exponential Moving Average)
float emaRoll = 1500, emaPitch = 1500, emaYaw = 1500, emaThrottle = 1000;
const float EMA_ALPHA = 0.4f; // Lower = smoother but slower response. Range: 0.0 to 1.0

// Apply deadband to ignore minor stick jitters at the center
int applyDeadband(int rawValue, int centerValue, int deadbandAmount) {
  if (abs(rawValue - centerValue) < deadbandAmount) {
    return centerValue;
  }
  return rawValue;
}

int simpleChannel(int raw) {
  return map(raw, 0, 4095, 1000, 2000); 
}

void setup() {
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP); 
  
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

  //Replaced delay(12) with a non-blocking millis() timer
  if (millis() - lastTxTime >= TX_INTERVAL) {
    lastTxTime = millis();

    // Read, filter, and map raw analog inputs smoothly
    int rawThrottle = simpleChannel(analogRead(vry1));
    int rawRoll     = simpleChannel(analogRead(vrx2));
    int rawPitch    = 3000 - simpleChannel(analogRead(vry2));
    int rawYaw      = simpleChannel(analogRead(vrx1));

    emaThrottle = (EMA_ALPHA * rawThrottle) + ((1.0f - EMA_ALPHA) * emaThrottle);
    emaRoll     = (EMA_ALPHA * rawRoll)     + ((1.0f - EMA_ALPHA) * emaRoll);
    emaPitch    = (EMA_ALPHA * rawPitch)    + ((1.0f - EMA_ALPHA) * emaPitch);
    emaYaw      = (EMA_ALPHA * rawYaw)      + ((1.0f - EMA_ALPHA) * emaYaw);

    dataToRX.throttle = (int)emaThrottle;
    dataToRX.roll     = applyDeadband((int)emaRoll, 1500, 15);
    dataToRX.pitch    = applyDeadband((int)emaPitch, 1500, 15);
    dataToRX.yaw      = applyDeadband((int)emaYaw, 1500, 15);

    // Read switches
    int currentSw3Reading = digitalRead(sw3);

    if (currentSw3Reading != lastSw3Reading) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (currentSw3Reading != sw3ButtonState) {
        sw3ButtonState = currentSw3Reading;

        if (sw3ButtonState == LOW) {
          sw3ToggleState = !sw3ToggleState; // Flip the state
          
          if (sw3ToggleState) {
            // ENTERING ANGLE MODE
            blinkCount = 10;     // 5 on + 5 off
            blinkInterval = 100; 
            lastBlinkTime = millis();
          } else {
            // ENTERING ACRO MODE
            blinkCount = 4;      // 2 on + 2 off
            blinkInterval = 50; 
            lastBlinkTime = millis();
          }
        }
      }
    }

    lastSw3Reading = currentSw3Reading;

    if (digitalRead(sw1) == LOW) {
      dataToRX.arm = 1;
      dataToRX.disarm = 0;
      digitalWrite(armled, HIGH); 
    } else {
      dataToRX.arm = 0;
      dataToRX.disarm = 1;
      digitalWrite(armled, LOW);  
    }

    if (digitalRead(sw2) == LOW) {
      dataToRX.aux2 = 2000;
    } else {
      dataToRX.aux2 = 1000;
    }

    if (sw3ToggleState) {    
      dataToRX.aux3 = 2000;
    } else {
      dataToRX.aux3 = 1000;
    }

    if (blinkCount > 0) {
      if (millis() - lastBlinkTime >= blinkInterval) {
        lastBlinkTime = millis();
        digitalWrite(nrfled, blinkCount % 2); 
        blinkCount--;
      }
    } else {
      digitalWrite(nrfled, !radio.isChipConnected()); 
    }

    // Transmit
    radio.write(&dataToRX, sizeof(packet));
  }
}


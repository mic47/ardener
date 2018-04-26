
#include <SerialESP8266wifi.h>
#include <SoftwareSerial.h>


/*
 * Constant / configuration.
 */

const int moistureSensorPin = A0;   
const int motorPin = 9;
const int sleepSwitchPin = 2;

const int motorInterval = 200;
const int motorBreakInterval = 800;

const int sleepIntervalSeconds = 11; 
const int warmupIntervalSeconds = 10;

SoftwareSerial wifiSerial(7, 8);

/* 
 *  Moisture state
 */


int moistureSensorValue = 0; 

enum MoistureState {
  HighMoisture = 0,
  NormalMoisture = 1,
  LowMoisture = 2
};

/*
 * Watering state
 */

enum WateringState {
  Watering = 0,
  NotWatering = 1
};

typedef struct {
  MoistureState moisture;
  WateringState watering;
} State;

State state;
unsigned long last_display_time = 0;

/*
 *
 */

/*
 * Read sensors and decide on the moisture state.
 */
void updateMoisture(MoistureState &moisture) {
  moistureSensorValue = analogRead(moistureSensorPin);
  if (moistureSensorValue >= 450) {
    moisture = LowMoisture;
  } else if (moistureSensorValue >= 350) {
    moisture = NormalMoisture;
  } else {
    moisture = HighMoisture;
  }
}

/*
 * Decide whether to water this cycle or not.
 */
void updateWatering(State &state) {
  switch (state.moisture) {
    case LowMoisture:
      state.watering = Watering;
      break;
    case HighMoisture:
      state.watering = NotWatering;
      break;
    case NormalMoisture:
      // Keep state as it is.
      break;
    default:
      Serial.print("ERROR: moisture state is in unknown level!!!\n");
      break;
  }
}

/*
 * Do watering if necessary.
 */
void maybePerformWatering(const WateringState &watering) {
  Serial.print("Sri\n");
  switch (watering)  {
      case Watering:
        digitalWrite(motorPin, HIGH);
        delay(motorInterval);
        digitalWrite(motorPin, LOW);
        delay(motorBreakInterval);
        break;
      case NotWatering:
        digitalWrite(motorPin, LOW);
        break;
      default:
        // Serial.print("ERROR: unknown watering state. This should never happen!!!\n");
        break;
  }
}

/*
 * In case we are done with current measurement, turn off sensors and wait predefined
 * period. After that, turn-on sensors, wait warmup period. Do nothing if we are watering.
 */
bool canSleep = false;
void maybeWaitForNextPeriod(const WateringState &watering) {
  switch (watering) {
    case NotWatering:
      // Turn off sensitive sensors.
      if (canSleep) {
        switchSensors(LOW);
        longDelayInSeconds(sleepIntervalSeconds, "sleeping");
      } else {
        canSleep = true;
      }
      // Turn them on, and wait until they initialize properly.
      switchSensors(HIGH);
      longDelayInSeconds(warmupIntervalSeconds, "warmup");
      // Now the sensor readings are good, we can proceed.  
      break; 
    case Watering:
      // If we are watering, we should not wait.
      break;
    default:
      // Serial.print("ERROR: unknown watering state. This should never happen!!!\n");
      break;
  }
}

void setup() {
  // Start debugging serial mode.
  Serial.begin(9600);
  state.moisture = HighMoisture;
  state.watering = NotWatering;
  pinMode(motorPin, OUTPUT);
  pinMode(moistureSensorPin, INPUT);
  pinMode(sleepSwitchPin, OUTPUT);

  // Initialize connection to serial wifi.
  wifiSerial.begin(9600);
}


void loop() {
  maybeWaitForNextPeriod(state.watering);
  updateMoisture(state.moisture);
  updateWatering(state);
  maybePerformWatering(state.watering);
}

// Tools

void switchSensors(int output) {
  digitalWrite(sleepSwitchPin, output);
}

void longDelayInSeconds(int seconds, const char *msg) {
  for(int i=seconds ; i > 0; i--) {
    delay(1000);
  }
}

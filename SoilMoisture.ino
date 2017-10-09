
#include <LiquidCrystal.h>


const char* moistureStrings[] = {
  "High",
  "Norm",
  "Low"
};

const char* wateringStrings[] = {
  "ON",
  "OFF"
};

/*
 * Constant / configuration.
 */

const int moistureSensorPin = A0;   
const int motorPin = 9;
const int sleepSwitchPin = 2;

const int motorInterval = 200;
const int motorBreakInterval = 800;

const int sleepIntervalSeconds = 3600; 
const int warmupIntervalSeconds = 10;

LiquidCrystal lcd(12, 11, 6, 5, 4, 3);

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
        Serial.print("ERROR: unknown watering state. This should never happen!!!\n");
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
      Serial.print("ERROR: unknown watering state. This should never happen!!!\n");
      break;
  }
}

/*
 * Update LCD screen: dump the State, show phase and how long will it take to switch it.
 */
char msg[17] = "";
void updateLCD(const State &state, const char* phase, const int howLong) {
  snprintf(
    msg,
    sizeof(msg),
    "%s (%d) [%s]                ", 
    moistureStrings[state.moisture],
    moistureSensorValue, 
    wateringStrings[state.watering]
  );
  lcd.setCursor(0, 0);lcd.print(msg);
  snprintf(
    msg,
    sizeof(msg),
    "%s %ds.               ",
    phase,
    howLong
  );
  lcd.setCursor(0, 1);lcd.print(msg);
}

void setup() {
  state.moisture = HighMoisture;
  state.watering = NotWatering;
  lcd.begin(16, 2);
  pinMode(motorPin, OUTPUT);
  pinMode(moistureSensorPin, INPUT);
  pinMode(sleepSwitchPin, OUTPUT);
}


void loop() {
  maybeWaitForNextPeriod(state.watering);
  updateMoisture(state.moisture);
  updateWatering(state);
  updateLCD(state, "Measuring", 0);
  maybePerformWatering(state.watering);
}

// Tools

void switchSensors(int output) {
  digitalWrite(sleepSwitchPin, output);
}

void longDelayInSeconds(int seconds, const char *msg) {
  for(int i=seconds ; i > 0; i--) {
    updateLCD(state, msg, i);
    delay(1000);
  }
}

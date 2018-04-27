
// Using forked version of ESP library: https://github.com/mic47/ESP8266wifi
#include <SerialESP8266wifi.h>
#include <SoftwareSerial.h>


class Sensor {
  const int sensorPin;
  const char* serialName;

  int sensorValue;

  public:
  Sensor(int pin, const char* serialName):
    sensorPin(pin),
    serialName(serialName),
    sensorValue(0) 
  {}

  void init() {
    pinMode(this->sensorPin, INPUT);
  }

  int update() {
    this->sensorValue = analogRead(this->sensorPin);
    return this->sensorValue;
  }

  int getValue() {
    return this->sensorValue;
  }

  String serialize() {
    return String(this->serialName) + String(this->sensorValue);
  }
};

/**
 * Some notes:
 *
 * There should be only sensor configuration, and then logic how to decide
 * when to water. It could be some sort of easy classifier (decision tree, logistic 
 * regression, or even neural net).
 *
 * On other hand, sensor configuration is fixed per device, so code might be 
 * generated (or manually written). But each configuration should be representable
 * as some canonical item, and configurations with same representation (i.e. types of 
 * sensors with same meaning, but ignoring pins, which are implementation details)
 * should be able to use same models.
 * 
 * Additionally, configuration should be pot-dependent. And 1 device should be able to 
 * handle multiple pots. What is pot? Pot should contains some sensors, button for
 * watering. Pots should not depend in each other, but can contain multiple stuff.
 * Pot is basically unit that you water at the same time.
 *
 * Also, sensors should be separate class.
 */

class PlantPot {

  Sensor sensors[1] = {
    Sensor(A0, "m")
  };

  public:

  enum Sensors {
    MoistureSensor = 0,
  };

  String serialize() {
    String ret = "";
    bool first = true;
    for(auto &s: this->sensors) {
      if (first) {
        first = false;
      } else {
        ret += ",";
      }
      ret += s.serialize();
    }
    return ret;
  }

  void update() {
    for(auto &s: this->sensors) {
      s.update();
    }
  }

  void init() {
    for(auto &s: this->sensors) {
      s.init();
    }
  }

  int getValue(Sensors sensor) {
    return sensors[sensor].getValue();
  }

} plantPot;



/*
 * Constant / configuration.
 */

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


enum MoistureState {
  HighMoisture = 0,
  NormalMoisture = 1,
  LowMoisture = 2,
};

/*
 * Watering state
 */

enum WateringState {
  Watering = 0,
  NotWatering = 1,
};

typedef struct {
  MoistureState moisture;
  WateringState watering;
} State;

State state;
unsigned long last_display_time = 0;

/*
 * Read sensors and decide on the moisture state.
 */
void updateMoisture(MoistureState &moisture) {
  auto moistureSensorValue = plantPot.getValue(PlantPot::MoistureSensor);
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
  pinMode(sleepSwitchPin, OUTPUT);

  plantPot.init();

  // Initialize connection to serial wifi.
  wifiSerial.begin(9600);
}


void loop() {
  maybeWaitForNextPeriod(state.watering);
  plantPot.update();
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

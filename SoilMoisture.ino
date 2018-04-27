
// Using forked version of ESP library: https://github.com/mic47/ESP8266wifi
#include <SerialESP8266wifi.h>
#include <SoftwareSerial.h>


class Sensor {
  public:
  enum SensorType {
    Digital = 0,
    Analog = 1,
  };

  private:
  const int sensorPin;
  const char* serialName;

  int sensorValue;
  bool serializable;
  SensorType type;

  public:

  Sensor(int pin, SensorType type, const char* serialName, bool serializable):
    sensorPin(pin),
    serialName(serialName),
    type(type),
    serializable(serializable),
    sensorValue(0)
  {}

  void init() {
    pinMode(this->sensorPin, INPUT);
  }

  int update() {
    if (this->type == Digital) {
      this->sensorValue = digitalRead(this->sensorPin);
    } else {
      this->sensorValue = analogRead(this->sensorPin);
    }
    return this->sensorValue;
  }

  int getValue() {
    return this->sensorValue;
  }

  String serialize() {
    return String(this->serialName) + String(this->sensorValue);
  }

  bool shouldSerialize() {
    return serializable;
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

  const char identifier[] = "First configuration";

  Sensor sensors[2] = {
    Sensor(A0, Sensor::Analog, "m", true),
    // This is weird, because there is likely to be only 1 in every pot. But
    // Let's keep it simple for now.
    Sensor(5, Sensor::Digital, "?", true),
  };

  public:

  enum Sensors {
    MoistureSensor = 0,
    WateringButton = 1,
  };

  String serialize() {
    String ret = "";
    bool first = true;
    for(auto &s: this->sensors) {
      if (!s.shouldSerialize()) {
        continue;
      }
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

  const char* getIdentifier() {
    return this->identifier;
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

SoftwareSerial wifiSerial(3, 4);
SerialESP8266wifi wifi = SerialESP8266wifi(wifiSerial, wifiSerial, -1, Serial);

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

enum WateringStateEnum {
  Watering = 0,
  NotWatering = 1,
};

enum WateringSourceEnum {
  AutomatedWatering = 0,
  ManualWatering = 1,
};

struct WateringState {
  int wateringCount;
  WateringStateEnum wateringState;
  WateringSourceEnum wateringSource;
};

typedef struct {
  WateringState watering;
} State;

State state;
unsigned long last_display_time = 0;

/*
 * Decide on moisture state.
 */
MoistureState classifyMoisture(const PlantPot &pot) {
  auto moistureSensorValue = pot.getValue(PlantPot::MoistureSensor);
  if (moistureSensorValue >= 650) {
    return LowMoisture;
  } else if (moistureSensorValue >= 450) {
    return NormalMoisture;
  } else {
    return HighMoisture;
  }
}

/*
 * Decide whether to water this cycle or not.
 */
void updateWateringState(const PlantPot &pot, State &state) {
  auto manualWatering = pot.getValue(PlantPot::WateringButton);
  auto moisture = classifyMoisture(pot);
  if (manualWatering) {
    state.watering.wateringState = Watering;
    state.watering.wateringCount += 1;
    state.watering.wateringSource = ManualWatering;
  } else {
    switch (moisture) {
      case LowMoisture:
        state.watering.wateringState = Watering;
        state.watering.wateringSource = AutomatedWatering;
        state.watering.wateringCount += 1;
        break;
      case HighMoisture:
        state.watering.wateringState = NotWatering;
        state.watering.wateringCount = 0;
        state.watering.wateringSource = AutomatedWatering;
        break;
      case NormalMoisture:
        if (
          state.watering.wateringState == Watering
          && state.watering.wateringSource != ManualWatering
        ) {
          state.watering.wateringCount += 1;
        } else {
          state.watering.wateringState = NotWatering;
          state.watering.wateringCount = 0;
          state.watering.wateringSource = AutomatedWatering;
        }
        // Keep state as it is.
        break;
      default:
        Serial.print("ERROR: moisture state is in unknown level!!!\n");
        break;
    }
  }
}

/*
 * Do watering if necessary.
 */
void maybePerformWatering(const WateringState &watering) {
  switch (watering.wateringState)  {
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
  switch (watering.wateringState) {
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
  state.watering.wateringState = NotWatering;
  state.watering.wateringCount = 0;
  pinMode(motorPin, OUTPUT);
  pinMode(sleepSwitchPin, OUTPUT);

  plantPot.init();

  wifiSerial.begin(9600);
  // Initialize connection to serial wifi.
  if(wifi.begin()) {
    Serial.println("Wifi is initialized");
  } else {
    Serial.println("Wifi failed to initialize");
  }
}


void loop() {
  maybeWaitForNextPeriod(state.watering);
  plantPot.update();
  State previousState = state;
  updateWateringState(plantPot, state);

  stateChangeHooks(previousState, state, plantPot);

  maybePerformWatering(state.watering);
  sendMeasurements(plantPot);
}

void stateChangeHooks(const State &prev, const State &cur, const PlantPot &pot) {
  if (
    prev.watering.wateringState == Watering
    && cur.watering.wateringState == NotWatering
  ) {
      sendWateringAction(pot, prev.watering);
  }
}

void sendWateringAction(const PlantPot &pot, WateringState watering) {
  bool ok = wifi.connectToServer("192.168.0.47", 2347);
  if (!ok) {
    Serial.println("ActionSend: Connecting to server failed.");
    return;
  }

  ok = wifi.send(SERVER, "i ", false);
  ok &= wifi.send(SERVER, pot.getIdentifier(), true);
  if (!ok) {
    Serial.println("ActionSend: Identifier message failed to send.");
    return;
  }
  char buf[32];
  sprintf(
    buf, "a %s %d",
    watering.wateringSource == AutomatedWatering
      ? "AutomatedWatering"
      : "ManualWatering",
    watering.wateringCount
  );
  ok = wifi.send(SERVER, buf, true);
  wifi.disconnectFromServer();
}

bool sendMeasurements(PlantPot &pot) {
  bool ok = wifi.connectToServer("192.168.0.47", "2347");
  if (!ok) {
    Serial.println("Connecting to server failed.");
    return;
  }

  ok = wifi.send(SERVER, "i ", false);
  ok &= wifi.send(SERVER, pot.getIdentifier(), true);
  if (!ok) {
    Serial.println("Identifier message failed to send.");
    return false;
  }
  ok = wifi.send(SERVER, "m ", false);
  ok &= wifi.send(SERVER, pot.serialize().c_str(), true);
  //ok &= wifi.send(SERVER, "\n", true);
  if (!ok) {
    Serial.println("Measurement message failed to be sent.");
    return false;
  }
  wifi.disconnectFromServer();
  return true;
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

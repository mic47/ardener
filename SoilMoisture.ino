
// Using forked version of ESP library: https://github.com/mic47/ESP8266wifi
#include <SerialESP8266wifi.h>
#include <SoftwareSerial.h>

const int RETRIES = 1;

const int motorInterval = 200;
const int motorBreakInterval = 800;

const int sleepIntervalSeconds = 15 * 60;
const int warmupIntervalSeconds = 10;

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

  int getValue() const {
    return this->sensorValue;
  }

  String serialize() const {
    return String(this->serialName) + String(this->sensorValue);
  }

  bool shouldSerialize() const {
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

#define IDENTIFIER "Kanary0"

class PlantPot {
  const char identifier[sizeof(IDENTIFIER)] = IDENTIFIER;

  Sensor sensors[4] = {
    Sensor(A0, Sensor::Analog, "m", true), // Soil moisture
    Sensor(A1, Sensor::Analog, "l", true), // Light
    Sensor(A2, Sensor::Analog, "t", true), // Temperature
    Sensor(A3, Sensor::Analog, "n", true), // Soil moisture
  };

  int wateringButtonPin = 11;
  int sleepSwitchPin = 12;
  int motorPin = 13;

  public:

  bool wateringButtonIsPressed() const {
    bool output = digitalRead(this->wateringButtonPin) == HIGH;
    if (output) {
      Serial.println("Watering button is pressed");
    }
    return output;
  }

  void turnSensorsOff() const {
    digitalWrite(this->sleepSwitchPin, LOW);
  }

  void turnSensorsOn() const {
    digitalWrite(this->sleepSwitchPin, HIGH);
  }

  void performWatering() const {
    digitalWrite(this->motorPin, HIGH);
    delay(motorInterval);
    digitalWrite(this->motorPin, LOW);
    delay(motorBreakInterval);
  }

  enum Sensors {
    MoistureSensor = 0,
    LightSensor = 1,
    TemperatureSensor = 2,
    MoistureSensor2 = 3,
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
    pinMode(this->wateringButtonPin, INPUT);
    pinMode(this->sleepSwitchPin, OUTPUT);
    pinMode(this->motorPin, OUTPUT);
  }

  int getValue(Sensors sensor) const {
    return sensors[sensor].getValue();
  }

  const char* getIdentifier() const {
    return this->identifier;
  }

};

/*
 * Communication tools.
 */

SoftwareSerial wifiSerial(2, 3);
SerialESP8266wifi wifi = SerialESP8266wifi(wifiSerial, wifiSerial, NO_RESET, Serial);

/*
 *  Moisture state
 */


enum MoistureState {
  HighMoisture = 0,
  NormalMoisture = 1,
  LowMoisture = 2,
};

/*
 * State management.
 */


enum State {
  Sleeping = 0,
  SensorWarmup = 1,
  Measuring = 2,
  AutomatedWatering = 3,
  ManualWatering = 4,
};


typedef struct {
  // Does not make sense outside of ManualWatering or AutomatedWatering.
  int wateringCycles;
  // If in Sleeping mode, then it's time when device went to sleep,
  // otherwise it's when device "woke up".
  unsigned int timeSinceSleepChange;
} StateStats;

// This could be in one struct, and then in array, in case arduino would
// control more than 1 pot.
State state;
StateStats stats;
PlantPot plantPot;

const char* ppState(const State &state) {
  const char* ret = NULL;
  switch (state) {
    case Sleeping: ret = "Sleeping"; break;
    case SensorWarmup: ret =  "SensorWarmup"; break;
    case Measuring: ret =  "Measuring"; break;
    case AutomatedWatering: ret =  "AutomatedWatering"; break;
    case ManualWatering: ret =  "ManualWatering"; break;
    default: ret =  "ERROR_STATE"; break;
  }
  return ret;
};

/*
 * Perform the core action that is done in each state.
 */
void performStateAction(
  PlantPot &pot,
  const State &state,
  StateStats &stats
) {
  switch (state) {
    case Sleeping:
      // Do nothing
      break;
    case SensorWarmup:
      // Do nothing
      break;
    case Measuring:
      // Read measurements and cache results.
      pot.update();
      break;
    case AutomatedWatering:
    case ManualWatering:
      pot.performWatering();
      stats.wateringCycles += 1;
      break;
    default:
      Serial.println("Unknown state. This should never happen.");
      // TODO: this should also send error to wifi?
      break;
  }
}

/*
 * Move to different transition.
 */
State decideOnStateTransition(
  const PlantPot &pot,
  const State state,
  const StateStats stats
) {
  if (pot.wateringButtonIsPressed()) {
    return ManualWatering;
  }
  switch (state) {
    case Sleeping:
      if(elapsedSeconds(stats.timeSinceSleepChange, sleepIntervalSeconds)) {
        return Measuring;
      }
      break;
    case SensorWarmup:
      if (elapsedSeconds(stats.timeSinceSleepChange, warmupIntervalSeconds)) {
        return Measuring;
      }
      break;
    case Measuring:
    case AutomatedWatering: {
      auto moisture = classifyMoisture(pot);
      bool shouldWater =
        moisture == LowMoisture ||
        (moisture == NormalMoisture && state == AutomatedWatering);
      if (shouldWater) {
        return AutomatedWatering;
      } else if (state == Measuring) {
        // Go and sleep;
        return Sleeping;
      } else {
        // Measure once more time after watering.
        // TODO: would be nice to have delay to let water disperse everywhere.
        return Measuring;
      }
      break;
    }
    case ManualWatering:
      if (!pot.wateringButtonIsPressed()) {
        return SensorWarmup;
      }
      break;
    default:
      Serial.println("Unknown state. This should never happen.");
      // TODO: this should also send error to wifi?
      break;
  }
  return state;
}

#define RETRY_WIFI(fun, retries) ({ \
  bool ok = (fun); \
  for (int i = 0; i < (retries) && !ok ; i++) {\
    Serial.println("ERROR found. Resetting wifi.");\
    wifi.begin();\
    ok = (fun);\
  }\
  if (!ok) {\
    Serial.println("UNABLE to do wifi action");\
  }\
  ok;\
})

#define WIFI_ACTION(action) ({\
  bool ok = connectToServer();\
  delay(10);\
  if (ok) {\
    ok &= (action);\
  }\
  disconnectFromServer();\
  ok;\
})

void printTime() {
  Serial.print("[time=");
  Serial.print(millis()/1000);
  Serial.print(".");
  Serial.print(millis()%1000);
  Serial.print("] ");
}

void stateChangeTriggers(
  PlantPot &pot,
  const State &prev,
  const State &cur,
  StateStats &stats
) {
  if (prev == cur) {
    // Skip if there is no state change (currently there is not useful logic here.
    return;
  }
  printTime();
  Serial.print("STATE CHANGE: ");
  Serial.print(ppState(prev));
  Serial.print(" ");
  Serial.println(ppState(cur));
  
  // From now on, I assume prev.state != cur.state
  if (prev == Sleeping) {
    pot.turnSensorsOn();
    stats.timeSinceSleepChange = millis();
  }
  if (cur == Sleeping) {
    pot.turnSensorsOff();
    stats.timeSinceSleepChange = millis();
  }
  if (prev == Measuring) {
    RETRY_WIFI(WIFI_ACTION(sendMeasurements(pot)), RETRIES);
  }
  if (prev == AutomatedWatering || prev == ManualWatering) {
    RETRY_WIFI(WIFI_ACTION(sendWateringAction(pot, prev, stats)), RETRIES);
    // TODO: Maybe I could reset this when moving from non-watering state to
    // watering state.
    stats.wateringCycles = 0;
  }

  printTime();
  Serial.println("STATE CHANGE TRIGGERS DONE");
}

/*
 * Decide on moisture state.
 */
MoistureState classifyMoisture(const PlantPot &pot) {
  auto moistureSensorValue = pot.getValue(PlantPot::MoistureSensor);
  moistureSensorValue += pot.getValue(PlantPot::MoistureSensor2);
  moistureSensorValue /= 2;
  if (moistureSensorValue >= 650) {
    return LowMoisture;
  } else if (moistureSensorValue >= 450) {
    return NormalMoisture;
  } else {
    return HighMoisture;
  }
}

void setup() {
  // Start debugging serial mode.
  Serial.begin(9600);
  state = SensorWarmup;
  stats.timeSinceSleepChange = millis();
  stats.wateringCycles = 0;

  plantPot.init();

  wifiSerial.begin(9600);
  // Initialize connection to serial wifi.
  Serial.println("Initializing wifi");
  if(wifi.begin()) {
    Serial.println("Wifi is initialized");
  } else {
    Serial.println("Wifi failed to initialize");
  }
}

void loop() {
  performStateAction(plantPot, state, stats);
  State newState = decideOnStateTransition(plantPot, state, stats);
  stateChangeTriggers(plantPot, state, newState, stats);
  state = newState;
}

/*
 * Sending messages over network.
 */
bool sendIdentifier(const PlantPot &pot) {
  bool ok = wifi.send(SERVER, "i ", false);
  ok &= wifi.send(SERVER, pot.getIdentifier(), true);
  if (!ok) {
    Serial.println("Identifier message failed to send.");
    return false;
  }
  return ok;
}

bool sendWateringAction(
  const PlantPot &pot,
  const State watering,
  const StateStats &stats
) {
  bool ok = sendIdentifier(pot);
  if (!ok) return false;
  char buf[32];
  sprintf(
    buf, "a %s %d",
    watering == AutomatedWatering
      ? "AutomatedWatering"
      : "ManualWatering",
    stats.wateringCycles
  );
  ok = wifi.send(SERVER, buf, true);
  if (!ok) {
    Serial.println("Error while sending watering action to server.");
  }
  return ok;
}

bool sendMeasurements(PlantPot &pot) {
  bool ok = sendIdentifier(pot);
  if (!ok) return false;
  ok = wifi.send(SERVER, "m ", false);
  ok &= wifi.send(SERVER, pot.serialize().c_str(), true);
  //ok &= wifi.send(SERVER, "\n", true);
  if (!ok) {
    Serial.println("Measurement message failed to be sent.");
    return false;
  }
}

/*
 * Tools for doing networking and debugging.
 */

bool connectToServer() {
  bool ok = wifi.connectToServer("192.168.100.14", "2347");
  if (!ok) {
    Serial.println("Failed to connect. Reseting wifi.");
    wifi.begin();
    if (!wifi.connectToServer("192.168.100.14", "2347")) {
      Serial.println("Connecting to server failed again.");
      return false;
    }
  }
  return true;
}

bool disconnectFromServer() {
  wifi.disconnectFromServer();
}

bool elapsedSeconds(unsigned int since, unsigned int seconds) {
  return millis() - since >= seconds * 1000;
}

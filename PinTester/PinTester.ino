
void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  for (int i = 2; i < 13; i++) {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);

}

char buf[128]={'\0'};

int getPin(int pin) {
  if (pin > 0) return pin;
  switch(-pin) {
    case 0: return A0;
    case 1: return A1;
    case 2: return A2;
    case 3: return A3;
    case 4: return A4;
    case 5: return A5;
  }
  return 0;
}

int analogWatch[20];
int analogWatchLen = 0;
int digitalWatch[20];
int digitalWatchLen = 0;

void loop() {

  if (analogWatchLen + digitalWatchLen > 0) {
    Serial.print("Sensors watch:");
    for (int i=0; i<analogWatchLen; i++) {
      Serial.print(" A");
      Serial.print(String(-analogWatch[i]).c_str());
      Serial.print("=");
      Serial.print(String(analogRead(getPin(analogWatch[i]))).c_str());
    }
    for (int i=0; i < digitalWatchLen; i++) {
      Serial.print(" ");
      Serial.print(String(digitalWatch[i]).c_str());
      Serial.print("=");
      Serial.print(String(digitalRead(getPin(digitalWatch[i]))).c_str());
    }
    Serial.println("");
  }
  auto ff = Serial.readBytesUntil('\n', buf, 128);
  Serial.println("Give command");
  buf[ff] = 0;
  if (!strcmp(buf, "")) {
    return;
  }
  Serial.print("Command received: ");
  Serial.println(buf);
  char cmd[10];
  int pin;
  auto ret = sscanf(buf, "%s %d", cmd, &pin);
  if (ret >= 2) {
    if (!strcmp(cmd, "read")) {
      if (pin > 0) {
        auto x = String(digitalRead(pin));
        Serial.println(x.c_str());
      } else {
        auto x = String(analogRead(getPin(pin)));
        Serial.println(x.c_str());
      }
    } else if (!strcmp(cmd, "input")) {
      pinMode(getPin(pin), INPUT);
    } else if (!strcmp(cmd, "output")) {
      pinMode(getPin(pin), OUTPUT);
    } else if (!strcmp(cmd, "high")) {
      digitalWrite(pin, HIGH);
    } else if (!strcmp(cmd, "low")) {
      digitalWrite(pin, LOW);
    } else if (!strcmp(cmd, "awatch")) {
      analogWatch[analogWatchLen++] = pin;
    } else if (!strcmp(cmd, "dwatch")) {
      digitalWatch[digitalWatchLen++] = pin;
    } else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  } else {
    Serial.println("Error: Expecting at least 2 tokens!");
  }
  Serial.println("Done");
}

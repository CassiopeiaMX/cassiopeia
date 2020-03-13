const int left_dir_pin = 2;
const int left_pwm_pin = 3;
const int right_dir_pin = 4;
const int right_pwm_pin = 5;
const int arm_dir_pin = 7;
const int arm_pwm_pin = 6;
const int shovel_dir_pin = 8;
const int shovel_pwm_pin = 9;
const int interrupt_button_pin = 12;

unsigned long last_update_time = 0.0;
unsigned long interrupt_button_vibration_time = 100.0;
unsigned long interrupt_time = 0.0;
bool waiting_for_interrupt = false;
unsigned long stop_time = 1000.0;

int combinations[9][2] = {
  { 1,  1},
  { 1,  0},
  { 1, -1},
  { -1,  0},
  { -1, -1},
  { 0, -1},
  { -1,  1},
  { 0,  1},
  { 0,  0}
};

void arm_low() {
  digitalWrite(arm_dir_pin, LOW);
  digitalWrite(arm_pwm_pin, LOW);
}

void all_low() {
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_pwm_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_pwm_pin, LOW);
  arm_low();
  digitalWrite(shovel_dir_pin, LOW);
  digitalWrite(shovel_pwm_pin, LOW);
}

void setup() {
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  pinMode(arm_dir_pin, OUTPUT);
  pinMode(arm_pwm_pin, OUTPUT);
  pinMode(shovel_dir_pin, OUTPUT);
  pinMode(shovel_pwm_pin, OUTPUT);
  pinMode(interrupt_button_pin, INPUT);
  all_low();
  Serial.begin(9600);
}

void write_motor(int dir_pin, int pwm_pin, int throttle) {
  if (throttle == 0) {
    digitalWrite(pwm_pin, LOW);
  }
  else {
    digitalWrite(pwm_pin, HIGH);
    if (throttle > 0) {
      digitalWrite(dir_pin, HIGH);
    }
    else {
      digitalWrite(dir_pin, LOW);
    }
  }
}

void write_motors(int l, int r) {
  write_motor(left_dir_pin, left_pwm_pin, l);
  write_motor(right_dir_pin, right_pwm_pin, r);
}

void write_arm(int arm, int shovel) {
  write_motor(arm_dir_pin, arm_pwm_pin, arm);
  write_motor(shovel_dir_pin, shovel_pwm_pin, shovel);
}

void loop() {
  if (millis() - last_update_time > stop_time) {
    all_low();
    last_update_time = millis();
    Serial.println("stop time");
  }
  if (digitalRead(interrupt_button_pin) == HIGH) {
    if (!waiting_for_interrupt) {
      waiting_for_interrupt = true;
      interrupt_time = millis();
    }
    else {
      if (millis() - interrupt_time > interrupt_button_vibration_time) {
        arm_low();
        Serial.println("interrupt button");
      }
    }
  }
  while (Serial.available() > 0) {
    String rx = Serial.readStringUntil('\n');
    if (rx.length() != 2 && !(rx.length() == 3 || rx.charAt(2) == ' ')) {
      break;
    }
    if (!(isDigit(rx.charAt(0)))) {
      break;
    }
    if (!(isDigit(rx.charAt(1)))) {
      break;
    }
    int code = rx.toInt();
    if (code <= 0 || code > 99) {
      break;
    }
    if (code == 99) {
      waiting_for_interrupt = false;
    }
    int arm = code % 10;
    code = code - arm;
    code = code / 10;
    int dir = code;
    write_motors(combinations[dir - 1][0], combinations[dir - 1][1]);
    write_arm(combinations[arm - 1][0], combinations[arm - 1][1]);
    last_update_time = millis();
  }
}

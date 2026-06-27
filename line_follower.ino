#define AIN1_PIN 33
#define AIN2_PIN 25
#define PWMA_PIN 32
#define BIN1_PIN 27
#define BIN2_PIN 14
#define PWMB_PIN 26
#define STBY_PIN 13 

// SENSOR PINS
const int sensorPins[8] = {21, 34, 35, 36, 39, 18, 19, 22};

// PID CONSTANTS
float Kp = 8.5;    
float Ki = 0.001;  
float Kd = 15.0;   

int maxSpeed = 255;
int minPWM = 60;   

float lastError = 0;
float integral = 0;

int weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

void setup() {
  Serial.begin(115200);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);

  for(int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  delay(2000); // Wait 2 seconds before starting
}

void loop() {
  int sensors[8];
  bool onLine = false;
  int sum = 0;
  int active = 0;

  // Read Sensors
  for(int i = 0; i < 8; i++) {
    sensors[i] = digitalRead(sensorPins[i]);
    if(sensors[i]) {
      onLine = true;
      sum += weights[i];
      active++;
    }
  }

  // ===== EMERGENCY RECOVERY (Line Lost) =====
  if(!onLine) {
    if(lastError < 0) setMotor(-160, 160); // Spin left
    else setMotor(160, -160);              // Spin right
    return;
  }

  // ===== ERROR CALCULATION =====
  float error = (active > 0) ? (float)sum / active : lastError;

  // ===== DYNAMIC SPEED CONTROL =====
  int baseSpeed;
  if(abs(error) > 3) {
    baseSpeed = 120; // Sharp turn detected, slow down
  }
  else {
    baseSpeed = 160; // Straight or slight turn, maintain speed
  }

  // ===== PID CALCULATION =====
  float P = error;
  integral = constrain(integral + error, -50, 50);
  float D = error - lastError;

  float adjustment = (Kp * P) + (Ki * integral) + (Kd * D);
  lastError = error;

  // Calculate individual motor speeds
  int leftMotorSpeed = baseSpeed + adjustment;
  int rightMotorSpeed = baseSpeed - adjustment;

  setMotor(leftMotorSpeed, rightMotorSpeed);
}

// ================= MOTOR FUNCTION =================
void setMotor(int left, int right) {
  // Constrain to physical limits
  left = constrain(left, -maxSpeed, maxSpeed);
  right = constrain(right, -maxSpeed, maxSpeed);

  // Apply deadzone compensation
  if(abs(left) > 0 && abs(left) < minPWM) left = (left > 0) ? minPWM : -minPWM;
  if(abs(right) > 0 && abs(right) < minPWM) right = (right > 0) ? minPWM : -minPWM;

  // Left Motor
  digitalWrite(AIN1_PIN, (left >= 0));
  digitalWrite(AIN2_PIN, (left < 0));
  analogWrite(PWMA_PIN, abs(left));

  // Right Motor
  digitalWrite(BIN1_PIN, (right >= 0));
  digitalWrite(BIN2_PIN, (right < 0));
  analogWrite(PWMB_PIN, abs(right));
}
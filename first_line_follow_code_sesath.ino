// ===== MOTOR PINS =====
#define LM1 5
#define LM2 6
#define RM1 9
#define RM2 10

// ===== SENSOR PINS =====
#define S1 4   // far left
#define S2 3
#define S3 2   // center
#define S4 7
#define S5 8   // far right

int s1, s2, s3, s4, s5;

// ===== PID CONSTANTS =====
float Kp = 25;
float Kd = 15;

int error = 0;
int lastError = 0;

int baseSpeed = 100;

void setup() {
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
}

// ===== MAIN LOOP =====
void loop() {

  readSensors();
  calculateError();

  int correction = Kp * error + Kd * (error - lastError);

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  motorDrive(leftSpeed, rightSpeed);

  lastError = error;
}

// ===== READ SENSORS =====
void readSensors() {
  s1 = digitalRead(S1);
  s2 = digitalRead(S2);
  s3 = digitalRead(S3);
  s4 = digitalRead(S4);
  s5 = digitalRead(S5);
}

// ===== ERROR CALCULATION =====
// black = 0 → line detected
void calculateError() {

  if(!s3) error = 0;

  else if(!s2) error = -2;
  else if(!s4) error = 2;

  else if(!s1) error = -4;
  else if(!s5) error = 4;

  else {
    // if line lost → keep last direction
    error = lastError;
  }
}

// ===== MOTOR CONTROL =====
void motorDrive(int L, int R) {

  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  // LEFT MOTOR
  if(L > 0){
    analogWrite(LM1, L);
    analogWrite(LM2, 0);
  } else {
    analogWrite(LM1, 0);
    analogWrite(LM2, -L);
  }

  // RIGHT MOTOR (reversed wiring)
  if(R > 0){
    analogWrite(RM1, 0);
    analogWrite(RM2, R);
  } else {
    analogWrite(RM1, -R);
    analogWrite(RM2, 0);
  }
}
#include <QTRSensors.h>
#include <BTS7960.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
//Col sensor + MUX
#define MUX_ADDR 0x70
#define NUM_SENSORS 3
bool col1=false;
bool col2=false;
bool col3=false;
bool col4=false;
int colnum=0;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);
void followLine();
int sarvo=0;
// Select mux channel
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read one sensor and decide if it sees blue
bool isBlue1(uint8_t channel) {
  tcaSelect(channel);
  delay(5);
  tcs.begin(); 
  int count = 0;

  for (int i = 0; i < 3; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    if (c == 0) continue;

    float rn = (float)r / c;
    float bn = (float)b / c;

    if (bn > rn * 1.2) {
      count++;
    }

    delay(3);
  }

  return count >= 2;
}
// Check all sensors and print which ones detected blue
void detectBlueStrips() {
  bool foundAny = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (isBlue1(i)) {
      if (i==0){
        col1=true;
        colnum++;
      }
      if(i==1){
        col2=true;
        colnum++;
      }
      if (i==2){
        col3=true;
        colnum++;
      }
      foundAny = true;
    } 
  }
  
  if (colnum!=2){
    col4=true;
  }

}
void colreset(){
  col4=false;
  col1=false;
  col2=false;
  col3=false;
  colnum=0;
}

//
Servo panServo;
Servo tiltServo;
// solenoid
int relayPin = 32;
// reset
int Flag=0;
int venda =0;
bool ignoreJunction = false;
int junctionCount=0;
unsigned long Junction_Debounce_MS=500;
int stopped=0;
int reset =0;
int State=2;
int check_Blue = 0;
int check_reset = 0;
int check_1_reset =0;
const int Right=49;
const int Left = 53;
//Left Motor
const int L_EN=2;
const int LM_PWM_L = 5;   // Direction pin
const int LM_PWM_R = 6;   // Direction pin
// const int LM2_PWM_L = 11;   // Direction pin
// const int LM2_PWM_R = 12;   
//Right Motor
const int R_EN=12;
const int RM_PWM_L = 9;   // Direction pin
const int RM_PWM_R = 10;   // Direction pin
// const int RM2_PWM_L = 13;   // Direction pin
// const int RM2_PWM_R = 46;
// QTR Array 
const uint8_t qtrPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT]; // holds calibrated readings
// Target line position (depends on calibration)
int reference = 3500;

// TCS3200 Pins
#define S0  2
#define S1  4
#define S2  7
#define S3  8
#define OUT 3

// ------------------- PD CONSTANTS -------------------
double kp = 0.05;
double kd = 0.03;
double lastError = 0;
// ------------------- SPEED LIMITS -------------------
int BASE_SPEED = 140;   // mid speed (0–255)
int MAX_SPEED  = 150;
int TURN_SPEED =140;
BTS7960 motorController01(L_EN,LM_PWM_L,LM_PWM_R);
BTS7960 motorController02(R_EN,RM_PWM_L,RM_PWM_R); 
// BTS7960 motorController03(L_EN,LM2_PWM_L,LM2_PWM_R);
// BTS7960 motorController04(R_EN,RM2_PWM_L,RM2_PWM_R); 
void setLeftMotor(int speed) { 
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    // Forward
    motorController01.Enable();
    motorController01.TurnRight(speed);
    // motorController03.Enable();
    // motorController03.TurnRight(speed);       // forward PWM
  } else {
    // Reverse
    motorController01.Enable();
    motorController01.TurnLeft(-speed);  
    // motorController03.Enable();
    // motorController03.TurnRight(-speed);     // reverse PWM
  }
}
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    // Forward
    motorController02.Enable();
    motorController02.TurnRight(speed); 
    // motorController04.Enable();
    // motorController04.TurnRight(speed);        // forward PWM
  } else {
    // Reverse
    motorController02.Enable();
    motorController02.TurnLeft(-speed);
    // motorController04.Enable();
    // motorController04.TurnRight(-speed);      // reverse PWM
  }
}
void moveBackward(int speed) {
  speed = constrain(speed, 0, 255);
  setLeftMotor(-speed);
  setRightMotor(-speed);
}
const int LINE_THRESHOLD = 700;    // same as your original detect
const int CENTER_THRESHOLD = 750;  // same as your original center detect
const int WHITE_THRESHOLD = 400;   // threshold for "no line" (adjust after calibration)
unsigned long lastJunctionTime = 0;
bool lastJunctionState = LOW;
void checkJunction() {
  bool currentState;
  unsigned long currentTime=millis();
  if (State == 0) {
    currentState =  (digitalRead(Right));
  }
  else if (State == 2) {
  currentState = digitalRead(Left);}
   else if (State == 3) {
   currentState = ( digitalRead(Left)&&(!digitalRead(Right)));
   }
   else if (State == 4) {
   currentState = ( digitalRead(Left)||(!digitalRead(Right)));
  }
  if (lastJunctionState == HIGH && currentState == LOW) {
  if (currentTime-lastJunctionTime>Junction_Debounce_MS){
    junctionCount++;
    lastJunctionTime=currentTime;
    }
  }

  lastJunctionState = currentState;
}

// making it a state machine
enum RobotState {
  FOLLOW_LINE,
  TURNING,
  STOPPED,
  CHECK
};
RobotState currentState = FOLLOW_LINE;
void turnR(int turnTime = 300) { // ms, adjust if needed
  unsigned long start = millis();
  while (millis() - start < turnTime) {
    setLeftMotor(BASE_SPEED);
    setRightMotor(-BASE_SPEED);
  }  
}
void turnl(int turnTime = 300) { // ms, adjust if needed
  unsigned long start = millis();
  while (millis() - start < turnTime) {
    setLeftMotor(-BASE_SPEED);
    setRightMotor(BASE_SPEED);
  }
}
// MPU

#define MPU_ADDR 0x68

float gyroZ_offset = 0;
float angleZ = 0;
unsigned long prevTime;
// -------- MPU INIT --------
void initIMU() {
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(500);
  calibrateGyro();

  prevTime = millis();
}


// -------- CALIBRATION --------
void calibrateGyro() {
  int samples = 500;
  float sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += readRawGyroZ();
    delay(5);
  }

  gyroZ_offset = sum / samples;
}

// -------- RAW READ --------
int16_t readRawGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  return Wire.read() << 8 | Wire.read();
}

// -------- DEG/S --------
float getGyroZ() {
  return (readRawGyroZ() - gyroZ_offset) / 131.0;
}

// -------- UPDATE ANGLE --------
void updateAngle() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  angleZ += getGyroZ() * dt;
}

void GRight() {
  angleZ = 0;
  prevTime = millis();
  unsigned long timeout = millis();

  while (true) {
    updateAngle();

    if (millis() - timeout > 1500) break;

    float error = -90 - angleZ;   // target = -90°

    // ---- slow down as you approach target ----
    if (error < -40) {
      setLeftMotor(TURN_SPEED);
      setRightMotor(-TURN_SPEED);
    }
    else if (error < -25) {
      setLeftMotor(130);
      setRightMotor(-130);
    }
    else if (error < -1) {
      setLeftMotor(107);
      setRightMotor(-107);
    }
    else {
      break; // close enough → stop
    }
  }

  // ACTIVE BRAKE (same as left)
  setLeftMotor(0);
  setRightMotor(0);
  delay(30);
}
void GLeft() {
  angleZ = 0;
  prevTime = millis();
  unsigned long timeout = millis();

  while (true) {
    updateAngle();

    if (millis() - timeout > 1500) break;

    float error = 90 - angleZ;  // target 90°

    // ---- slow down as you approach target ----
    if (error > 40) {
      setLeftMotor(-TURN_SPEED);
      setRightMotor(TURN_SPEED);
    }
    else if (error > 25) {
      setLeftMotor(-130);
      setRightMotor(130);
    }
    else if (error > 0.4) {
      setLeftMotor(-107);
      setRightMotor(107);
    }
    else {
      break; // close enough → stop
    }
  }

  // ACTIVE BRAKE (VERY IMPORTANT)
  setLeftMotor(0);
  setRightMotor(0);
  delay(30);
  setLeftMotor(0);
  setRightMotor(0);
}


void stopRobot(int durationMs) {
  unsigned long start = millis();
  while((millis() - start < durationMs)){
   setLeftMotor(0);
   setRightMotor(0);
  }
}
bool allSensorsWhite(int whiteThresh = WHITE_THRESHOLD) {
  qtr.read(sensorValues);
  for (int i = 7; i < SENSOR_COUNT; ++i) {
    if (sensorValues[i] > whiteThresh) return false;
  }
  return true;
}
bool lineDetectedCenter() {
  return (sensorValues[5] > 800 && sensorValues[4] > 800);
}
// --- Utility: check any sensor on left side or right side sees line ---
bool anyLeftSideLine(int thresh = LINE_THRESHOLD) {
  qtr.read(sensorValues);
  for (int i = 0; i <= 3; ++i) if (sensorValues[i] > thresh) return true;
  return false;
}
bool anyRightSideLine(int thresh = LINE_THRESHOLD) {
  qtr.read(sensorValues);
  for (int i = 4; i <= 7; ++i) if (sensorValues[i] > thresh) return true;
  return false;
}
void turnRight90() {
  // --- PHASE 1: CLEAR THE INTERSECTION ---
 setLeftMotor(100);
 setRightMotor(100);
 delay(90);
  // --- PHASE 2: COARSE TURN (Fast) ---
  unsigned long timeout21 = millis();
  while (millis()-timeout21<100){
  setLeftMotor(TURN_SPEED);
  setRightMotor(-TURN_SPEED);
}
  delay(150); // Blind spin to get off the old line clearly
  unsigned long timeout = millis();
  while (true) {
    qtr.readLineBlack(sensorValues);
    setLeftMotor(BASE_SPEED);
    setRightMotor(-BASE_SPEED);
    // If any sensor (0-7) sees the line, we are close
    bool anything = false;
    for(int i=0; i<8; i++) {if( sensorValues[0]>700&&sensorValues[1]>700) {anything = true;}};
    if (anything) break;
    if (millis() - timeout > 1500) break; // Safety
  }
  // --- PHASE 4: FINE ALIGNMENT (Slow) ---
  // This is the Secret Sauce: Slow down to lock the center
  while (true) {
   qtr.readLineBlack(sensorValues);
    // Slow speed for precision
    setLeftMotor(90);  // Reduced speed
    setRightMotor(-90);
    // Stop exactly when center is locked
    if (lineDetectedCenter()) {
        // Active Braking: Briefly reverse motors to kill momentum
        setLeftMotor(-50);
        setRightMotor(50);
        delay(30);
        break;
    }
  }
  stopRobot(100); // Short pause to let physics settle
  // Reset PID error so it doesn't "remember" the turn error
}

void turnLeft90() {
  // --- PHASE 1: CLEAR THE INTERSECTION ---
  // setLeftMotor(100);
  // setRightMotor(100);
  // delay(70);
  // --- PHASE 2: COARSE TURN (Fast) ---
  unsigned long timeout22 = millis();
  while (millis()-timeout22<290){
  setLeftMotor(-TURN_SPEED);
  setRightMotor(TURN_SPEED);
}
  delay(150); // Blind spin to get off the old line clearly
  unsigned long timeout2 = millis();
  while (true) {
    qtr.readLineBlack(sensorValues);
    setLeftMotor(-TURN_SPEED);
    setRightMotor(TURN_SPEED);
    // If any sensor (0-7) sees the line, we are close
    bool anything = false; 
    {if(sensorValues[6] >700||sensorValues[7] >700)
    { anything = true;}
    }
    if (anything) break;
    if (millis() - timeout2 > 1500) break; // Safety
  }
  while (true) {
    qtr.readLineBlack(sensorValues);
    // Slow speed for precision
    setLeftMotor(-80);  // Reduced speed
    setRightMotor(80);
    // Stop exactly when center is locked
    if (lineDetectedCenter()) {
        // Active Braking: Briefly reverse motors to kill momentum
        setLeftMotor(50);
        setRightMotor(-50);
        delay(30);
        break;
    }
  }
  stopRobot(100); // Short pause to let physics set  // Reset PID error so it doesn't "remember" the turn error
    }
unsigned long readBlue() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delayMicroseconds(100);
  unsigned long pulse =pulseIn(OUT, LOW, 10000);
  return pulse;
}

int readRed() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(2);
  return pulseIn(OUT, LOW);
}
int readGreen() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(2);
  return pulseIn(OUT, LOW);
}

bool isBlue() {
  int r = readRed();
  int b = readBlue();

  // Blue surface: red pulse will be noticeably larger than blue pulse
  if (r > b * 1.59) {   // 30% difference, tune this
    return true;
  }

  return false;
}


// firing mechanism
void fireSolenoid() {
  digitalWrite(relayPin, LOW);   // turn relay ON (fire)
  delay(90);             // keep it ON
  digitalWrite(relayPin, HIGH);  // turn relay OFF
}

int currentPan = 90;
int currentTilt = 90;

void setPan(int angle) {
  angle = constrain(angle, 0, 180);

  while (currentPan != angle) {
    if (currentPan < angle) currentPan++;
    else currentPan--;

    panServo.write(currentPan);
    delay(15);  // small delay → smooth motion
  }
}

void setTilt(int angle) {
  angle = constrain(angle, 0, 180);

  while (currentTilt != angle) {
    if (currentTilt < angle) currentTilt++;
    else currentTilt--;

    tiltServo.write(currentTilt);
    delay(10);  // same delay → consistent speed
  }
}
//
void followLine() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - reference;
  int correction = kp * error + kd * (error - lastError);
  lastError = error;

  int leftSpeed  = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}
void asta(int x1,int y1,int x2,int y2,int x3,int y3, int x4, int y4){
  detectBlueStrips();
  if (col1==true){
    setPan(x1);
    delay(1000);
    setTilt(y1);
    delay(300);
    fireSolenoid();
    delay(1000);
  }
  if (col2==true){
    setPan(x2);
    delay(1000);
    setTilt(y2);
    delay(300);
    fireSolenoid();
    delay(1000);
  }
  if (col3==true){
    setPan(x3);
    delay(1000);
    setTilt(y3);
    delay(300);
    fireSolenoid();
    delay(1000);
  }
  if (col4==true){
    setPan(x4);
    delay(1000);
    setTilt(y4);
    delay(300);
    fireSolenoid();
    delay(1000);
  }
  colreset();
  setPan(90);
  setTilt(90);
}

float targetAngle = 0;
void goStraightGyro(int baseSpeed) {
  updateAngle();

  float error = targetAngle - angleZ;

  float Kp = 2.1;
  int correction = 0;

  if (abs(error) > 0.5) {
    correction = Kp * error;
  }

  correction = constrain(correction, -40, 40);

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void setup()
{ 
  Serial.begin(9600);
  initIMU();
  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);
// junction
  pinMode(Left,INPUT_PULLUP);
  pinMode(Right, INPUT_PULLUP);//INPUT
  // Calibration (recommended to rotate the robot left–right)
  for (int i = 0; i < 90; i++) {
    qtr.calibrate();
  }
  
  //pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Set Frequency scaling to 20%
  //digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  //LPWM and RPWM basically depends on how you connect the 
  // MOTOR FUNCTION
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  panServo.attach(44);
  tiltServo.attach(45);

  // center position
  panServo.write(90);
  tiltServo.write(90);
  // mux
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    delay(10);}
  } 


void loop(){


  checkJunction();

  switch (currentState) {

    case FOLLOW_LINE:
      followLine();
      // Decide if we need to turn at a junction
      if (junctionCount == 3 && reset==0 &&Flag==0) {
        
        currentState = TURNING;
      }// Case 1
       else if (junctionCount == 3 && reset==1 &&Flag==0 ) {
        currentState = CHECK;
        unsigned long current=millis();
        while(millis()-current<200){
        setLeftMotor(100);
        setRightMotor(100);
        }
        check_Blue=1;
        State=2;
      }
       else if (junctionCount==1 && reset==2 && Flag==1){
        currentState=TURNING;
        State=2;
       }
       else if(junctionCount==3 && reset==3 && Flag==1){
        currentState=TURNING;
        State == 2;
       }
       else if(junctionCount==3 && reset==3 && Flag==1){
        
       }
       //Case2
       else if (junctionCount == 1 && reset==2 && Flag==0 ) {
        currentState = TURNING;
      }else if (junctionCount == 1 && reset==3 &&Flag==0 ) {
        currentState = CHECK;
        unsigned long current1=millis();
        while(millis()-current1<200){
        setLeftMotor(100);
        setRightMotor(100);
        }
        State == 2;
      }else if (junctionCount == 2 && reset==4 &&Flag==2 ) {
        currentState = TURNING;
      }// Case 3
       else if (junctionCount == 1 && reset==4 &&Flag==0 ) {
        currentState = TURNING;
        unsigned long current20=millis();
        while(millis()-current20<100){
        followLine();
        }
        State == 2;
      }
      else if (junctionCount == 1 && reset==50 &&Flag==0 ) {
        currentState = CHECK;
        unsigned long current2=millis();
        while(millis()-current2<100){
        setLeftMotor(100);
        setRightMotor(100);
        }
      }
      else if (junctionCount == 1 && reset==5 &&Flag==3 ) {
        currentState = TURNING;
      }
      // Case 4
      else if (junctionCount == 1 && reset==5 && Flag==0 ) {
        currentState = TURNING;
        State == 0;
      }
      else if (junctionCount == 1 && reset==6 &&Flag==0 ) {
        currentState = TURNING;
        State == 0;
      }
      else if (junctionCount == 2 && reset==7 &&Flag==0 ) {
        currentState = TURNING;
      }
      else if (junctionCount == 2 && reset==8 &&Flag==0 ) {
        currentState = TURNING;
        State == 2;
      }
       // the unification
       else if(junctionCount==3 && reset==99){
        currentState=STOPPED;
        // change here
       }
       else if(junctionCount==2 && reset==100){
        currentState=TURNING;
       }
       else if(junctionCount==0 && reset==101){
        currentState=TURNING;
       }
       //
       
  
      break;


    case TURNING:
     if (junctionCount == 3 && reset==0) {
        GLeft();
        unsigned long current100=millis();
        while(millis()-current100<100){
        setLeftMotor(100);
        setRightMotor(100);
        }
        junctionCount=0;
        reset=1;
      }// Case1
      else if (junctionCount == 1 && reset==2 && Flag==1){
        unsigned long current333=millis();
        while(millis()-current333<100){
        setLeftMotor(100);
        setRightMotor(100);
        }
        GLeft();
        junctionCount=0;
        reset =3;
      }
      else if(junctionCount==3 && reset==3 && Flag==1){
        GRight();
        junctionCount=0;
        reset=99;
       }
       //Case 2
      else if (junctionCount == 1 && reset==2 && Flag==0 ) {
        GLeft();
        unsigned long current1133=millis();
        while(millis()-current1133<255){
        followLine();
        }
        junctionCount=0;
        reset =3;
      }
      else if (junctionCount == 2 && reset==4 &&Flag==2 ) {
        GRight();
        junctionCount=0;
        reset=99;
      }//
      //Case 3
      else if (junctionCount == 1 && reset==4 &&Flag==0 ) {
        stopRobot(100);
        GLeft();
        unsigned long current229=millis();
        while(millis()-current229<350){
        followLine();
        }
        junctionCount=0;
        reset=50;
      }
      else if (junctionCount == 1 && reset==5 &&Flag==3 ){
        GRight();
        junctionCount=0;
        while(junctionCount==0){
          followLine();
          checkJunction();
        }
        junctionCount=0;
        reset=99;
      }
      //Case 4
      else if (junctionCount == 1 && reset==5 &&Flag==0 ) {
        GLeft();
        unsigned long current402=millis();
        while(millis()-current402<350){
        followLine();
        }
        junctionCount=0;
        reset=6;
      }
      else if (junctionCount == 1 && reset==6 &&Flag==0 ) {
        unsigned long current174=millis();
        while(millis()-current174<70){
        setLeftMotor(100);
        setRightMotor(100);
        }
        setLeftMotor(0);
        setRightMotor(0);
        delay(50);
        GLeft();
        stopRobot(1000);
        unsigned long current1124=millis();
        while(millis()-current1124<350){
        setLeftMotor(100);
        setRightMotor(100);
        }
        setLeftMotor(0);
        setRightMotor(0);
        asta(105,51,67,51,110,100,67,100);
        delay(10);
        unsigned long current114=millis();
        while(millis()-current114<133){
        setLeftMotor(-100);
        setRightMotor(-100);
        }
        GLeft();
        State=0;
        junctionCount=0;
        reset=7;
        BASE_SPEED=100;
      }
      else if (junctionCount == 2 && reset==7 && Flag==0 ) {
        GRight();
        junctionCount=0;
        reset=8;
      }
      else if (junctionCount == 2 && reset==8 &&Flag==0 ) {
        junctionCount=0;
        reset=99;
      }

      //Unify
      //  
    else if(junctionCount==2 && reset==100){

      State = 2;
    
      GLeft();
      delay(300);   // let robot stabilize

      angleZ = 0;
      targetAngle = 0;
      prevTime = millis();

      junctionCount = 0;

      while (venda != 1) {
        checkJunction();
        if (junctionCount== 0 || junctionCount== 1 || junctionCount== 2){ 
        goStraightGyro(195);
        int BASE_SPEED = 170; 
        }
        else if (junctionCount == 3){
          goStraightGyro(205);
          followLine();
        }
        else if (junctionCount == 4 || junctionCount == 5) {
          delay(55);
          setLeftMotor(0);
          setRightMotor(0);
          delay(100);
          GRight();
          delay(100);
          venda = 1;
          break;
        }
      }
      if (venda ==1){
        unsigned long current135=millis();
          while(millis()-current135<1290){
            followLine();
          }
          setLeftMotor(0);
          setRightMotor(0);
          asta(105,53,65,53,110,101,65,101);
          unsigned long current163=millis();
          while(millis()-current163<2411){
            setLeftMotor(-100);
            setRightMotor(-100);
          }
          stopRobot(15000); 
    }

    junctionCount = 0;
    reset = 101;
  }
       
    else if (junctionCount == 0 && reset==101){
        
    }
    
    currentState = FOLLOW_LINE; // back to line following after turn
    break;

    case STOPPED:
     if (stopped==0){
      unsigned long current117=millis();
      while(millis()-current117<137){
        followLine();
      }
      stopRobot(100);
      GLeft();
      stopRobot(1500); 
      unsigned long current115=millis();
      while(millis()-current115<597){
      followLine();
      }
      setLeftMotor(0);
      setRightMotor(0);
      asta(104,53,65,53,110,104,65,104);
      unsigned long current153=millis();
      while(millis()-current153<377){
      setLeftMotor(-100);
      setRightMotor(-100);
      }
      unsigned long tame78=millis();
         // VERY IMPORTANT
      GLeft();
      stopped=3;
      junctionCount=0;
      reset=100;
      BASE_SPEED=60;

     }currentState = FOLLOW_LINE;
     break;





    case CHECK:
    if (check_Blue==1){
      stopRobot(1000);
      if(isBlue()){
        Flag=1;
        GLeft();
        stopRobot(1000);
        unsigned long current109=millis();
        while(millis()-current109<260){
        setLeftMotor(100);
        setRightMotor(100);
        }
        setLeftMotor(0);
        setRightMotor(0);
        asta(107,53,65,53,110,100,65,100);
        unsigned long current123=millis();
        while(millis()-current123<149){
        setLeftMotor(-100);
        setRightMotor(-100);
        }
        GRight();
        unsigned long current209=millis();
        while(millis()-current209<150){
        followLine();
        }
        //
        //
        junctionCount=0;
        reset=2;
      }
      else{
        check_Blue=2;
        junctionCount=0;
        reset=2;
      }
    }
    else if(check_Blue==2){
      stopRobot(1000);
      if(isBlue()){
        Flag=2;
        GLeft();
        stopRobot(1000);
        unsigned long current110=millis();
        while(millis()-current110<305){
        setLeftMotor(100);
        setRightMotor(100);
        }
        setLeftMotor(0);
        setRightMotor(0);
        asta(107,53,65,53,110,101,65,101);
        unsigned long current231=millis();
        while(millis()-current231<211){
        setLeftMotor(-100);
        setRightMotor(-100);
        }
        turnRight90();
        unsigned long current219=millis();
        while(millis()-current219<100){
        followLine();
        }
        junctionCount=0;
        reset=4;
        BASE_SPEED=100;
      }
      else{
        check_Blue=3;
        junctionCount=0;
        reset=4;
      }
      }

      else if(check_Blue==3){
      unsigned long current319=millis();
      while(millis()-current319<50){
        followLine();
      }
      stopRobot(1000);
      if(isBlue()){
        Flag=3;
        GLeft();
        stopRobot(1000);
        unsigned long current11123=millis();
        while(millis()-current11123<211){
        setLeftMotor(100);
        setRightMotor(100);
        }
        setLeftMotor(0);
        setRightMotor(0);
        asta(105,53,62,53,110,101,65,101);
        unsigned long current232=millis();
        while(millis()-current232<187){
        setLeftMotor(-100);
        setRightMotor(-100);
        }
        GLeft();
        delay(100);
        GLeft();
        junctionCount=0;
        reset=5;
        BASE_SPEED=100;
      }
      else{
        check_Blue=4;
        junctionCount=0;
        reset=5;
      }

    }
    currentState = FOLLOW_LINE; 
    break;

  }

   }
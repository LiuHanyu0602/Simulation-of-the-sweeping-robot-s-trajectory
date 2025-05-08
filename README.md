# Simulation-of-the-sweeping-robot-s-trajectory
Comparison of Trajectory Coverage for Two Behavioral Models

**The robotic vacuum cleaner is be able to:**
Clean an area of 1000 sq ft in one charge cycle.
Detect and avoid physical obstacles during operation.
Automatically return to a charging station either after cleaning or when the dust bag exceeds a set weight.
Trigger an audible alarm (e.g., buzzer) when the dust bag weight reaches the preset threshold.

**Arduino 代码**
#include <MeMCore.h>
// #include "MeOrion.h"

MeDCMotor m1(M1);
MeDCMotor m2(M2);
MeBuzzer buzzer;
MeUltrasonicSensor ultraSensor(PORT_3);

uint8_t mSpeed = 90;
unsigned long lastHitTime = 0;
unsigned long prevHitTime = 0;
bool nowBuzzing = false; 

// 卡角检测涉及到的变量
int lastHitSide = -1; // -1:初始化， 0 ：左边撞线，1 ：右边撞线
int switchCount = 0;
unsigned long lastSwitchTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(9, INPUT);
  m1.run(-mSpeed);
  m2.run(mSpeed);
}

void loop() {
  int start = millis();
  //  0. IR回充
  int IR = digitalRead(9);
  Serial.print("IR: ");
  Serial.println(IR);
  if (IR){
    stopMotors();
    return;
  }
  //  1. 超声波避障
  long distance = ultraSensor.distanceCm();
  if (distance > 0 && distance < 10) { // 这个10cm可能还是有点近了
    stopMotors();
    backOff(300);
  //随机左转或者右转避障
    int avoidDirection = random(0, 2); // 0：向左转 1：向右转
    if (avoidDirection == 0) {
      rotateLeft(400);
    }
    else {
      rotateRight(400);
    }

    goForward();
    return;  
  }

  // 2. 压力传感器 + 蜂鸣器（压力传感器接 A0）
  int pressure = analogRead(A0);
  Serial.print("Pressure now is: ");
  Serial.println(pressure);

  if (pressure >= 600) {
    if (!nowBuzzing) {
      buzzer.tone(440, 1000);  
      nowBuzzing = true;
    }
  } 
  else {
    buzzer.noTone();
    nowBuzzing = false;
  }

  //  3. 红外
  bool L = digitalRead(11); // 左传感器，0 = 黑线（撞线）
  bool R = digitalRead(12); // 右传感器

  bool hitL = !L; //左传感器实时状态
  bool hitR = !R; //右传感器实时状态

  //计算连续两次撞击的时间间隔 
  unsigned long now = millis();
  unsigned long interval = now - lastHitTime;

  // 左边撞线：右转 
  if (hitL && !hitR) {
    processHit(0);           // 左边撞线
    handleTurn(1, interval); // 右转
  }
  //右边撞线：左转
  else if (!hitL && hitR) {
    processHit(1);           // 右边撞线
    handleTurn(0, interval); // 左转
  }

  //没撞线，正常前进
  else if (!hitL && !hitR) {
    goForward();             // 正常前进
  }

  // 两个传感器都撞线，垂直，随机向左或向右转
  else if (hitL && hitR) {
    prevHitTime = lastHitTime;
    lastHitTime = now;
    interval = lastHitTime - prevHitTime;

    // stopMotors();
    // delay(200);
    backOff(300);

    int angleTime = map(interval, 200, 4000, 700, 1500);
    angleTime = constrain(angleTime, 700, 1500);
  
    rotateRandom(angleTime);
    goForward();
  }

  delay(40);
  int end = millis();
  int delta = end - start;
  Serial.print("delta: ");
  Serial.println(delta);
}

//撞线行为处理 & 判断是否卡角,是否（>=3）
void processHit(int side) {
  unsigned long now = millis();

  //连续两次撞线的传感不同，且两次间隔小于1秒，计作一次连续压线的累积
  if (side != lastHitSide && (now - lastSwitchTime < 1500)) {
    switchCount++;
    lastSwitchTime = now;
  }
  else {
    switchCount = 0;
  }

  lastHitSide = side;

  if (switchCount >= 3) {
    stopMotors();
    backOff(400);
    rotateRandom(random(800, 1200)); // 随机180~270度大角度旋转脱困
    //两个变量重置
    switchCount = 0;
    lastHitSide = -1;
  }

  prevHitTime = lastHitTime;
  lastHitTime = now;
}

//执行左右转+角度动态调整函数
void handleTurn(int dir, unsigned long interval) {
  stopMotors();
  delay(100);

  int turnTime = map(interval, 200, 4000, 200, 800);
  turnTime = constrain(turnTime, 200, 800);

  if (dir == 0) rotateLeft(turnTime);
  else rotateRight(turnTime);

  goForward();
}

//运动函数
void goForward() {
  m1.run(-mSpeed);
  m2.run(mSpeed);
}

void stopMotors() {
  m1.stop();
  m2.stop();
}

void backOff(int t) {
  m1.run(mSpeed);
  m2.run(-mSpeed);
  delay(t);
}

void rotateLeft(int t) {
  m1.run(mSpeed);
  m2.run(mSpeed);
  delay(t);
}

void rotateRight(int t) {
  m1.run(-mSpeed);
  m2.run(-mSpeed);
  delay(t);
}

//为了垂直撞到黑线的情况
void rotateRandom(int t) {
  int dir = random(0, 2);
  if (!dir) {
    m1.run(mSpeed);
    m2.stop();
    delay(t);
  }
  else {
    m2.run(-mSpeed);
    m1.stop();
    delay(t);
  }
}


With a very limited number of sensors, we simulated the closed-loop control of ‘sense-judge-decide-act-feedback’ as much as possible in code. This is the basis of the core behavioural logic of a real sweeping robot. This algorithm realizes adaptive steering according to the timing and intensity of collision by dynamically mapping the intervals of consecutive line-bumping events into rotation angles. In addition, when the number and time of alternating left and right collisions are monitored to exceed a threshold value, a backward plus large-angle random steering extrication strategy is triggered.


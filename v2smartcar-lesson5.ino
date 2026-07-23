/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Arduino Smart Car Tutorial Lesson 5
 * Tutorial URL http://osoyoo.com/2018/12/19/osoyoo-robot-car-kit-lesson-4-obstacle-avoidance-robot-car/
 * CopyRight www.osoyoo.com

 * This project will show you how to make Osoyoo robot car in auto drive mode and avoid obstacles
 *   
 * 
 */
#include <Servo.h>
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 3   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  11    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   ///Left Motor direction pin 1 to MODEL-X IN4

#define SERVO_PIN     9  //servo connect to D9

#define Echo_PIN    2 // Ultrasonic Echo pin connect to D11
#define Trig_PIN    10  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN     13
#define SPEED  160     //both sides of the motor speed
#define TURN_SPEED  140     //both sides of the motor speed
#define SHARP_TURN_INNER_SPEED  70   //inside wheel speed for tight turns
#define SHARP_TURN_OUTER_SPEED  200  //outside wheel speed for tight turns

const int distancelimit = 30; //distance limit for obstacles in front           
const int targetLeftDistance = 20;
const int leftDistanceTolerance = 4;
const int sharpFrontDistance = 18;
const int sharpLeftDistanceOffset = 10;
const int frontSensorAngle = 90;
const int leftSensorAngle = 180;
const unsigned long controlIntervalMs = 200;
const unsigned long sensorSettleMs = 50;
const unsigned long sensorSampleGapMs = 150;
unsigned long lastControlAt = 0;

Servo head;
/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Left()  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Left_Sharp()  //Turn left with differential speed for tight corners
{
  // Differential steering: inside wheel turns slower while outside wheel turns faster.
  go_Left();
  set_Motorspeed(SHARP_TURN_INNER_SPEED, SHARP_TURN_OUTER_SPEED);
}
void go_Right_Sharp()  //Turn right with differential speed for tight corners
{
  // Differential steering: inside wheel turns slower while outside wheel turns faster.
  go_Right();
  set_Motorspeed(SHARP_TURN_OUTER_SPEED, SHARP_TURN_INNER_SPEED);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(0,0);
}

/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void buzz_ON()   //open buzzer
{
  
  for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
  }
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
}

int medianOfThree(int first, int second, int third)
{
  if ((first <= second && second <= third) || (third <= second && second <= first)) {
    return second;
  }
  if ((second <= first && first <= third) || (third <= first && first <= second)) {
    return first;
  }
  return third;
}

/*detection of ultrasonic distance*/
int watch(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH,25000UL);
  if(echo_distance == 0){
    return 200;
  }
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}

int readDistanceAt(int angle){
  head.write(angle);
  delay(sensorSettleMs);

  int firstSample = watch();
  delay(sensorSampleGapMs);
  int secondSample = watch();
  delay(sensorSampleGapMs);
  int thirdSample = watch();

  return medianOfThree(firstSample, secondSample, thirdSample);
}

int readFrontDistance(){
  return readDistanceAt(frontSensorAngle);
}

int readLeftDistance(){
  return readDistanceAt(leftSensorAngle);
}

void turn_Left_Manual(bool sharpTurn)  //optional manual control helper
{
  if (sharpTurn) {
    go_Left_Sharp();
  } else {
    set_Motorspeed(TURN_SPEED, TURN_SPEED);
    go_Left();
  }
}

void turn_Right_Manual(bool sharpTurn)  //optional manual control helper
{
  if (sharpTurn) {
    go_Right_Sharp();
  } else {
    set_Motorspeed(TURN_SPEED, TURN_SPEED);
    go_Right();
  }
}

void auto_avoidance(){

  unsigned long now = millis();

  if (now - lastControlAt < controlIntervalMs) {
    return;
  }
  lastControlAt = now;

  int frontDistance = readFrontDistance();
  int leftDistance = readLeftDistance();

  if (frontDistance <= distancelimit) {
    if (frontDistance <= sharpFrontDistance) {
      go_Right_Sharp();
    } else {
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      go_Right();
    }
  } else if (leftDistance > targetLeftDistance + leftDistanceTolerance) {
    if (leftDistance > targetLeftDistance + leftDistanceTolerance + sharpLeftDistanceOffset) {
      go_Left_Sharp();
    } else {
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      go_Left();
    }
  } else if (leftDistance < targetLeftDistance - leftDistanceTolerance) {
    if (leftDistance < targetLeftDistance - leftDistanceTolerance - sharpLeftDistanceOffset) {
      go_Right_Sharp();
    } else {
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      go_Right();
    }
  } else {
    set_Motorspeed(SPEED, SPEED);
    go_Advance();
  }
}

void setup() {
  /*setup L298N pin mode*/
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();//stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
  buzz_OFF(); 

  digitalWrite(Trig_PIN,LOW);
  /*init servo*/
  head.attach(SERVO_PIN); 
  head.write(frontSensorAngle);
  delay(1000);
  
  Serial.begin(9600);
 
}

void loop() {
  auto_avoidance();
}

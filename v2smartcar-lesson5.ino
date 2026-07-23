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
#define SPEED  180     //both sides of the motor speed
#define TURN_SPEED  160     //both sides of the motor speed
#define BACK_SPEED1  160     //back speed
#define BACK_SPEED2  160     //back speed

const int distancelimit = 30; //distance limit for obstacles in front           
const int targetLeftDistance = 18;
const int leftDistanceTolerance = 4;
const int turntime = 400; //Time the robot spends turning (miliseconds)
const int backtime = 200; //Time the robot spends turning (miliseconds)

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
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
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

void alarm(){
   buzz_ON();
 
   buzz_OFF();
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

int readFrontDistance(){
  head.write(90);
  delay(180);
  return watch();
}

int readLeftDistance(){
  head.write(180);
  delay(180);
  return watch();
}

int readRightDistance(){
  head.write(10);
  delay(180);
  return watch();
}

void driveForward(int leftSpeed, int rightSpeed, int durationMs){
  set_Motorspeed(leftSpeed, rightSpeed);
  go_Advance();
  delay(durationMs);
  stop_Stop();
}

void auto_avoidance(){

  int frontDistance = readFrontDistance();
  int leftDistance = readLeftDistance();

  if (frontDistance <= distancelimit) {
    stop_Stop();
    alarm();
    set_Motorspeed(BACK_SPEED1,BACK_SPEED2);
    go_Back();
    delay(backtime);
    stop_Stop();

    int rightDistance = readRightDistance();
    if (rightDistance > distancelimit) {
      set_Motorspeed(TURN_SPEED,TURN_SPEED);
      go_Right();
      delay(turntime);
    } else {
      set_Motorspeed(TURN_SPEED,TURN_SPEED);
      go_Left();
      delay(turntime * 2);
    }
    stop_Stop();
  } else if (leftDistance > targetLeftDistance + leftDistanceTolerance) {
    driveForward(SPEED, TURN_SPEED - 35, backtime + 100);
  } else if (leftDistance < targetLeftDistance - leftDistanceTolerance) {
    driveForward(SPEED, TURN_SPEED, backtime);
  } else {
    driveForward(SPEED, SPEED, backtime);
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
  head.write(90);
   delay(2000);
  
  Serial.begin(9600);
 
}

void loop() {
  auto_avoidance();
}

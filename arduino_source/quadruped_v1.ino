/* -----------------------------------------------------------------------------
 *  Project: Quadruped Robot
 *  Author:  Jason L Verhagen (jlverhagen@gmail.com)
 *  Website: www.newboticslabs.com
 *  Date:    04/10/2016
 *  
 *  This updated project is a modified version of this:
 *  https://github.com/regishsu/SpiderRobot
 *  
 *  This project uses a modified upper base plate to accommodate a Parrot
 *  AR Drone 2.0 battery (aftermarket battery with balance charging pigtail) 
 *  and uses an Arduino Mega 2560 (clone) and a SainSmart Mega Sensor 
 *  Shield v2.0. The power supply is a Hobbyking YEP 20A HV (2~12S) SBEC.
 *  The project also uses an HC-SR04 ultrasonic range module and IR obstacle
 *  module from a sunfounder.com sensor kit. The project also uses a HC-06
 *  bluetooth module to send and receive data from the robot.
 * ----------------------------------------------------------------------------- 
 *  Orignial program by panerqiang@sunfounder.com for 
 *  Remote Control Crawling Robot @ sunfounder.com 1/27/2015:
 *  http://www.sunfounder.com/learn/category/Quadruped-Crawling-Robot-Kit-for-Arduino.html
 *  
 *  And edited by Regis for spider project:
 *  (http://www.instructables.com/id/DIY-Spider-RobotQuad-robot-Quadruped/)
 *  
 *  Mods and additions for Mega 2560 include:
 *    -Add HC-SR04 ultrasonic range module (echo=pin 15, trigger=pin 16)
 *    -Add IR obstacle module (from sunfounder sensor kit) (pin 17)
 *    -Add HC-06 bluetooth module to Serial1 (pins 18 an 19)
 *    -Moved last servo from pin 13 to pin 14 to avoid pin 13 LED conflict
 *    -Replace demo code in loop and integrate sonar/IR data for movement
 * ---------------------------------------------------------------------------*/


/* Includes ------------------------------------------------------------------*/
#include <Servo.h>          //to define and control servos
#include <FlexiTimer2.h>    //to set a timer to manage all servos
#include <NewPing.h>        //to define and read sonar module
/* ---------------------------------------------------------------------------*/

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 14} };
/* ---------------------------------------------------------------------------*/

/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* ---------------------------------------------------------------------------*/

/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
/* ---------------------------------------------------------------------------*/

/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* ---------------------------------------------------------------------------*/

/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/

/* Ultrasonic range module hc-sr04 -------------------------------------------*/
#define TRIGGER_PIN 16
#define ECHO_PIN 15
#define MAX_DISTANCE 100
int maximumRange = 100;  // Maximum range needed
int minimumRange = 1;    // Minimum range needed
long duration, distance; // Duration used to calculate distance
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);
/* ---------------------------------------------------------------------------*/

/* IR obstacle avoidance module ----------------------------------------------*/
const int IR_Pin = 17; //the ir obstacle sensor attached to pin 17
/* ---------------------------------------------------------------------------*/


/* ---------------------------------------------------------------------------
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //added delay to allow for time to connect via bluetooth
  delay(20000);

  //initialize HC06 Bluetooth module
  Serial1.begin(9600);
  Serial.println("Bluetooth initialization complete");
  Serial1.println("Bluetooth initialization complete");

  //start serial for debug
  Serial.begin(115200);
  Serial.println("Robot starts initialization");
    
  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  
  //initialize IR module
  pinMode(IR_Pin, INPUT); //set the avoidPin as INPUT
  Serial.println("IR obstacle avoidance initialization complete");
  Serial1.println("IR obstacle avoidance itialization complete");

  //sonar should already be initialized
  Serial.println("HC-SR04 sonar module initialization complete");
  Serial1.println("HC-SR04 sonar module initialization complete");

  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  Serial1.println("Servo service started");
  
  //initialize servos
  servo_attach();
  Serial.println("Servos initialized");
  Serial1.println("Servos initialized");

  //finish initialization
  stand();
  delay(3000);
  Serial.println("Robot initialization complete");
  Serial1.println("Robot initialization complete");
  delay(1000); 
}


/* ---------------------------------------------------------------------------
  - servo_attach function
   ---------------------------------------------------------------------------*/
void servo_attach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}


/* ---------------------------------------------------------------------------
  - servo_detach function
   ---------------------------------------------------------------------------*/
void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
  }
}

char mode;
char command;
/* ---------------------------------------------------------------------------
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
  sit();
  
  //read serial1
  while (Serial1.available()<1){
    delay(1000);
  }
  mode = ' ';
  command = ' ';
  if (Serial1.available()>0) {
    mode = Serial1.read();
  }

  //parse serial1 data and go to proper mode case
  switch (mode) {
    case '1':
      //mode 1 - demo mode
      Serial1.println("Mode 1: Demo Mode");
      while (mode=='1') {
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          Serial1.println("Standing...");
          stand();
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Stepping forward 5 steps...");
          step_forward(5);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          if (Serial1.available()>0) command = Serial1.read();
          step_back(5);
          command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Turning left 5 steps...");
          turn_left(5);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Turning right 5 steps...");
          turn_right(5);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Head up...");
          stand();
          head_up(30);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Head down...");
          stand();
          head_down(20);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Body left...");
          stand();
          body_left(20);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Body right...");
          stand();
          body_right(40);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Body dance...");
          stand();
          body_dance(10);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Waving hand 3 times...");
          stand();
          hand_wave(3);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);
          Serial1.println("Shaking hand 3 times...");
          hand_shake(3);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          delay(1500);  
          Serial1.println("Sitting down...");
          sit();
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
          Serial1.println("Waiting 10 seconds to continue...");
          delay(10000);
          if (Serial1.available()>0) command = Serial1.read();
          if (command == 'q') {
            Serial1.println("Exit Demo Mode - Mode 1");
            break;
          }
      }
      break;
      
    case '2':
      //mode 2 - remote control mode
      //Serial.println("Mode 2 set");
      Serial1.println("Mode 2 set");
      while (mode == '2') {
        command = ' ';
        if (Serial1.available()>0) command = Serial1.read();
        if (command == 'f') {
          Serial1.println("Stepping forward...");
          step_forward(1);
        }
        if (command == 'b') {
          Serial1.println("Stepping backward...");
          step_back(1);
        }
        if (command == 'l') {
          Serial1.println("Stepping left...");
          turn_left(1);
        }
        if (command == 'r') {
          Serial1.println("Stepping right...");
          turn_right(1);
        }
        if (command == 'u') {
          Serial1.println("Standing up...");
          stand();
        }
        if (command == 'd') {
          Serial1.println("Sitting down...");
          sit();
        }
        if (command == 'q') {
          Serial1.println("Exit Mode 2");
          stand();
          sit();
          break;
        }
      }
      break;
      
    case '3':
      //mode 3 - autonomous mode
      //Serial.println("Mode 3 set");
      Serial1.println("Mode 3 set");
      while (mode == '3') {
        if (Serial1.available()>0) command = Serial1.read();
        int obstacle = obstacle_detect();
        if (obstacle == 0) {
          //moving forward
          Serial1.println("Stepping forward (no obstacles)...");
          step_forward(1);
        }
        if (obstacle == 3) {
          //slight move left
          Serial1.println("Stepping left/forward (long range obstacle detected)...");
          turn_left(2);
          step_forward(1);
        }
        if (obstacle == 2) {
          //bigger move left
          Serial1.println("Stepping left (mid range obstacle detected)...");
          turn_left(3);
        }
        if (obstacle == 1) {
          //hard move left
          Serial1.println("Stepping left (short range obstacle detected)...");
          step_back(1);
          turn_left(5);
        }
        if (command == 'q') {
          Serial1.println("Exit Mode 3");
          break;
        }
      }
      break;
  }
}


/*
  - obstacle_detect function
  - returns 0=no obstacle, 1= <20cm, 2= 20-40cm, 3= 40-60cm
   ---------------------------------------------------------------------------*/
int obstacle_detect(void)
{
  int obstacle = 0;
  
  //read sonar module set obstacle to 1 < 20cm, 2 20cm-40cm, 3 40cm-60cm
  int distance = sonar.ping_median(5)/US_ROUNDTRIP_CM; 
  if (distance > 40 && distance < 60)  obstacle = 3;
  if (distance > 20 && distance <= 40) obstacle = 2;
  if (distance > 0 && distance <= 20)  obstacle = 1;

  //read IR module
  boolean IRStatus = digitalRead(IR_Pin); //read the value of IR module
  if (IRStatus == LOW) {
    obstacle = 1;
  }
  Serial1.print("Sonar = ");
  Serial1.print(distance);
  Serial1.print("cm  |  IR = ");
  if (IRStatus == LOW) Serial1.println("Triggered");
  else Serial1.println("Not Triggered");
  return obstacle;
}


/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}


/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


/* ---------------------------------------------------------------------------
  - added bt RegisHsu
   ---------------------------------------------------------------------------*/
/*
  - body turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}
/*
  - body turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}
/*
  - hand wave
  - blocking function
  - parameter number of hand waves
   ---------------------------------------------------------------------------*/
void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}
/*
  - hand shake
  - blocking function
  - parameter number of hand shakes
   ---------------------------------------------------------------------------*/
void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}
/*
  - head up
  - blocking function
  - parameter 
   ---------------------------------------------------------------------------*/
void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}
/*
  - head down
  - blocking function
  - parameter 
   ---------------------------------------------------------------------------*/
void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}
/*
  - body dance
  - blocking function
  - parameter 
   ---------------------------------------------------------------------------*/
void body_dance(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  //stand();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}
/* ---------------------------------------------------------------------------*/


/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}


/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}


/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}


/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}


/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}


/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}

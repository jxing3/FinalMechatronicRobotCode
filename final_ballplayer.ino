#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

//Sensor variables
const int SENSOR_PIN1 = 2;
const int SENSOR_PIN2 = 3;
const int SENSOR_PIN3 = 8;
const float c = 346.5;
const int power = 100;
const int pwm = power / 100.0 * 255;

const int startSwitch = 10;

const int int_gain = 3;

boolean rightflag = false;

boolean shootcheck;

//Pixy Object
Pixy pixy;

//Variable to make loop code run once
boolean runonce = true;

//Used in program
//int count;

//Servo Motor object
Servo arm;

//Motor angle parameters
int def = 90;//default position 80
int collect = 10;//collection position
int sht = 120;//shooting position

//the speed with which the arm moves: delay() in ms
int shootSpd = 3;
int collectSpd = 3;

//counts number of ball collected
int counter;


//Setup motors and other stuff
void setup() {

  //Attaches motor to Arduino
  arm.attach(9);
  //set arm to default position
  arm.write(def);
  //sets counter to 0
  counter = 0;

  //Setup output pins
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  pinMode(startSwitch, INPUT_PULLUP);

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  pixy.init();
  Serial.begin(9600);

  waitForLittleRobot();

  shootcheck = false;
}
//Loops!
void loop() {
 
  
  while (!shootcheck) { //if shootcheck false then capture balls
    //Rotate to find the correct ball
    rotateToFind();
    //Centers the ball in our view
    centerBall();
    //Goes towards the ball until it is a certain distance away;
    //Calls centerBall every second or so to make sure ball is centered as we move
    goToBallCam(); //first use camera to go towards ball
    goToBall();
    //move servomotor arm down
    capture();
    if (counter >= 3) {
      counter = 0;
      break;
    }
  }
  shootcheck = false;

  delay(500);
  
  //move towards the center of the arena
  goToCenterArena();
  //Rotate to find the goal
  rotateToFindGoal();
  //Center the middle of the goal in our sights
  centerGoal();
  //Goes to goal, centers every second or so
  goToGoalTopSensor();
  goToGoal();
  //Turns around to face the back sensor towards the goal
  turnTillGoal();
  //go back a little
  backALittle();
  // shoot the ball
  shoot();
  //move foward a little after shot
  forwardALittle();
  delay(2000);

}


//Function that executes the collecting motion
void capture() {
  if (arm.attached()) { //checks if motor is attached to Arduino
    //Lowers arm to collect ball
    for (int i = def; i >= collect; i--) {
      arm.write(i);
      delay(collectSpd);
    }
    delay(500);
    //Moves arm to default position after collection
    counter++; //increments counter variable
    for (int j = collect; j <= def; j++) {
      arm.write(j);
      delay(10);
    }
  } else { //If motor is not attached print error prompt
    Serial.print("Error: capture function. Not attached");
  }
}

//Function that executes the shooting motion
void shoot() {
  if (arm.attached()) {
    //Shoots ball
    for (int k = def; k >= 50; k--) {
      arm.write(k);
      delay(7);
    }
    for (int i = 50; i <= sht; i++) {
      arm.write(i);
      delay(shootSpd);
    }
    delay(2000);
    counter = 0; //resets counter variable
    //moves arm to default position
    for (int j = sht; j >= def; j--) {
      arm.write(j);
      delay(7);
    }
  } else {
    Serial.print("Error: shoot function. Not attached");
  }
}
//Returns height of the largest correct ball
int height() {
  int correctBall = findLargest();
  if (correctBall != -1) {
    return pixy.blocks[correctBall].height;
  }
  return -1;
}
//Returns top y position of the largest correct ball
int top() {
  //Finds largest correct ball
  int correctBall = findLargest();
  if (correctBall != -1) {
    return pixy.blocks[correctBall].y + (1 / 2) * pixy.blocks[correctBall].height;
  }
  return -1;
}
//Goes to the center of the arena
void goToCenterArena() {

  long tot = millis();
  long time = millis();
  while (true) {

    if (millis() > tot + 8000) { //(added condition)
      // forward(500);
      break;
    }
    int dist = PINGDistance(SENSOR_PIN2);
    int dist1 = PINGDistance(SENSOR_PIN3);
    if (dist == 0) { //dist1==0

      left(40000); //left if any distance reading messes up
      continue;
    }
    if (dist < 400 && dist1 > 100) { //added double condition
      backward(120);
      time = millis();
    } else if (dist1 < 400 && dist > 100) {
      forward(120);
      time = millis();
    } else {
      left(40000);

    }
  }
}

// waits for the robot with the front PING sensor until 5 cm away
void waitForLittleRobot() {
  int count;
  while (true) {

    int dist = PINGDistance(SENSOR_PIN1);

    if (dist < 50 && dist != 0) {
      count++;
    } else {
      count = 0;
    }

    if (count > 15) {
      delay(3000); //waits an extra 3 seconds
      return;
    }
  }
}

//Goes backward for 500 milliseconds
void backALittle() {

  backward(500);
}

// Goes forward for 500 milliseconds
void forwardALittle() {

  forward(500);
}


//Turns until the goal opening is in view of back sensor
void turnTillGoal() {

  int distCount = 0;
  int tot = millis();

  //first while condition breaks when sensor detects less than 47 cm
  while (true) {

    int dist = PINGDistance(SENSOR_PIN3);

    left(2600);
    if (dist < 470 && dist != 0) {
      break;
    }

  }
  // second condition breaks when distance is greater than 70 cm
  while (true) {
    int dist = PINGDistance(SENSOR_PIN3);

    left(2600);
    if (dist > 700) { //|| dist == 0
      return;
    }
  }
}

//Turns 180 degress
void turn180() {
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  delay(1200);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

//Uses top sensor to go to goal until it is 25 cm away
void goToGoalTopSensor() {
  int count;
  //Loop until close enough
  long time = millis();
  while (true) {
    if (millis() > time + 6000) {
      break;
    }
    //Get distance of goal
    int distance = PINGDistance(SENSOR_PIN2);

    if (distance == 0) {

      rightMilli(25);
    }
    //Not close enough, need to move forward
    if (distance > 250) { //140 100
      forward(20);
      //Every 40 counts center the goal again
      if (count % 40 == 0) {
        centerGoal();
      }
    }
    //Too close, move backwards
    else if (distance < 240) { //130  90
      backward(20);
      //Every 40 counts center the goal
      if (count % 40 == 0) {
        centerGoal();
      }
    }
    //Close enough, center twice and return
    else {
      centerGoal();
      delay(20);
      centerGoal();
      break;
    }
    //The number of times we have run this code
    count++;
  }
}

//Go towards the goal, centering the goal every second or so
void goToGoal() {
  int count;
  //Loop until close enough
  while (true) {
    //Get distance of goal
    int distance = PINGDistance(SENSOR_PIN1);
    //Not close enough, need to move forward
    if (distance > 120) { //140 100
      forward(20);
      //Every 40 counts center the goal again
      if (count % 40 == 0) {
        centerGoal();
      }
    }
    //Too close, move backwards
    else if (distance < 110) { //130  90
      backward(20);
      //Every 40 counts center the goal
      if (count % 40 == 0) {
        centerGoal();
      }
    }
    //Close enough, center twice and return
    else {
      centerGoal();
      delay(20);
      centerGoal();
      break;
    }
    //The number of times we have run this code
    count++;
  }
}
//Rotates around until goal is sighted
void rotateToFindGoal() {
  boolean t = true;

  while (t) {

    if (PINGDistance(SENSOR_PIN1) < 160) {
      backward(160);
    } else {
      int goal = findRightGoal();
      if (goal != -1) {
        t = false;
        break;
      }
      right(8000);
      delayMicroseconds(5000);
    }
  }
}
/*Returns array index of rightmost goal*/
int findRightGoal() {
  uint16_t num = pixy.getBlocks();
  int rightmost = -1;
  double curr_x = 0;
  //No goal in view
  if (num == 0) {
    return -1;
  }
  //Find goal of largest area of correct size
  for (int i = 0; i < num; i++) {
    if (pixy.blocks[i].signature == 3) {
      double new_x = pixy.blocks[i].x;
      if (new_x > curr_x) {
        curr_x =  new_x;
        rightmost = i;
      }
    }
  }

  return rightmost;
}

//Not sure if working
/*Returns array index of Leftmost goal*/
int findLeftGoal() {

  uint16_t num = pixy.getBlocks();
  int leftmost = 100000000;
  double curr_x = 0;

  //No goal in view
  if (num == 0) {
    return -1;
  }

  //Find goal of largest area of correct size
  for (int i = 0; i < num; i++) {

    if (pixy.blocks[i].signature == 3) {
      double new_x = pixy.blocks[i].x;
      if (new_x < curr_x) {
        curr_x =  new_x;
        leftmost = i;
      }
    }
  }

  // pixy.blocks[largestSize].print();
  return leftmost;
}


//Center goal in view

void centerGoal() {

  int iterations = 0;
  int integrator = 0;

  int low_threshold = 155;
  int high_threshold = 165;

  long tot = millis();


  while (true) {

    if (millis() > tot + 2000) { //(added condition)
      break;
    }


    int goal = findRightGoal();

    if (PINGDistance(SENSOR_PIN1) > 150) {
      low_threshold = 152;
      high_threshold = 168;

    } else {
      low_threshold = 156;
      high_threshold = 164;
    }

    if (abs(integrator) > 200) {
      if (integrator > 0) {
        integrator = 200;
      } else {
        integrator = -200;
      }
    }

    //Test to see if we found a correct ball
    if (goal != -1) {
      iterations = 0;
      if (pixy.blocks[goal].x < low_threshold) {
        int error = 160 - pixy.blocks[goal].x;
        integrator += error;
        Serial.println("turn left");


        if (error > 80) {
          error = 80;
        } 
        left(80 * error + int_gain * abs(integrator)); //250

      } else if (pixy.blocks[goal].x > high_threshold) {


        int error = pixy.blocks[goal].x - 160;

        Serial.println("turn right");
        integrator -= error;

        if (error > 80) {
          error = 80;
        } 
        right(80 * error + int_gain * abs(integrator));

      } else {
        Serial.println("Centered");
        return;


      }


    } else {


      //Didnt find ball
      iterations++;

      if (iterations > 10) {
        iterations = 0;
        backward(500);  //go backward for a little bit before refinding goal
        rotateToFindGoal();

      }

      Serial.println("NO GOAL");
    }
  }

}


//Goes forward until you are at a certain distance
//away from object detected by PING
void goToDistance(int ref_dist) {
  int count;
  while (true) {
    int dist = PINGDistance(SENSOR_PIN1);
    if (dist > ref_dist + 3) {

      int error = dist - ref_dist;
      Serial.println("turn left");

      if (error > 20) {
        error = 20;
      }

      forward(5 * error);

    } else if (dist < ref_dist - 3) {

      int error = ref_dist - dist;

      if (error > 20) {
        error = 20;
      }
      backward(5 * error);

    } else {
      break;
    }

    count++;
  }

}

//Go to Ball - depends on size of green in camera

void goToBallCam() {

  int count;

  while (true) {

    int ball = findLargest();
    forward(20);
    if (pixy.blocks[ball].width > 250) {
      break;
    }
    if (count % 40 == 0) {
      centerBall();
    }
    count++;
  }

}

//Go to the ball to a distance of 5.5 cm, centers every once in a while
void goToBall() {
  int count;

  while (true) {
    int dist = PINGDistance(SENSOR_PIN1);

    if (dist > 58) { //86

      int error = dist - 55; //85
      Serial.println("turn left");

      if (error > 20) {
        error = 20;
      }

      forward(.4 * error);

      if (count % 40 == 0) {
        centerBall();
      }
    } else if (dist < 52) { //84

      int error = 55 - dist; //85

      if (error > 20) {
        error = 20;
      }

      backward(.4 * error);

      if (count % 40 == 0) {
        centerBall();
      }
    } else {
      centerBall();
      delay(20);
      centerBall();
      break;
    }

    count++;
  }

}

//Rotates to find ball (doesn't back away from walls)
void rotateToFind2() {
  boolean t = true;
  long time = millis();

  while (t) {

    if (millis() > time + 4000) {
      backward(100);
    }
    int correctBall = findLargest();

    if (correctBall != -1) {
      t = false;
      break;
    }

    left(2500);
    delayMicroseconds(5000);
  }
}


//Rotates to find ball also backs away from walls
void rotateToFind() {
  boolean t = true;
  long time = millis();
  while (t) {

    if (millis() > time + 10000) {
      shootcheck = true;
    }

    if (PINGDistance(SENSOR_PIN1) < 160) {
      backward(160);

    } else {

      int correctBall = findLargest();

      if (correctBall != -1) {
        t = false;
        break;
      }

      left(2500);
      delayMicroseconds(5000);

    }
  }
}

//Center ball in view

void centerBall() {

  int iterations = 0;
  int integrator = 0;

  int low_threshold = 155;
  int high_threshold = 165;

  long tot = millis();

  while (true) {

    if (millis() > tot + 1000) { //breaks out if fails for 1 seconds
      break;
    }
    int correctBall = findLargest();

    if (PINGDistance(SENSOR_PIN1) > 300) {
      low_threshold = 150;
      high_threshold = 170;

    } else {
      low_threshold = 156;
      high_threshold = 164;
    }

    if (abs(integrator) > 200) { //200
      if (integrator > 0) {
        integrator = 200; //200
      } else {
        integrator = -200; //200
      }
    }

    //Test to see if we found a correct ball
    if (correctBall != -1) {
      iterations = 0;
      if (pixy.blocks[correctBall].x < low_threshold) {
        int error = 160 - pixy.blocks[correctBall].x;
        integrator += error;
        Serial.println("turn left");


        if (error > 80) {
          error = 80;
        } 
        left(80 * error + int_gain * abs(integrator)); //250 10

      } else if (pixy.blocks[correctBall].x > high_threshold) {

        int error = pixy.blocks[correctBall].x - 160;

        Serial.println("turn right");
        integrator -= error;

        if (error > 80) {
          error = 80;
        } 
        right(80 * error + int_gain * abs(integrator));

      } else {
        Serial.println("Centered");
        return;
      }
    } else {

      //Didnt find ball
      iterations++;

      if (iterations > 10) {
        iterations = 0;
        rotateToFind2(); //calls rotate to find 2 
      }

      Serial.println("NO BALLS");
    }
  }

}

/*Returns array index of largest ball */
int findLargest() {

  uint16_t num = pixy.getBlocks();
  int largestSize = -1;
  double maxArea = 0;

  //No Balls in view
  if (num == 0) {
    return -1;
  }

  //Find ball of largest area of correct size
  for (int i = 0; i < num; i++) {

    if (pixy.blocks[i].signature == 1) {
      double newArea = pixy.blocks[i].width * pixy.blocks[i].height;
      if (newArea > maxArea) {
        maxArea =  newArea;
        largestSize = i;
      }
    }
  }

  return largestSize;
}


/* Code to go forward for time milliseconds */
void forward(int time) {

  // digitalWrite(4, HIGH);
  analogWrite(4, pwm);
  digitalWrite(5, LOW);
  // digitalWrite(6, HIGH);
  analogWrite(6, pwm);
  digitalWrite(7, LOW);
  delay(time);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

/* Code to go back for time milliseconds */

void backward(int time) {
  digitalWrite(4, LOW);
  analogWrite(5, pwm);
  // digitalWrite(5, HIGH);
  //  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  analogWrite(7, pwm);
  delay(time);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

/* Code to turn left for time milliseconds */

void leftMilli(int time) {
  digitalWrite(4, LOW);
  //digitalWrite(5, HIGH);
  analogWrite(5, pwm);
  // digitalWrite(6, HIGH);
  analogWrite(6, pwm);
  digitalWrite(7, LOW);
  delay(time);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}
/* Code to turn left for time microseconds */

void left(int time) {
  digitalWrite(4, LOW);
  //digitalWrite(5, HIGH);
  analogWrite(5, pwm);
  // digitalWrite(6, HIGH);
  analogWrite(6, pwm);
  digitalWrite(7, LOW);
  delayMicroseconds(time);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}
/* Code to turn right for time milliseconds */
void rightMilli(int time) {
  analogWrite(4, pwm);
  // digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  analogWrite(7, pwm);
  //  digitalWrite(7, HIGH);
  delay(time);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

/*Code to turn right for time in microseconds */
void right(int time) {
  analogWrite(4, pwm);
  // digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  analogWrite(7, pwm);
  //  digitalWrite(7, HIGH);
  delayMicroseconds(time);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

// function to return distance from PING sensor
int PINGDistance(int SENSOR_PIN) {
  // set up sensor pin as output
  pinMode(SENSOR_PIN, OUTPUT);
  // send an initial 2 microsecond low pulse
  digitalWrite(SENSOR_PIN, LOW);
  delayMicroseconds(2);
  // send a 5 microsecond input trigger high pulse
  digitalWrite(SENSOR_PIN, HIGH);
  delayMicroseconds(5);
  // return signal to low
  digitalWrite(SENSOR_PIN, LOW);

  // change sensor to input mode
  pinMode(SENSOR_PIN, INPUT);
  // wait until signal is high to start reading pulse
  while (digitalRead(SENSOR_PIN) != HIGH) {}

  // record beginning of pulse
  unsigned long pulse_beginning = micros();
  unsigned long duration;
  // wait for a maximum of 19000 microseconds
  while (micros() <= pulse_beginning + 19000) {
    // if pulse has ended (signal is low again)
    if (digitalRead(SENSOR_PIN) == LOW) {
      // record duration, and break out of loop
      duration = micros() - pulse_beginning;
      break;
    }
  }

  // compute distance in mm from duration in us
  int distance = pow(10, -3) * c / 2.0 * duration;

  delay(8); //10 5
  // return computed distance
  return distance;
}

#include <Adafruit_AMG88xx.h>
#include <Servo.h>

// Include all needed packages needed for RF 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// stuff for IR camera
#include <Wire.h>
#define sensor_threshold 30
#define activated_pixels 8
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int threshold_counter = 0;
bool lastIR = false;
unsigned long lastIRframeTime = 0;
unsigned long frameRate = 100;

bool detectIR() {
  if (millis() - lastIRframeTime < frameRate) {
    return lastIR;
  }
  
  threshold_counter = 0;
  amg.readPixels(pixels);

  for(int i = 0; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++){
    if (pixels[i] > sensor_threshold) {
      threshold_counter++;
      if (threshold_counter >= activated_pixels) {
        break;
      }
    }
  }

  if (threshold_counter >= activated_pixels) {
    Serial.println("PUCK FOUND!");
    lastIR = true;
  }
  lastIRframeTime = millis();
  return lastIR;
}


// Address of the RF communication
const byte address[6] = "00018";
char text[2];

enum State {
  right, left, forward, backward, freeze
};

State state = freeze;
// RF... Things
RF24 radio(7, 8); // CE, CSN

int drive = A4;
int goalie = A5; // analog pin to read command

#define err 14
#define driveregion  1024/5
#define goalieregion 1024/4

/* motors */
#define M1_ENABLE 9
#define M2_ENABLE 10
#define M1_IN1 6
#define M1_IN2 5
#define M2_IN3 4
#define M2_IN4 3

int discretizeDrive() {
  int val = analogRead(drive);
  if (val < driveregion-err) {
    return 0;
  } else if (val < 2*driveregion-err) {
    return 1;
  } else if (val < 3*driveregion-err) {
    return 2;
  } else if (val < 4*driveregion-err) {
    return 3;
  } else if (val < 5*driveregion) {
    return 4;
  }
}
int discretizeGoalie() {
  int val = analogRead(goalie);
  if (val < goalieregion-err) {
    return 0;
  } else if (val < 2*goalieregion-err) {
    return 1;
  } else if (val < 3*goalieregion-err) {
    return 2;
  } else if (val < 4*goalieregion) {
    return 3;
  }
}

void transmitToGoalie(){
  radio.write(&text, sizeof(text));
}

/* drive functions - not sure if the directions are right but we will find out */

void turnLeft() {
//  analogWrite(M1_ENABLE, 255);
//  analogWrite(M2_ENABLE, 255);
  digitalWrite(M1_ENABLE, HIGH);
  digitalWrite(M2_ENABLE, HIGH);
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, HIGH);
  digitalWrite(M2_IN4, LOW);
}
void turnRight() {
//  analogWrite(M1_ENABLE, 255);
//  analogWrite(M2_ENABLE, 255);
  digitalWrite(M1_ENABLE, HIGH);
  digitalWrite(M2_ENABLE, HIGH);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, HIGH);
}
void freez() {
//  analogWrite(M1_ENABLE, 0);
//  analogWrite(M2_ENABLE, 0);
  
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, LOW);
  digitalWrite(M1_ENABLE, LOW);
  digitalWrite(M2_ENABLE, LOW);
}
void goForward() {
//  analogWrite(M1_ENABLE, 255);
//  analogWrite(M2_ENABLE, 255);
  digitalWrite(M1_ENABLE, HIGH);
  digitalWrite(M2_ENABLE, HIGH);
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, HIGH);
}
void goBackward() {
  //analogWrite(M1_ENABLE, 255);
  //analogWrite(M2_ENABLE, 255);
  digitalWrite(M1_ENABLE, HIGH);
  digitalWrite(M2_ENABLE, HIGH);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN3, HIGH);
  digitalWrite(M2_IN4, LOW);
}

void setup() {
  // Initialize RF communication
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  text[0] = 'a'; text[1] = '\0';

  /* motor stuff */
  pinMode(M1_ENABLE, OUTPUT);
  pinMode(M2_ENABLE, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT); 
  analogWrite(M1_ENABLE, 0);
  analogWrite(M2_ENABLE, 0);

  
  amg.begin();
  
  delay(300);
  Serial.begin(9600);
}

void loop() {
  int d = discretizeGoalie();
  switch (d) {
    case 0: // gLeft
      text[0] = 'L';
      break;
    case 1: // gRight
      text[0] = 'R';
      break;
    case 2: // gStop
      text[0] = 'S';
      break;
    case 3: // gPass
      text[0] = 'P';
      break;
  }

  Serial.println("Sending: " + String(text));
  transmitToGoalie();
  
  d = discretizeDrive();
  switch (d) {
    case left:
      Serial.println("left");
      turnLeft();
      break;
    case right:
      Serial.println("right");
      turnRight();
      break;
    case forward:
      Serial.println("4ward");
      goForward();
      break;
    case backward:
      Serial.println("backward");
      goBackward();
      break;
    case freeze:
      Serial.println("freeze");
      freez();
      break;
    default:
      Serial.println("none");
      break;
  }
}

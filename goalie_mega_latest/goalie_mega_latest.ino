 // includes
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>

// define directions
#define LEFT 0
#define RIGHT 1
#define SIDE LEFT

// comm enums
enum goalieCommand {
  gLeft='L', gRight='R', gStop='S', gPass='P'
};
enum moveCommand {
  mLeft, mRight, mFor, mBack, mStop
};

// RF Communication global vars
int values[10];
const byte address[6] = "00018";
RF24 radio(7,8); // CE, CSN

// motor defines
#define M1_ENABLE 6
#define M2_ENABLE 5
#define M1_IN1 22
#define M1_IN2 23
#define M2_IN3 24
#define M2_IN4 25

// puck passing functions
int enable = A4;
int in1 = A3;
int in2 = A2;

int lastCommand;
int curCommand;
goalieCommand command = gStop;

void goLeft() {
  curCommand = LEFT;
#if SIDE==LEFT
  forward();
  extendForward();
#else
  backward();
  extendBackward();
#endif
}

void goRight() {
  curCommand = RIGHT;
#if SIDE==LEFT
  backward();
  extendBackward();
#else
  forward();
  extendForward();
#endif
}

void goStop() {
  freeze();
}

void goPass() {
  if (lastCommand == LEFT) {
    goLeft();
  } else {
    goRight();
  }
}

// RF Functions
void rf_parse() {
  // Once the RF receiver gets the signal from the trasmitter
  if (radio.available()) {
    // Read in the values from the transmitter
    char text[100] = "";
    radio.read(&text, sizeof(text));
    Serial.println("(" + String(text) + ")");

    // Extract all the values from the received string
    // Stop at the first 10 values (Transmitter should send this correctly)
    // char* token = strtok(text, " ");
    // int i = 0;
    switch (text[0]) {
      case gLeft:
        command = gLeft;
        goLeft();
        break;
      case gRight:
        command = gRight;
        goRight();
        break;
      case gStop:
        command = gStop;
        //Serial.println("gstop");
        //goStop();
        break;
      case gPass:
        command = gPass;
        goPass();
        break;
    }
  }
}


void extendForward() {
  digitalWrite(enable, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void extendBackward() {
  digitalWrite(enable, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// movement functions
void forward() {
  analogWrite(M1_ENABLE, 255);
  analogWrite(M2_ENABLE, 255);
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, HIGH);
}
void backward() {
  analogWrite(M1_ENABLE, 255);
  analogWrite(M2_ENABLE, 255);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN3, HIGH);
  digitalWrite(M2_IN4, LOW);
}
void freeze() {
  analogWrite(M1_ENABLE, 0);
  analogWrite(M2_ENABLE, 0);
}

void setup() {
  /* motors */
  pinMode(M1_ENABLE, OUTPUT);
  pinMode(M2_ENABLE, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT); 
  analogWrite(M1_ENABLE, 0);
  analogWrite(M2_ENABLE, 0);
  
  // Initialize RF communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // serial monitor, TO REMOVE
  Serial.begin(9600);
}

void loop() {
  rf_parse(); // get latest codes from forward
  if (lastCommand == LEFT && curCommand == RIGHT) {
    goLeft();
  } else if (lastCommand == RIGHT && curCommand == LEFT) {
    goRight();
  }else if (command == gStop) {
    goStop();
  } else if (command == gLeft) {
    goLeft();
  } else if (command == gRight) {
    goRight();
  }
  delay(15);
  lastCommand = curCommand;
}  

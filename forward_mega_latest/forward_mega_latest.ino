// RF includes
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

// IMU includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <utility/imumaths.h>

// IR includes
#include <Adafruit_AMG88xx.h>

// IR setup
#define sensor_threshold 30
#define activated_pixels 8
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int threshold_counter;

// defines for the math
#define XMAX 292
#define YMAX 167
#define XMIN 40
#define YMIN 43
#define XMIDLINE (XMAX-XMIN)/2+XMIN
#define YMIDLINE (YMAX-YMIN)/2+YMIN
#define MIDPOINT XMIDLINE
#define THIRD ((YMAX-YMIN)/3 + YMIN)
#define THIRD2 (2*(YMAX-YMIN)/3 + YMIN)
#define LGOAL ((XMAX-XMIN)/4 + XMIN)-10
#define RGOAL (3*(XMAX-XMIN)/4 +XMIN)+10

#define LEFT 0
#define RIGHT 1

#define OURSIDE LEFT

#if OURSIDE==LEFT
#define GOALIE_TOP 80
#define GOALIE_BOTTOM 105
#define COLOR BLUE
#else
#define GOALIE_TOP 105
#define GOALIE_BOTTOM 80
#define COLOR GREEN
#endif

#define TGOAL GOALIE_TOP
#define BGOAL GOALIE_BOTTOM

// IMU defines
#define BNO055_SAMPLERATE_DELAY_MS (50)

// Shooting defines
#define LIMIT 10
#define STICK 11
#define SPEED 255

// sound defines
#define PIN_MICROPHONE A0   //pin reading microphone analog signal
#define DC_OFFSET 247       //constant offset to subtract from microphone to zero-center it
#define THRESHOLD 30        //Volume threshold at which we detect the siren. Currently low end, maybe want higher...
#define WINDOW 500          //amount of samples over which to take the Mean Absolute Value of the sound

long i = 0;             //count how many times loop has occured (for slowing down serial data printing)
float MAV = 0;          //Mean Absolute Value, a rough measure of how loud the sound is over the window
bool detected = false;  //keep track of if the siren is being detected
bool start_game = false;
bool first_start = true;

// enums
enum shootstate {
  arm, shoot, stay
};

enum goalieCommand {
  gLeft, gRight, gStop, gPass
};
enum moveCommand {
  mLeft, mRight, mFor, mBack, mStop
};

enum arena_pos {
  l_goalie_box,
  l_half,
  r_half,
  r_goalie_box
};

enum Region {
  top,
  center,
  bottom
};

enum item_type {
  o_goalie,
  o_forward,
  t_goalie,
  t_forward,
  puck
};

enum poss_state {
  our_goalie,
  our_forward,
  their_goalie,
  their_forward,
  unclaimed,
  muddy
};

enum forward_state { 
  ult_defense_t, // top
  ult_defense_b, // bottom
  has_puck,
  getting_puck
};

// typedefs
typedef struct _vector {
  double x;
  double y;
} fpVector;

typedef struct _item {
  fpVector pos;
  fpVector vel;
  item_type type;
  arena_pos arenaPos;
  Region region;
} fpItem;

// global vars
fpItem theirGoalie;
fpItem theirForward;
fpItem ourGoalie;
fpItem ourForward;
fpItem Puck;
poss_state Possession;

forward_state forwardState;
shootstate shoot_state = arm;

// RF Communication global vars
int values[10];
const byte address[6] = "00001";
RF24 radio(7,8); // CE, CSN

// uno comm global vars
int unoGoaliePin = 5; // must be pwm pin
int unoMovePin = 6;
int oRange4 = 256/4; // 4 possible out values.
int oRange5 = 256/5; // 5 possible out values

// imu global vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float ref_imu_x, ref_imu_y, imu_x, imu_y, curr_imu_x, curr_imu_y, imu_angle;
float ftp_x, ftp_y, ftp_angle, fp_theta;
int imu_quad = 1;
unsigned long lastSampleTime;

bool test_ir() {
  threshold_counter = 0;
  amg.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    if (pixels[i-1] > sensor_threshold) {
      threshold_counter++;
    } 
    if (threshold_counter >= activated_pixels) {
      break;
    }
  }

  if (threshold_counter >= activated_pixels) {
    return true;
  }
  return false;
}

// localization
void localize(fpItem& item){
  if (item.pos.x < LGOAL) {
    if (item.type == puck) {
      //Serial.println("puck in LGOAL. " + String(item.pos.x) + " < " + String(LGOAL));
    }
    item.arenaPos = l_goalie_box;
  } else if (item.pos.x < XMIDLINE) {
    if (item.type == puck) {
      //Serial.println("puck < XMIDLINE. " + String(item.pos.x) + " < " + String(XMIDLINE));
    }
    item.arenaPos = l_half;
  } else if (item.pos.x < RGOAL) {
    if (item.type == puck) {
      //Serial.println("puck < RGOAL. " + String(item.pos.x) + " < " + String(RGOAL));
    }
    item.arenaPos = r_half;
  } else {
    if (item.type == puck) {
      //Serial.println("puck in RGOAL. " + String(item.pos.x) + " > " + String(RGOAL));
    }
    item.arenaPos = r_goalie_box;
  }

  if (item.pos.y < THIRD) {
    item.region = top;
  } else if (item.pos.y < THIRD2) {
    item.region = center;
  } else {
    item.region = bottom;
  }
}

double distance(fpItem i1, fpItem i2) {
  return sqrt(pow(i1.pos.x - i2.pos.x, 2) + pow(i1.pos.y - i2.pos.y, 2));
}

bool nearby_puck(fpItem item) {
  if (distance(item, Puck) < 30.0) {
    return true;
  }
  return false;
}

void possession() {
  bool tGoalie = nearby_puck(theirGoalie);
  bool tFwd = nearby_puck(theirForward);
  bool oGoalie = nearby_puck(ourGoalie);
  bool oFwd = nearby_puck(ourForward);

  if (oFwd && abs(fp_theta) < 10) {
    Possession = our_forward;
  } else if (tGoalie && !tFwd && !oGoalie && !oFwd) {
    Possession = their_goalie;
  } else if (!tGoalie && tFwd && !oGoalie && !oFwd) {
    Possession = their_forward;
  } else if (!tGoalie && !tFwd && oGoalie && !oFwd) {
    Possession = our_goalie;
  } else if (!tGoalie && !tFwd && !oGoalie && oFwd) {
    if (test_ir()) {
      Possession = our_forward;
    }
  } else if (!tGoalie && !tFwd && !oGoalie && !oFwd) {
    Possession = unclaimed;
  } else {
    Possession = muddy;
  }
}

bool puckNearOurGoal() {
  if (Puck.arenaPos == l_goalie_box && OURSIDE == LEFT) {
    return true;
  } else if (Puck.arenaPos == r_goalie_box && OURSIDE == RIGHT) {
    return true;
  }
  return false;
}

bool puckNearTheirGoal() {
  if (Puck.arenaPos == l_goalie_box && OURSIDE == RIGHT) {
    return true;
  } else if (Puck.arenaPos == r_goalie_box && OURSIDE == LEFT) {
    return true;
  }
  return false;
}

int setFwdState() {
  possession();
  if (puckNearOurGoal()) {
    //Serial.print("puck near our goal. ");
    if (Possession != our_goalie && Possession != our_forward) {
      // ultimate defense if we don't have possession
      return ultDef();
    } else {
      // otherwise get puck and yeet it
      Serial.println("we dont have possession");
      return attack();
    }
  } else if (puckNearTheirGoal()) {
    Serial.print("puck near their goal. ");
    if (Possession != our_forward) {
      //Serial.println("we dont have possession");
      return attack(); // get puck and attack
    } else {
      Serial.println("we have possession");
      return shootfn();
    }
  } else if (Puck.arenaPos == l_half || Puck.arenaPos == r_half) {
    // get puck and attack
    Serial.println("Puck in midrange");
    return attack();
  }
}

// coordinate with the goalie to block shots
int ultDef() {
  fpVector pos;
  int a = 0;
  if (Puck.pos.y > YMIDLINE) { // puck is bottom
    forwardState = ult_defense_t;
    pos = pickUDPos(true);
    // a = ult_defense_t;
  } else { // puck is top
    forwardState = ult_defense_b;
    pos = pickUDPos(false);
    // a = ult_defense_b;
  }
  Serial.println("UltDef goal pos: (" + String(pos.x) + ", " + String(pos.y) + ")");
  Serial.println("Cur pos: (" + String(ourForward.pos.x) + ", " + String(ourForward.pos.y) + ")");
  a = moveToPos(pos);
  
  return a;
}

int moveToPos(fpVector pos) {
  int err = 5; // not sure what this should be
  Serial.println("Goal pos: " + String(pos.x) + " " + String(pos.y));
  Serial.println("Cur pos: " + String(ourForward.pos.x) + " " + String(ourForward.pos.y));
  if (abs(ourForward.pos.x-pos.x) < 5 && abs(ourForward.pos.y-pos.y) < 5){
    Serial.println("Position obtained");
    return mStop; 
  }
  // turn to point
  int i = getDirection(pos);
  if (i < 0) {
    // left
    Serial.println("left");
    return mLeft;
  } if (i > 0) {
    // right
    Serial.println("right");
    return mRight;
  } else {
    // fwd
    Serial.println("4ward");
    return mFor;
  }
}

fpVector pickUDPos(bool top) {
  fpVector goalPosition;
  if (OURSIDE == LEFT) {
    goalPosition.x = XMIN;
  } else {
    goalPosition.x = XMAX;
  }
  if (top) {
    goalPosition.y = BGOAL;
  } else {
    goalPosition.y = TGOAL;
  }
  return goalPosition;
}

// take the puck and run with it
int attack() {
  Serial.println("attack");
  if (Possession == unclaimed || Possession == muddy) {
    // go to puck
    return moveToPos(Puck.pos);
    // this next line may be good for catching up to the puck
    // may want to vel multiply by a constant
    // depending on puck speed, etc.
    // but puck.vel will be scaled by speed so? should be good
    //moveToPos(Puck.pos + Puck.vel);
    
    Serial.println("attack goal pos: (" + String(Puck.pos.x) + ", " + String(Puck.pos.y) + ")");
  } else if (Possession == our_forward) {
    // go to the other goal
    fpVector v;
    if (theirGoalie.pos.y > YMIDLINE) { // goalie at bottom
      v.y = BGOAL+5;
    } else {
      v.y = TGOAL-5;
    }
    if (OURSIDE == LEFT) {
      v.x = RGOAL+10;
    } else {
      v.x = LGOAL-10;
    }
    
    Serial.println("attack goal pos: (" + String(v.x) + ", " + String(v.y) + ")");
    return moveToPos(v);
  }
}

// TODO
// Probably requires more pro strats than we have currently
int shootfn() {
  Serial.println("shoot");
  // do the shooting mechanism
  shoot_state = shoot;
  return moveToPos(Puck.pos);
}

// RF functions
void rfsetup() {
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void rfread() {
  if (radio.available()) {
    // Read in the values from the transmitter
    char text[150] = "";
    radio.read(&text, sizeof(text));
    Serial.println("text: " + String(text));

    // Extract all the values from the received string
    // Stop at the first 10 values (Transmitter should send this correctly)
    char* token = strtok(text, " ");
    int i = 0;
    
    while (token != NULL && i < 10) {
      int a = atoi(token);
      if (a != -1) {
        values[i] = a;
      }
      token = strtok(NULL, " ");
      i++;
    }
  } else {
    Serial.println("noradio");
  }
}

void getGoalieCoordsFromPixy() {
  fpVector v1;
  v1.x = values[6];
  v1.y = values[7];
  fpVector v2;
  v2.x = values[8];
  v2.y = values[9];

#if OURSIDE==LEFT
  if (v1.x <= MIDPOINT) { // v1 is left side
    ourGoalie.pos = v1;
    theirGoalie.pos = v2;
  } else {
    theirGoalie.pos = v1;
    ourGoalie.pos = v2;
  }
#else
  if (v1.x >= MIDPOINT) { // v1 is right side
    ourGoalie.pos = v1;
    theirGoalie.pos = v2;
  } else {
    theirGoalie.pos = v1;
    ourGoalie.pos = v2;
  }
#endif
}

void setPositions() {
  Puck.pos.x = values[0];
  Puck.pos.y = values[1];
#if COLOR==GREEN
  ourForward.pos.x = values[4];
  ourForward.pos.y = values[5];
  theirForward.pos.x = values[2];
  theirForward.pos.y = values[3];
#else
  ourForward.pos.x = values[2];
  ourForward.pos.y = values[3];
  theirForward.pos.x = values[4];
  theirForward.pos.y = values[5];
#endif

  getGoalieCoordsFromPixy();
}

// communicate with Uno
void sendMoveType(int a) {// how does the forward need to move
  // left, right, forward, back, stop
  analogWrite(unoMovePin, a*oRange5);
}

void sendCommandToGoalie(int a) {
  // one of Left, Right, Stop, Pass
  analogWrite(unoGoaliePin, a*oRange4);
}

bool isGoalieInPos(int pos) {
  if (abs(ourGoalie.pos.y - pos) < 5) {
    return true;
  } else {
    return false;
  }
}

int getGoalieCommand() {
  if (Possession == our_goalie) {
    return gPass;
  } else if (Puck.region == top) {
    if (!isGoalieInPos(GOALIE_TOP)) {
#if OURSIDE==LEFT
    Serial.println("gLeft");
    return gLeft;
#else
    Serial.println("gRight");
    return gRight;
#endif
    } else {
      Serial.println("gStop");
      return gStop;
    }
  } else if (Puck.region == bottom) {
    if (!isGoalieInPos(GOALIE_BOTTOM)) {
#if OURSIDE==LEFT
    Serial.println("gRight");
    return gRight;
#else
    Serial.println("puck bottom, gLeft");
    return gLeft;
#endif
    } else {
      Serial.println("gStop");
      return gStop;
    }
  } else {
    Serial.println("gStop");
    return gStop;
  }
}

// shooting mechanism functions
  void shoot_stick() {
  analogWrite(STICK, SPEED);    //rotate to release stick
  if(!digitalRead(LIMIT)) {   //wait till rotation has released stick
    shoot_state = arm; // immediately begin re-arming
    analogWrite(STICK, 0); 
  }//stop the motor
}

void arm_stick() {
  analogWrite(STICK, SPEED);  //pull the stick back
  if (digitalRead(LIMIT)) {
    shoot_state = stay;       //change state if limit is pressed
    analogWrite(STICK, 0);    //stop the motor
  }
}

// IR functions


// Rotation Functions
void IMUsetup(){
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  delay(1000);
  
  // ---- Get IMU reference data ----
  sensors_event_t event;
  bno.getEvent(&event);
  ref_imu_y = event.magnetic.y;
  ref_imu_x = event.magnetic.x;
  lastSampleTime = millis();
}

double getSmallestAngle(fpVector f) {
  if (millis() - lastSampleTime < BNO055_SAMPLERATE_DELAY_MS) {
    Serial.println("returning");
    return fp_theta;
  }
  
  lastSampleTime = millis();
  
  double forward_x = ourForward.pos.x;
  double forward_y = ourForward.pos.y;
  
  // ---- Get current IMU data ----
  sensors_event_t event;
  bno.getEvent(&event);
  
  curr_imu_x = event.magnetic.x;

  double puck_x = f.x;
  double puck_y = f.y;

  // ---- Get cartesian vector ----
  ftp_x = forward_x - puck_x;
  ftp_y = forward_y - puck_y;
  
  // ---- Get polar angle from cartesian vector ----
  ftp_angle = atan2(ftp_y, ftp_x)*180 / 3.14;
  
  // ---- Get IMU angle based on current and reference ----
  imu_x = curr_imu_x - ref_imu_x;
  while (imu_x < 0) {
    imu_x += 360;
  }
  imu_angle = imu_x;
  
  // ---- Find FPtheta by adding polar angle and IMU angle ----
  double theta = ftp_angle - imu_angle;
  
  if (abs(theta) > 180) {
    if (ftp_angle > 180) {
      ftp_angle = ftp_angle - 360;
    }
    if (imu_angle > 180) {
      imu_angle = imu_angle - 360;
    }
    theta = ftp_angle - imu_angle;
  }
  
  Serial.print("theta: ");
  Serial.println(theta, 4);
  fp_theta = theta;
  
  return theta;
}

int getDirection(fpVector pos) {
  double theta = getSmallestAngle(pos);
  if (abs(theta) < 5) {
    // go straight
    return 0;
  } else if (theta < 0) {
    // go left
    return -1;
  } else {
    // go right
    return 1;
  }
}


// setup & loop
void setup() {
  // put your setup code here, to run once:
  rfsetup();
  IMUsetup();
  Serial.begin(9600);
  theirGoalie.type = t_goalie;
  ourGoalie.type = o_goalie;
  theirForward.type = t_forward;
  ourForward.type = o_forward;
  Puck.type = puck;
  amg.begin();

  pinMode(PIN_MICROPHONE, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!start_game) {
    detected = MAV > THRESHOLD;                       //save whether or not the current volume is above a threshold (indicating a loud sound such as the siren)
    int sample = analogRead(PIN_MICROPHONE);          //get the current sample from the microphone
    MAV += (abs(sample - DC_OFFSET) - MAV) / WINDOW;  //update the MAV rolling average with the new sample
    
    if (MAV > THRESHOLD && !detected && first_start) {
      Serial.println("The game has started.");
      start_game = true;             //print only when transitioning from not detected to detected
      first_start = false;
    }
  } else {
    rfread();
    setPositions();
    localize(ourGoalie);
    localize(ourForward);
    localize(Puck);
    //Serial.println("Puck: " + String(Puck.region));
    localize(theirGoalie);
    localize(theirForward);
    //updateIMU();
    int fcmd = setFwdState();
    sendMoveType(fcmd);
    
    int gcmd; // goalie command
    gcmd = getGoalieCommand();
    //Serial.println("Cmd: " + String(gcmd));
    sendCommandToGoalie(gcmd);
    
    switch (shoot_state) {
      case arm:
        arm_stick();
        break;
      case shoot:
        shoot_stick();
        break;
      case stay:
      default:
        analogWrite(STICK, 0);
        break;
    }
    delay(200);
    Serial.println("Possession: " + Possession);
    if (Possession == their_goalie) {
      Serial.println("Possession = their_goalie");
    } else if (Possession == their_forward) {
      Serial.println("Possession = their_forward");
    } else if (Possession == our_goalie) {
      Serial.println("Possession = our_goalie");
    } else if (Possession == our_forward) {
      Serial.println("Possession = our_forward");
    } else if (Possession == unclaimed) {
      Serial.println("Possession = unclaimed");
    } else {
      Serial.println("Possession = muddy");
    }
  }
}

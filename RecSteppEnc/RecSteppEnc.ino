// Recording two stepper motor movement using rotary encoder without SD Card
// Limited recorded movement
// Record every pulses given to stepper motor


#include <AccelStepper.h>

// ===================== STEPPERS =====================
#define STEP1_PUL 8
#define STEP1_DIR 9
#define STEP1_ENA 10

#define STEP2_PUL 11
#define STEP2_DIR 12
#define STEP2_ENA 13

AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PUL, STEP1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PUL, STEP2_DIR);

// Manual & playback parameters
const float MANUAL_MAX_SPEED   = 5000;
const float MANUAL_ACCEL       = 300;
const float PLAYBACK_MAX_SPEED = 7000;
const float PLAYBACK_ACCEL     = 600;


// ===================== ENCODERS =====================
#define ENC1_CLK 2
#define ENC1_DT  3
#define ENC2_CLK 18
#define ENC2_DT  19

volatile int8_t enc1Delta = 0;
volatile int8_t enc2Delta = 0;

long encoder1Pulses = 0;             // total pulse count
float encoder1Degrees = 0;
long encoder2Pulses = 0;             // total pulse count
float encoder2Degrees = 0;

const long pulses1PerRev = 400;
const float deg1PerPulse = 360.0 / pulses1PerRev;
const long pulses2PerRev = 20;
const float deg2PerPulse = 360.0 / pulses2PerRev;

// ===================== BUTTONS =====================
#define BTN_REC  28
#define BTN_PLAY 29

const unsigned long DEBOUNCE_MS = 20;
bool stableRec = false, stablePlay = false;
bool lastStableRec = false, lastStablePlay = false;
bool rawRec, rawPlay;
bool lastRawRec = false, lastRawPlay = false;
unsigned long lastChangeRec = 0, lastChangePlay = 0;



// ===================== RECORDING =====================
const int MAX_MOVES = 800;

struct Move {
  long d1;
  long d2;
};

Move moves[MAX_MOVES];
int moveCount = 0;

bool isRecording = false;
bool isPlaying   = false;
bool needGoToZero = false;
int playIndex = 0;

long lastPos1 = 0;
long lastPos2 = 0;

// ===================== ENCODER ISR =====================
void encoderISR1() {
  static uint8_t old = 3;
  static int8_t val = 0;

  old <<= 2;
  if (digitalRead(ENC1_CLK)) old |= 0x02;
  if (digitalRead(ENC1_DT))  old |= 0x01;

  static const int8_t tbl[] = {
     0,-1, 1, 0,
     1, 0, 0,-1,
    -1, 0, 0, 1,
     0, 1,-1, 0
  };

  val += tbl[old & 0x0F];
  if (val > 3)  { enc1Delta++; val = 0; }
  if (val < -3) { enc1Delta--; val = 0; }
  old &= 0x0F;
}

void encoderISR2() {
  static uint8_t old = 3;
  static int8_t val = 0;

  old <<= 2;
  if (digitalRead(ENC2_CLK)) old |= 0x02;
  if (digitalRead(ENC2_DT))  old |= 0x01;

  static const int8_t tbl[] = {
     0,-1, 1, 0,
     1, 0, 0,-1,
    -1, 0, 0, 1,
     0, 1,-1, 0
  };

  val += tbl[old & 0x0F];
  if (val > 3)  { enc2Delta++; val = 0; }
  if (val < -3) { enc2Delta--; val = 0; }
  old &= 0x0F;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  pinMode(STEP1_ENA, OUTPUT);
  pinMode(STEP2_ENA, OUTPUT);
  digitalWrite(STEP1_ENA, HIGH);
  digitalWrite(STEP2_ENA, HIGH);

  stepper1.setMaxSpeed(MANUAL_MAX_SPEED);
  stepper1.setAcceleration(MANUAL_ACCEL);
  stepper2.setMaxSpeed(MANUAL_MAX_SPEED);
  stepper2.setAcceleration(MANUAL_ACCEL);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  pinMode(ENC1_CLK, INPUT_PULLUP);
  pinMode(ENC1_DT,  INPUT_PULLUP);
  pinMode(ENC2_CLK, INPUT_PULLUP);
  pinMode(ENC2_DT,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_CLK), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_DT),  encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CLK), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_DT),  encoderISR2, CHANGE);

  pinMode(BTN_REC,  INPUT_PULLUP);
  pinMode(BTN_PLAY, INPUT_PULLUP);

  Serial.println("System ready.");
}

// ===================== LOOP =====================
void loop() {
  readButtons();
  handleControlButtons();
  readEncoders();

  stepper1.setSpeed(500);
  stepper1.runSpeedToPosition();  
  stepper2.setSpeed(500);
  stepper2.runSpeedToPosition();
  

  handleRecording();
  handlePlayback();
}

// ===================== BUTTONS =====================
void readButtons() {
  unsigned long now = millis();

  rawRec = digitalRead(BTN_REC) == LOW;
  if (rawRec != lastRawRec) { lastChangeRec = now; lastRawRec = rawRec; }
  else if (now - lastChangeRec > DEBOUNCE_MS) stableRec = rawRec;

  rawPlay = digitalRead(BTN_PLAY) == LOW;
  if (rawPlay != lastRawPlay) { lastChangePlay = now; lastRawPlay = rawPlay; }
  else if (now - lastChangePlay > DEBOUNCE_MS) stablePlay = rawPlay;
}

void handleControlButtons() {
  if (stableRec && !lastStableRec) toggleRecording();
  if (stablePlay && !lastStablePlay) startPlayback();
  lastStableRec  = stableRec;
  lastStablePlay = stablePlay;
}

// ===================== ENCODER MOTION =====================
void readEncoders() {
  int8_t d1, d2;

  noInterrupts();
  d1 = enc1Delta;
  d2 = enc2Delta;
  enc1Delta = 0;
  enc2Delta = 0;
  interrupts();

  if (isPlaying) return;

  if (d1){
    encoder1Pulses += d1;
    stepper1.moveTo(encoder1Pulses * 2);
  } 
  if (d2){
    encoder2Pulses += d2;
    stepper2.moveTo(encoder2Pulses * 40);
  }
}

// ===================== RECORDING =====================
void toggleRecording() {
  if (isPlaying) return;

  isRecording = !isRecording;

  if (isRecording) {
    moveCount = 0;
    lastPos1 = 0;
    lastPos2 = 0;
    Serial.println("Recording started.");
  } else {
    Serial.print("Recording stopped. Moves: ");
    Serial.println(moveCount);
  }
}

void handleRecording() {
  if (!isRecording || isPlaying) return;

  long p1 = stepper1.currentPosition();
  long p2 = stepper2.currentPosition();

  long d1 = p1 - lastPos1;
  long d2 = p2 - lastPos2;

  if ((d1 || d2) && moveCount < MAX_MOVES) {
    moves[moveCount++] = { d1, d2 };
    lastPos1 = p1;
    lastPos2 = p2;
  }
}

// ===================== PLAYBACK =====================
void startPlayback() {
  if (isRecording || isPlaying || moveCount == 0) return;

  isPlaying = true;
  needGoToZero = true;
  playIndex = 0;

  stepper1.setMaxSpeed(PLAYBACK_MAX_SPEED);
  stepper1.setAcceleration(PLAYBACK_ACCEL);
  stepper2.setMaxSpeed(PLAYBACK_MAX_SPEED);
  stepper2.setAcceleration(PLAYBACK_ACCEL);

  stepper1.moveTo(0);
  stepper2.moveTo(0);

  Serial.println("Playback started.");
}

void handlePlayback() {
  if (!isPlaying) return;

  if (needGoToZero) {
    stepper1.run();
    stepper2.run();
    if (stepper1.distanceToGo() == 0 &&
        stepper2.distanceToGo() == 0) {
      needGoToZero = false;
    }
    return;
  }

  if (stepper1.distanceToGo() == 0 &&
      stepper2.distanceToGo() == 0) {

    if (playIndex >= moveCount) {
      isPlaying = false;
      stepper1.setMaxSpeed(MANUAL_MAX_SPEED);
      stepper1.setAcceleration(MANUAL_ACCEL);
      stepper2.setMaxSpeed(MANUAL_MAX_SPEED);
      stepper2.setAcceleration(MANUAL_ACCEL);
      Serial.println("Playback finished.");
      return;
    }

    stepper1.move(moves[playIndex].d1);
    stepper2.move(moves[playIndex].d2);
    playIndex++;
  }
}

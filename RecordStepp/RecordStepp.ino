#include <AccelStepper.h>

const int stepPul = 9;
const int stepDir = 8;
const int stepEna = 7;

AccelStepper stepper(1, stepPul, stepDir);

// Manual settings
const float MANUAL_MAX_SPEED = 5000.0;
const float MANUAL_ACCEL     = 50.0;
const float MANUAL_SPEED     = 1000.0; // used by runSpeed()

// Playback (faster) settings
const float PLAYBACK_MAX_SPEED = 5000.0;
const float PLAYBACK_ACCEL     = 400.0;

const long STEPS_PER_REV = 1600;

// Buttons
const int btn1 = 30; // CW
const int btn2 = 31; // CCW
const int btnRec  = 34; // Record toggle
const int btnPlay = 35; // Playback

const unsigned long DEBOUNCE_MS = 20UL;

// button states
bool rawB1=false, rawB2=false, rawRec=false, rawPlay=false;
bool stableB1=false, stableB2=false, stableRec=false, stablePlay=false;
bool lastRawB1=false, lastRawB2=false, lastRawRec=false, lastRawPlay=false;
bool lastStableRec=false, lastStablePlay=false;
unsigned long lastChangeB1=0, lastChangeB2=0, lastChangeRec=0, lastChangePlay=0;

int dirState = 0; // -1 ccw, 0 stop, 1 cw
int prevDirState = 0;

// recording storage
const int MAX_MOVES = 500;
long moves[MAX_MOVES];
int moveCount = 0;

bool isRecording = false;
long recordStartPos = 0;

// playback state
bool isPlaying = false;
bool needGoToZero = false; // true while we move physically to logical 0 before replay
int playIndex = 0;

float currMaxSpeed = MANUAL_MAX_SPEED;
float currAccel = MANUAL_ACCEL;

void setup() {
  // Serial kept for debug; remove if truly headless
  Serial.begin(115200);

  pinMode(stepEna, OUTPUT);
  stepper.setEnablePin(stepEna);
  stepper.disableOutputs();

  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btnRec, INPUT_PULLUP);
  pinMode(btnPlay, INPUT_PULLUP);

  stepper.setMaxSpeed(MANUAL_MAX_SPEED);
  stepper.setAcceleration(MANUAL_ACCEL);

  currMaxSpeed = MANUAL_MAX_SPEED;
  currAccel = MANUAL_ACCEL;

  Serial.println("Ready.");
}

void loop() {
  readButtons();
  handleControlButtons(); // REC / PLAY
  handleAction(); // manual movement based on B1/B2

  // normal manual stepping
  runStepper();

  // recording capture (only while not playing)
  if (!isPlaying && isRecording) {
    if (prevDirState == 0 && dirState != 0) {
      recordStartPos = stepper.currentPosition();
    } else if (prevDirState != 0 && dirState == 0) {
      long delta = stepper.currentPosition() - recordStartPos;
      storeMove(delta);
    } else if (prevDirState != 0 && dirState != 0 && prevDirState != dirState) {
      long delta = stepper.currentPosition() - recordStartPos;
      storeMove(delta);
      recordStartPos = stepper.currentPosition();
    }
  }
  prevDirState = dirState;

  // Playback (non-blocking)
  if (isPlaying) {
    if (needGoToZero) {
      // move physically to logical 0 first
      if (stepper.distanceToGo() != 0) {
        stepper.run();
      } else {
        // reached zero, next: start playing segments
        needGoToZero = false;
        // start first segment
        playIndex = 0;
        if (playIndex < moveCount) {
          long target = stepper.currentPosition() + moves[playIndex];
          stepper.moveTo(target);
        }
      }
    } else {
      // normal segment playback
      if (stepper.distanceToGo() != 0) {
        stepper.run();
      } else {
        // finished a segment -> next
        playIndex++;
        if (playIndex < moveCount) {
          long target = stepper.currentPosition() + moves[playIndex];
          stepper.moveTo(target);
        } else {
          // all done
          isPlaying = false;
          stepper.disableOutputs();
          // restore manual settings
          stepper.setMaxSpeed(currMaxSpeed);
          stepper.setAcceleration(currAccel);
          Serial.println("Playback finished.");
        }
      }
    }
  }
}

// ---------------- Button handling ----------------

void readButtons() {
  unsigned long now = millis();

  // B1
  rawB1 = (digitalRead(btn1) == LOW);
  if (rawB1 != lastRawB1){
    lastChangeB1 = now;
    lastRawB1 = rawB1;
    }
  else if ((now - lastChangeB1) >= DEBOUNCE_MS){
    stableB1 = rawB1; 
    stepper.enableOutputs();
    }

  // B2
  rawB2 = (digitalRead(btn2) == LOW);
  if (rawB2 != lastRawB2){
    lastChangeB2 = now;
    lastRawB2 = rawB2;
    }
  else if ((now - lastChangeB2) >= DEBOUNCE_MS){
    stableB2 = rawB2;
    stepper.enableOutputs();
  }

  // REC
  rawRec = (digitalRead(btnRec) == LOW);
  if (rawRec != lastRawRec){
    lastChangeRec = now;
    lastRawRec = rawRec;
    }
  else if ((now - lastChangeRec) >= DEBOUNCE_MS){
    stableRec = rawRec;
  }

  // PLAY
  rawPlay = (digitalRead(btnPlay) == LOW);
  if (rawPlay != lastRawPlay){
    lastChangePlay = now;
    lastRawPlay = rawPlay;
    }
  else if ((now - lastChangePlay) >= DEBOUNCE_MS){
    stablePlay = rawPlay;
  }
}

void handleControlButtons() {
  // detect pressed edges
  if (stableRec && !lastStableRec) toggleRecording();
  if (stablePlay && !lastStablePlay) startPlayback();

  lastStableRec = stableRec;
  lastStablePlay = stablePlay;
}

// ---------------- Recording / Playback ----------------

void toggleRecording() {
  if (isPlaying) return; // ignore while playing

  if (!isRecording) {
    // Start recording: clear old, reset logical position to 0
    isRecording = true;
    moveCount = 0;
    stepper.setCurrentPosition(0); // LOGICAL zero set here
    prevDirState = dirState;
    if (dirState != 0) recordStartPos = stepper.currentPosition();
    Serial.println("Recording STARTED. Logical position reset to 0.");
  } else {
    // Stop recording; capture last segment if moving
    if (dirState != 0) {
      long delta = stepper.currentPosition() - recordStartPos;
      storeMove(delta);
    }
    isRecording = false;
    Serial.print("Recording STOPPED. Segments: ");
    Serial.println(moveCount);
  }
}

void startPlayback() {
  if (isRecording) return;
  if (isPlaying) return;
  if (moveCount == 0) {
    Serial.println("No recorded moves.");
    return;
  }

  // Save manual config
  currMaxSpeed = MANUAL_MAX_SPEED;
  currAccel = MANUAL_ACCEL;

  // Set faster playback params
  stepper.setMaxSpeed(PLAYBACK_MAX_SPEED);
  stepper.setAcceleration(PLAYBACK_ACCEL);

  // Begin playback flow: first move to logical 0
  isPlaying = true;
  needGoToZero = true;
  playIndex = 0;

  // Disable manual movement
  dirState = 0;

  stepper.enableOutputs();
  stepper.moveTo(0); // physically go to logical zero where setCurrentPosition(0) was called
  Serial.println("Playback: moving to logical 0 then will replay.");
}

void storeMove(long deltaSteps) {
  if (!isRecording) return;
  if (deltaSteps == 0) return;
  if (moveCount < MAX_MOVES) {
    moves[moveCount++] = deltaSteps;
    Serial.print("Recorded seg #"); Serial.print(moveCount);
    float deg = (deltaSteps * 360) / STEPS_PER_REV;
    Serial.print(": "); Serial.print(deg, 2); Serial.println(" deg");
  } else {
    Serial.println("Buffer full. Stopping recording.");
    isRecording = false;
  }
}

// ---------------- Motor control ----------------

void handleAction() {
  if (isPlaying) { dirState = 0; return; } // ignore manual during playback

  if (stableB1 && !stableB2) dirState = 1;
  else if (!stableB1 && stableB2) dirState = -1;
  else dirState = 0;
}

void runStepper() {
  if (isPlaying) return; // playback handles stepping
  if (dirState == 1) {
    stepper.setSpeed(MANUAL_SPEED);
    stepper.runSpeed();
  } else if (dirState == -1) {
    stepper.setSpeed(-MANUAL_SPEED);
    stepper.runSpeed();
  } else {
    stepper.disableOutputs();
  }
}

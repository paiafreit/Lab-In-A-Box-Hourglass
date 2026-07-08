#include "Arduino.h"
#include "LedControl.h"
#include "Delay.h"

#define  MATRIX_A  1
#define MATRIX_B  0

// Most analog joystick modules are centered around ~512 with a
// full swing of roughly 0-1023, rather than the accelerometer's
// 260/330/400-ish range. These thresholds are now relative to a
// center value with a dead zone, which is the natural way to
// read a joystick (it self-centers when released).
#define JOY_CENTER     512
#define JOY_DEADZONE   150   // +/- around center treated as "centered"

// Matrix
#define PIN_DATAIN 5
#define PIN_CLK 4
#define PIN_LOAD 6

// Joystick analog outputs (replace accelerometer X/Y) temporary until accellerometer is shipped
#define PIN_X A1
#define PIN_Y A2

// Buzzer (not yet arrived so no ability to test, placeholder)

#define PIN_BUZZER 9


#define PIN_RANDOM_SEED A0

// This takes into account how the matrixes are mounted
#define ROTATION_OFFSET 90

#define DELAY_FRAME 100

#define DEBUG_OUTPUT 1

#define MODE_HOURGLASS 0

byte delayHours = 0;
byte delayMinutes = 1;
int mode = MODE_HOURGLASS;
int gravity;
LedControl lc = LedControl(PIN_DATAIN, PIN_CLK, PIN_LOAD, 2);
NonBlockDelay d;
int resetCounter = 0;
bool alarmWentOff = false;


//  Per-matrix orientation correction
// ------------------------------------------------------------
//  lc.setRotation() applies ONE rotation to the whole LedControl
//  instance, so it can't express "matrix 0 is mounted differently
//  to matrix 1". If your two physical matrices are NOT mounted
//  with the same orientation relative to each other, set each
//  matrix's correction below to match how you actually wired it.
//
//  For each matrix address, set:
//   - a rotation in degrees (0, 90, 180, or 270), applied first
//   - then optionally flip horizontally and/or vertically
//
//  Example: if MATRIX_B is soldered in physically upside-down
//  relative to MATRIX_A, set its rotation to 180.
//  If MATRIX_B's ribbon/wires come out of a different edge than
//  MATRIX_A's (a 90-degree mounting difference), set its rotation
//  to 90 or 270, whichever matches what you see on the matrix.

struct MatrixOrientation {
  int rotation;         // 0, 90, 180, or 270
  bool flipHorizontal;  // applied after rotation
  bool flipVertical;    // applied after rotation
};

// Index 0 = matrix at address 0, Index 1 = matrix at address 1.
// Defaults below assume both matrices are mounted identically
// (matching the original hardware build) - i.e. no correction.
// Edit these two lines to match your own wiring/mounting:
MatrixOrientation matrixOrientation[2] = {
  /* addr 0 */ { 180,   false, false },
  /* addr 1 */ { 90,   false, false }
};

/**
 * Applies the configured correction for a given matrix address to
 * a logical (x,y) coordinate, returning the physical coordinate
 * that should actually be sent to/read from the LedControl driver.
 *
 * All particle-physics logic in this sketch (canGoLeft, moveParticle,
 * fill, etc.) works entirely in LOGICAL coordinates and never calls
 * lc.setXY/lc.getXY/lc.getRawXY/lc.invertRawXY directly - instead it
 * goes through correctXY() first, so per-matrix orientation only
 * needs to be defined in one place.
 */
coord correctXY(int addr, coord xy) {
  MatrixOrientation o = matrixOrientation[addr];

  switch (o.rotation) {
    case 90:  xy = lc.rotate90(xy);  break;
    case 180: xy = lc.rotate180(xy); break;
    case 270: xy = lc.rotate270(xy); break;
    default: break; // 0 - no rotation
  }
  if (o.flipHorizontal) {
    xy = lc.flipHorizontally(xy);
  }
  if (o.flipVertical) {
    xy = lc.flipVertically(xy);
  }
  return xy;
}
coord correctXY(int addr, int x, int y) {
  coord xy;
  xy.x = x;
  xy.y = y;
  return correctXY(addr, xy);
}


/**
 * Get delay between particle drops (in seconds)
 */
long getDelayDrop() {
  return delayMinutes + delayHours * 60;
}


#if DEBUG_OUTPUT
void printmatrix() {
  Serial.println(" 0123-4567 ");
  for (int y = 0; y<8; y++) {
    if (y == 4) {
      Serial.println("|----|----|");
    }
    Serial.print(y);
    for (int x = 0; x<8; x++) {
      if (x == 4) {
        Serial.print("|");
      }
      Serial.print(lc.getXY(0, correctXY(0, x, y)) ? "X" :" ");
    }
    Serial.println("|");
  }
  Serial.println("-----------");
}
#endif


coord getDown(int x, int y) {
  coord xy;
  xy.x = x-1;
  xy.y = y+1;
  return xy;
}
coord getLeft(int x, int y) {
  coord xy;
  xy.x = x-1;
  xy.y = y;
  return xy;
}
coord getRight(int x, int y) {
  coord xy;
  xy.x = x;
  xy.y = y+1;
  return xy;
}

bool canGoLeft(int addr, int x, int y) {
  if (x == 0) return false;
  return !lc.getXY(addr, correctXY(addr, getLeft(x, y)));
}
bool canGoRight(int addr, int x, int y) {
  if (y == 7) return false;
  return !lc.getXY(addr, correctXY(addr, getRight(x, y)));
}
bool canGoDown(int addr, int x, int y) {
  if (y == 7) return false;
  if (x == 0) return false;
  if (!canGoLeft(addr, x, y)) return false;
  if (!canGoRight(addr, x, y)) return false;
  return !lc.getXY(addr, correctXY(addr, getDown(x, y)));
}

void goDown(int addr, int x, int y) {
  lc.setXY(addr, correctXY(addr, x, y), false);
  lc.setXY(addr, correctXY(addr, getDown(x,y)), true);
}
void goLeft(int addr, int x, int y) {
  lc.setXY(addr, correctXY(addr, x, y), false);
  lc.setXY(addr, correctXY(addr, getLeft(x,y)), true);
}
void goRight(int addr, int x, int y) {
  lc.setXY(addr, correctXY(addr, x, y), false);
  lc.setXY(addr, correctXY(addr, getRight(x,y)), true);
}

int countParticles(int addr) {
  int c = 0;
  for (byte y=0; y<8; y++) {
    for (byte x=0; x<8; x++) {
      if (lc.getXY(addr, correctXY(addr, x, y))) {
        c++;
      }
    }
  }
  return c;
}

bool moveParticle(int addr, int x, int y) {
  if (!lc.getXY(addr, correctXY(addr, x, y))) {
    return false;
  }

  bool can_GoLeft = canGoLeft(addr, x, y);
  bool can_GoRight = canGoRight(addr, x, y);

  if (!can_GoLeft && !can_GoRight) {
    return false;
  }

  bool can_GoDown = canGoDown(addr, x, y);

  if (can_GoDown) {
    goDown(addr, x, y);
  } else if (can_GoLeft && !can_GoRight) {
    goLeft(addr, x, y);
  } else if (can_GoRight && !can_GoLeft) {
    goRight(addr, x, y);
  } else if (random(2) == 1) {
    goLeft(addr, x, y);
  } else {
    goRight(addr, x, y);
  }
  return true;
}

void fill(int addr, int maxcount) {
  int n = 8;
  byte x,y;
  int count = 0;
  for (byte slice = 0; slice < 2*n-1; ++slice) {
    byte z = slice<n ? 0 : slice-n + 1;
    for (byte j = z; j <= slice-z; ++j) {
      y = 7-j;
      x = (slice-j);
      lc.setXY(addr, correctXY(addr, x, y), (++count <= maxcount));
    }
  }
}

/**
 * Reads the joystick and maps it to a "gravity" direction, the same
 * way the accelerometer's tilt used to. Centered joystick = hold
 * last orientation (dead zone), pushed fully in a direction = that
 * orientation.
 */
int getGravity() {
  int x = analogRead(PIN_X);
  int y = analogRead(PIN_Y);

  if (y < JOY_CENTER - JOY_DEADZONE) { return 0;   }
  if (x > JOY_CENTER + JOY_DEADZONE) { return 90;  }
  if (y > JOY_CENTER + JOY_DEADZONE) { return 180; }
  if (x < JOY_CENTER - JOY_DEADZONE) { return 270; }

  return gravity; // joystick centered/dead zone: keep last orientation
}

int getTopMatrix() {
  return (getGravity() == 90) ? MATRIX_A : MATRIX_B;
}
int getBottomMatrix() {
  return (getGravity() != 90) ? MATRIX_A : MATRIX_B;
}

void resetTime() {
  for (byte i=0; i<2; i++) {
    lc.clearDisplay(i);
  }
  fill(getTopMatrix(), 60);
  d.Delay(getDelayDrop() * 1000);
}

bool updateMatrix() {
  int n = 8;
  bool somethingMoved = false;
  byte x,y;
  bool direction;
  for (byte slice = 0; slice < 2*n-1; ++slice) {
    direction = (random(2) == 1);
    byte z = slice<n ? 0 : slice-n + 1;
    for (byte j = z; j <= slice-z; ++j) {
      y = direction ? (7-j) : (7-(slice-j));
      x = direction ? (slice-j) : j;
      if (moveParticle(MATRIX_B, x, y)) {
        somethingMoved = true;
      };
      if (moveParticle(MATRIX_A, x, y)) {
        somethingMoved = true;
      }
    }
  }
  return somethingMoved;
}

boolean dropParticle() {
  if (d.Timeout()) {
    d.Delay(getDelayDrop() * 1000);
    if (gravity == 0 || gravity == 180) {
      coord cornerA = correctXY(MATRIX_A, 0, 0);
      coord cornerB = correctXY(MATRIX_B, 7, 7);
      if ((lc.getRawXY(MATRIX_A, cornerA.x, cornerA.y) && !lc.getRawXY(MATRIX_B, cornerB.x, cornerB.y)) ||
          (!lc.getRawXY(MATRIX_A, cornerA.x, cornerA.y) && lc.getRawXY(MATRIX_B, cornerB.x, cornerB.y))
      ) {
        lc.invertRawXY(MATRIX_A, cornerA.x, cornerA.y);
        lc.invertRawXY(MATRIX_B, cornerB.x, cornerB.y);
        tone(PIN_BUZZER, 440, 10);
        return true;
      }
    }
  }
  return false;
}

void alarm() {
  for (int i=0; i<5; i++) {
    tone(PIN_BUZZER, 440, 200);
    delay(1000);
  }
}

/**
 * Setup
 */
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(PIN_RANDOM_SEED));

  for (byte i=0; i<2; i++) {
    lc.shutdown(i,false);
    lc.setIntensity(i,1);
  }

  gravity = getGravity(); // prime initial orientation before first resetTime()
  resetTime();
}

/**
 * Main loop
 */
void loop() {
  delay(DELAY_FRAME);

  gravity = getGravity();
  lc.setRotation((ROTATION_OFFSET + gravity) % 360);

  bool moved = updateMatrix();
  bool dropped = dropParticle();

  if (!moved && !dropped && !alarmWentOff && (countParticles(getTopMatrix()) == 0)) {
    alarmWentOff = true;
    alarm();
  }
  if (dropped) {
    alarmWentOff = false;
  }
}

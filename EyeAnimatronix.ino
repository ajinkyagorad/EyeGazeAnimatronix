#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define DEADZONE_EYE 25
#define DEADZONE_NECK 20
#define Kp 0.03
#define NECK_DELAY 1200

#define CH_LR      0
#define CH_UD      1
#define CH_TL      2
#define CH_TR      4
#define CH_BASEX   6
#define CH_BASEY   7


// Angle limits
struct ServoLimit { int minA, maxA; };
ServoLimit limits[] = {
  {40,140}, // 0 dummy
  {40,140}, // 1 LR
  {40,140}, // 2 UD
  {90,170}, // 3 TL
  {90,170}, // 4 BL
  {90,10},  // 5 TR reversed
  {90,10},  // 6 BR reversed
  {10,170}, // 7 BaseX
  {40,140}  // 8 BaseY
};

// Servo targets
float LR_target=90, UD_target=90;
float TL_target=90, TR_target=90;
float BX_target=90, BY_target=90;
float BaseX=90, BaseY=90;

unsigned long lastUpdate=0;
unsigned long neckTimer=0;
bool neckFlag=false;

// ======================== FUNCTIONS =========================
int angleToPulse(float angle){
  float pulse = (angle * 500.0f / 90.0f) + 1500.0f; // 0°→1500µs, ±90°→500–2500
  int tick = (int)(pulse * SERVO_FREQ * 4096 / 1000000); 
  return constrain(tick, 102, 512); 
}

void writeServo(uint8_t ch, float angle){
  angle = constrain(angle, limits[ch].minA, limits[ch].maxA);
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

void calibrate(){
  writeServo(CH_LR,90); writeServo(CH_UD,90);
  writeServo(CH_TL,90); writeServo(CH_TR,90);
  writeServo(CH_BASEX,90); writeServo(CH_BASEY,90);
}

void blink(){
  writeServo(CH_TL,90);
  writeServo(CH_TR,90);
  delay(60);
}

void lidSync(){
  float ud_progress = (UD_target - limits[CH_UD].minA) /
                      (float)(limits[CH_UD].maxA - limits[CH_UD].minA);
  TL_target = limits[CH_TL].maxA - ((limits[CH_TL].maxA - limits[CH_TL].minA)*(0.5*(1-ud_progress))) -10;
  TR_target = limits[CH_TR].maxA + ((limits[CH_TR].minA - limits[CH_TR].maxA)*(0.5*(1-ud_progress))) +10;
  writeServo(CH_TL, TL_target);
  writeServo(CH_TR, TR_target);
}

void moveTarget(float &target, int ch, int error){
  if (abs(error) <= DEADZONE_EYE) return;
  if (error > 0) error -= DEADZONE_EYE;
  else error += DEADZONE_EYE;
  int step = (int)(Kp * error);
  if (step == 0) step = (error > 0)?1:-1;
  target += step;
  target = constrain(target, limits[ch].minA, limits[ch].maxA);
  writeServo(ch, target);
}

void neckTarget(float LR, float UD){
  BX_target = LR;
  BY_target = 90 - ((90 - UD)*0.6);
}

void neckSmoothMove(float speed_deg_s=60){
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;
  float step = speed_deg_s * dt;

  float dx = BX_target - BaseX;
  if (abs(dx) <= step) BaseX = BX_target;
  else BaseX += (dx > 0 ? step : -step);

  float dy = BY_target - BaseY;
  if (abs(dy) <= step) BaseY = BY_target;
  else BaseY += (dy > 0 ? step : -step);

  writeServo(CH_BASEX, BaseX);
  writeServo(CH_BASEY, BaseY);
}

// ======================== SETUP =========================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  calibrate();
  lastUpdate = millis();
}

// ======================== LOOP =========================
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    int comma = line.indexOf(',');
    if (comma > 0) {
      int errX = line.substring(0, comma).toInt();
      int errY = line.substring(comma+1).toInt();

      moveTarget(LR_target, CH_LR, -errX);
      moveTarget(UD_target, CH_UD,  errY);

      if (random(0,60) < 1) {
        //blink();
        //lidSync();
      }

      lidSync();

      if (abs(UD_target-90)>=DEADZONE_NECK || abs(LR_target-90)>=DEADZONE_NECK){
        if (!neckFlag){
          neckTimer = millis();
          neckFlag = true;
        }
        if (neckFlag && (millis()-neckTimer)>=NECK_DELAY){
          neckTarget(LR_target, UD_target);
          neckFlag=false;
        }
      }
      neckSmoothMove();
    }
  }
}

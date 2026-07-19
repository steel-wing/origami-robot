#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <string.h>

// constants
const float pi = 3.141592653;

// MOSFET
const int mosfetPin = 13;

// hinge variables
const int NUM_HINGES = 7;
float hingeAngles[NUM_HINGES];

// servo variables
const int NUM_SERVOS = 13;
Servo servos[NUM_SERVOS];
float servoAngles[NUM_SERVOS];

//                                 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
const int servoHinges[NUM_SERVOS] = {0, 2, 1, 3, 0, 1, 2, 5, 6, 4, 5, 4, 6};
const int servoPins[NUM_SERVOS] = {18, 23, 19, 22, 21, 2, 12, 14, 26, 25, 4, 16, 17};
const int servoType[NUM_SERVOS] = {1, 1, 2, 1, 1, 2, 1, 1, 1, 2, 1, 2, 1};

// point struct
typedef struct
{
  float x;
  float y;
  float z;
} Point3D;

// gait state struct
typedef struct
{
  Point3D p1, p2, p3, p4;
  int top;
  int bottom;
  float current_phase;
  int divisions;
  float scale;
  int current_step;
  int max_cycles;
  int ramp_steps;
  bool is_active;
} GaitState;

// gait variables
GaitState currentGait;
unsigned long lastGaitUpdate = 0;

// prototypes
Point3D eval_gait_curve(GaitState *gait, float phase);
void update_gait(GaitState *gait, float targetAngles[]);
float getTheta2(float theta1);
float getServoAngle(int i, float hingeAngles[]);
float quad_bezier(float t, float p0, float p1, float p2);

// math
float getTheta2(float theta1)
{
  return 2.0f * atanf(sqrtf(2.0f) * tanf(theta1 / 2.0f));
}

float getServoAngle(int i, float hingeAngles[])
{
  // restrict max extension due to hinge preloading
  float servoAngle = min(180.0f * hingeAngles[servoHinges[i]] / pi, 170.0f);

  // a few of the motors are installed "backwards" so we have to flip their orientation
  float angle = servoType[i] == 2 ? 180.0f - servoAngle : servoAngle;

  return angle;  
}

float quad_bezier(float t, float p0, float p1, float p2)
{
  float u = 1.0f - t;
  return (u * u * p0) + (2.0f * u * t * p1) + (t * t * p2);
}

Point3D eval_gait_curve(GaitState *gait, float phase)
{
  Point3D pts[4] = {gait->p1, gait->p2, gait->p3, gait->p4};
  Point3D mid[4];

  int max_steps = gait->max_cycles * 4 * gait->divisions;

  if (gait->current_step <= gait->ramp_steps)
  {
    gait->scale = (float)gait->current_step / (float)gait->ramp_steps;
  }
  else if (gait->current_step >= max_steps)
  {
    gait->scale = 0.0f;
    gait->is_active = false;
  }
  else if (gait->current_step >= (max_steps - gait->ramp_steps))
  {
    int steps_remaining = max(max_steps - gait->current_step, 0);
    gait->scale = (float)(steps_remaining) / (float)gait->ramp_steps;
  }
  else
  {
    gait->scale = 1.0f;
  }

  for (int i = 0; i < 4; i++)
  {
    int next = (i + 1) % 4;
    mid[i].x = (pts[i].x + pts[next].x) / 2.0f;
    mid[i].y = (pts[i].y + pts[next].y) / 2.0f;
    mid[i].z = (pts[i].z + pts[next].z) / 2.0f;
  }

  float wrapped_phase = fmodf(phase, 4.0f);
  int segment = (int)wrapped_phase;
  float local_t = wrapped_phase - segment;
  int next_mid = (segment + 1) % 4;
  int ctrl_pt = (segment + 1) % 4;

  Point3D result;
  result.x = gait->scale * quad_bezier(local_t, mid[segment].x, pts[ctrl_pt].x, mid[next_mid].x);
  result.y = gait->scale * quad_bezier(local_t, mid[segment].y, pts[ctrl_pt].y, mid[next_mid].y);
  result.z = gait->scale * quad_bezier(local_t, mid[segment].z, pts[ctrl_pt].z, mid[next_mid].z);
  return result;
}

void update_gait(GaitState *gait, float targetAngles[])
{
  if (!gait->is_active)
    return;

  Point3D targets = eval_gait_curve(gait, gait->current_phase);
  float tx = targets.x;
  float ty = targets.y;
  float tz = targets.z;

  if (gait->top == 4)
  {
    targetAngles[2] = tx;
    targetAngles[1] = getTheta2(tx);
    targetAngles[0] = getTheta2(tx);
  }
  else if (gait->top == 3)
  {
    targetAngles[2] = tx;
    targetAngles[1] = 0;
    targetAngles[0] = 0;
  }
  else if (gait->top == 2)
  {
    targetAngles[2] = 0;
    targetAngles[1] = tx;
    targetAngles[0] = 0;
  }
  else if (gait->top == 1)
  {
    targetAngles[2] = 0;
    targetAngles[1] = 0;
    targetAngles[0] = tx;
  }

  targetAngles[3] = ty;

  if (gait->bottom == 4)
  {
    targetAngles[6] = tz;
    targetAngles[4] = getTheta2(tz);
    targetAngles[5] = getTheta2(tz);
  }
  else if (gait->bottom == 3)
  {
    targetAngles[6] = tz;
    targetAngles[4] = 0;
    targetAngles[5] = 0;
  }
  else if (gait->bottom == 2)
  {
    targetAngles[6] = 0;
    targetAngles[4] = tz;
    targetAngles[5] = 0;
  }
  else if (gait->bottom == 1)
  {
    targetAngles[6] = 0;
    targetAngles[4] = 0;
    targetAngles[5] = tz;
  }

  gait->current_step++;
  gait->current_phase += 1 / (float)gait->divisions;
  if (gait->current_phase > 4)
  {
    gait->current_phase -= 4;
  }
}

void init_gait()
{
  currentGait = (GaitState){
      // .p1 = {0.56, 1.89, 0.8},        // crawl diagonal (3,1) center hinge up
      // .p2 = {2.97, 0.1, 2.93},
      // .p3 = {0.25, 0.22, 2.85},
      // .p4 = {0.25, 0.8, 0.71},
      // .top = 3,
      // .bottom = 1,
      .p1 = {0.2, 0.2, 2.2},        // gallop forwards (4,4) center hinge up
      .p2 = {2.2, 0.9, 0.2},
      .p3 = {1.8, 2.2, 0.2},
      .p4 = {0.2, 0.2, 0.2},
      .top = 4,
      .bottom = 4,
      // .p1 = {0.4, 0.4, 0.5},    // walker/rotator (1,1) center hinge down
      // .p2 = {0.3, 2.5, 2.8},
      // .p3 = {2.5, 2.5, 0.5},
      // .p4 = {1.4, 0.5, 0.5},
      // .top = 1,
      // .bottom = 1,
      // .p1 = {0.22, 0.28, 0.84},    // crab rotate plus (1,2) center hinge up
      // .p2 = {2.91, 0.15, 0.2},
      // .p3 = {1.94, 2.05, 0.2},
      // .p4 = {0.29, 2.15, 2.99},
      // .top = 1,
      // .bottom = 2,
      .current_phase = 0.0,
      .divisions = 100,
      .scale = 0.0,
      .current_step = 0,
      .max_cycles = 8,
      .ramp_steps = 400, // if you make this divisions * 4, it'll ramp up for 1 whole cycle
      .is_active = true};
}

// SETUP
void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(mosfetPin, OUTPUT);
  digitalWrite(mosfetPin, LOW);
  Serial.println("Power on...");

  // TURN ON MOSFET
  digitalWrite(mosfetPin, HIGH);

  // set all servos to 6 degrees
  Serial.println("Moving Servos to 6 (hold closed)");
  for (int i = 0; i < NUM_HINGES; i++)
  {
    hingeAngles[i] = 6.0 * pi/180.0;
  }

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    servos[i].setPeriodHertz(50);
    // special treatment for the Futaba
    if (i == 3) {
        servos[i].attach(servoPins[i], 900, 2100);
    } else {
        servos[i].attach(servoPins[i], 500, 2500);
    }

    servoAngles[i] = getServoAngle(i, hingeAngles);
    servos[i].write(servoAngles[i]);
    Serial.print(i + 1);
    Serial.print(" ");
    Serial.print(servoHinges[i]);
    Serial.print(" ");
    Serial.print(hingeAngles[servoHinges[i]]);
    Serial.print(" ");
    Serial.println(servoAngles[i]);
  }

  // 1 second of delay
  delay(1000);

  init_gait();
  Serial.println("Initialization complete.");
}


int count = 0;

// LOOP (fixed + MOSFET OFF at end)
void loop()
{
  unsigned long now = millis();

  if (now - lastGaitUpdate >= 20)
  {
    lastGaitUpdate = now;

    if (currentGait.is_active)
    {
      update_gait(&currentGait, hingeAngles);
      count += 1;
      for (int i = 0; i < NUM_SERVOS; i++)
      {
        servoAngles[i] = getServoAngle(i, hingeAngles);
        servos[i].write(servoAngles[i]);

        // check to see that 1 and 5 aline
        if (count % 10 == 0) {
          Serial.print(i + 1);
          Serial.print(" ");
          Serial.print(servoHinges[i]);
          Serial.print(" ");
          Serial.print(hingeAngles[servoHinges[i]]);
          Serial.print(" ");
          Serial.println(servoAngles[i]);
        }
      }
      if (count % 10 == 0) {
        Serial.println(currentGait.scale);
        Serial.println("");
      }
    }
    else
    {
      delay(1000);
      Serial.println("Moving Servos to 10");
      // set all servos to 10 degrees, in case of disassembly.
      for (int i = 0; i < NUM_HINGES; i++)
      {
        hingeAngles[i] = 10.0 * pi/180.0;
      }

      for (int i = 0; i < NUM_SERVOS; i++)
      {
        servoAngles[i] = getServoAngle(i, hingeAngles);
        servos[i].write(servoAngles[i]);
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.print(servoHinges[i]);
        Serial.print(" ");
        Serial.print(hingeAngles[servoHinges[i]] + 1  );
        Serial.print(" ");
        Serial.println(servoAngles[i]);
      }

      Serial.println("Powering down.");
      delay(1000);
      // TURN OFF MOSFET (same as second code)
      digitalWrite(mosfetPin, LOW);

      while (true)
      {
        Serial.println("...sleeping...");
        delay(20000);
      }
    }
  }
}

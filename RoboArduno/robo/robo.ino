#include <Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include <math.h>

#define RIGHT0 11
#define RIGHT1 12
#define LEFT0 3
#define LEFT1 4
#define SERVO0 7
#define SERVO1 8
#define ERROR 13
#define MAX_SPEED 255
#define MIN_SPEED 0

#define JOYSTICK_LEFT "LEFT"
#define JOYSTICK_RIGHT "RIGHT"
#define TOLORANCE_ARC_MOTO M_PI / 8

#define TOLORANCE_ARC_CAMERA M_PI / 4
#define HOST_REQUEST "HEY!!!"
#define PERIPHERAL "BODY"
Servo servo0;
Servo servo1;

struct RoboCommandPayload
{
  String joystick;
  float x;
  float y;
  float r;
};

struct MotoDirection
{
  float forward;
  float backward;
  float left;
  float right;
};

void setup()
{
  Serial.begin(9600); // Match the baud rate
  Serial.setTimeout(2);

  servo0.attach(SERVO0);
  servo1.attach(SERVO1);

  pinMode(RIGHT0, OUTPUT);
  pinMode(RIGHT1, OUTPUT);
  pinMode(LEFT0, OUTPUT);
  pinMode(LEFT1, OUTPUT);
}

String buffer = "";
bool receiving = false;

void loop()
{
  if (Serial.available())
  {
    char ch = Serial.read();

    if ('<' == ch)
    {
      receiving = true;
      buffer = "";
    }
    else if ('>' == ch)
    {
      receiving = false;
      handleBuffer(buffer);
    }
    else if (receiving)
    {
      if ('<' != ch && '>' != ch)
        buffer += ch;
    }
  }
  delay(1);
}

void handleBuffer(String jsonBuffer)
{
  
  if (jsonBuffer == HOST_REQUEST)
  {
    Serial.write(PERIPHERAL);
    return;
  }

    DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, jsonBuffer);
  if (error)
  {
    handleError();
    Serial.println(error.f_str());
    return;
  }
  else
  {
    handleJsonCommand(doc);
  }
}

void handleJsonCommand(DynamicJsonDocument doc)
{
  RoboCommandPayload payload;
  payload.joystick = doc["joystick"].as<String>();
  payload.x = doc["x"].as<float>();
  payload.y = doc["y"].as<float>();
  payload.r = doc["r"].as<float>();



  if (payload.joystick == JOYSTICK_LEFT)
  {
    if (payload.x == 0.0 && payload.y == 0.0)
    {
      digitalWrite(LEFT0, LOW);
      digitalWrite(LEFT1, LOW);
      digitalWrite(RIGHT0, LOW);
      digitalWrite(RIGHT1, LOW);
    }
    else
    {
      MotoDirection directions = handlePayload(payload, TOLORANCE_ARC_MOTO);
      move(directions.forward, directions.backward, directions.left, directions.right);
    }
  }
  else if (payload.joystick == JOYSTICK_RIGHT)
  {
    if (payload.x == 0.0 && payload.y == 0.0)
    {
    }
    else
    {
      MotoDirection directions = handlePayload(payload, TOLORANCE_ARC_CAMERA);

      // triggerServo(servo0, angles);
      // triggerServo(servo1, angles);

      // move(directions.forward, directions.backward, directions.left, directions.right);
    }
  }
}

void triggerServo(Servo servo, float angle)
{
  servo.write(angle);
}

void move(int forwardSpeed, int backwardSpeed, int leftSpeed, int rightSpeed)
{
  Serial.println("move");
  Serial.println(forwardSpeed);
  Serial.println(backwardSpeed);
  Serial.println(leftSpeed);
  Serial.println(rightSpeed);

  if (0 == forwardSpeed && 0 == backwardSpeed)
  {
    forwardLeft(rightSpeed);
    forwardRight(leftSpeed);
  }
  else
  {
    if (0 != forwardSpeed)
    {
      forwardLeft(forwardSpeed - leftSpeed);
      forwardRight(forwardSpeed - rightSpeed);
    }

    if (0 != backwardSpeed)
    {
      backwardLeft(backwardSpeed - leftSpeed);
      backwardRight(backwardSpeed - rightSpeed);
    }
  }
}

void forwardLeft(int speed)
{
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(LEFT0, HIGH);
  analogWrite(LEFT1, MAX_SPEED - speed);
}

void forwardRight(int speed)
{
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(RIGHT0, HIGH);
  analogWrite(RIGHT1, MAX_SPEED - speed);
}

void backwardLeft(int speed)
{
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(LEFT0, LOW);
  analogWrite(LEFT1, speed);
}

void backwardRight(int speed)
{
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(RIGHT0, LOW);
  analogWrite(RIGHT1, speed);
}

void handleError()
{
  digitalWrite(ERROR, HIGH);
  delay(10);
  digitalWrite(ERROR, LOW);
  delay(10);
  digitalWrite(ERROR, HIGH);
  delay(10);
  digitalWrite(ERROR, LOW);
}

// MotoDirection handleAngle(float rawArcRadian)
MotoDirection handlePayload(RoboCommandPayload payload, float toloranceArc)
{
  float vectorLength = sqrt(payload.x * payload.x + payload.y * payload.y);
  float speedRatio = vectorLength / payload.r;
  float joystickRadian = -1 * atan2(payload.y, payload.x);

  // Process joystick angle within tolerance
  if (joystickRadian < toloranceArc && joystickRadian > -toloranceArc)
  {
    joystickRadian = 0;
  }
  // Determine joystick direction
  else if (joystickRadian > M_PI / 2 - toloranceArc && joystickRadian < M_PI / 2 + toloranceArc)
  {
    joystickRadian = M_PI / 2;
  }
  else if ((joystickRadian > M_PI - toloranceArc && joystickRadian < M_PI) || (joystickRadian < -M_PI + toloranceArc && joystickRadian > -M_PI))
  {
    joystickRadian = M_PI;
  }
  else if (joystickRadian < -M_PI / 2 + toloranceArc && joystickRadian > -M_PI / 2 - toloranceArc)
  {
    joystickRadian = -M_PI / 2;
  }

  // Calculate back and forth (Y) and left and right (X) movements
  float backAndForth = sin(joystickRadian);
  float leftAndRight = cos(joystickRadian);

  // Determine forward/backward and right/left movement directions
  int forward = 1;
  if (backAndForth < 0)
  {
    forward = 0;
    backAndForth = -backAndForth; // Make sure it's positive
  }

  int right = 1;
  if (leftAndRight < 0)
  {
    right = 0;
    leftAndRight = -leftAndRight; // Make sure it's positive
  }

  // Calculate the movement intensities (0-255)
  float moveForward = speedRatio * MAX_SPEED * forward * backAndForth;
  float moveBackward = speedRatio * MAX_SPEED * (1 - forward) * backAndForth;
  float moveRight = speedRatio * MAX_SPEED * right * leftAndRight;
  float moveLeft = speedRatio * MAX_SPEED * (1 - right) * leftAndRight;

  // Print the results
  Serial.print("F:");
  Serial.print(moveForward);
  Serial.print(" B:");
  Serial.print(moveBackward);
  Serial.print(" R:");
  Serial.print(moveRight);
  Serial.print(" L:");
  Serial.println(moveLeft);

  MotoDirection motoDirection;
  motoDirection.forward = moveForward;
  motoDirection.backward = moveBackward;
  motoDirection.left = moveLeft;
  motoDirection.right = moveRight;

  return motoDirection;
}

float getAngle(float x, float y)
{
  if (0.0 == x && 0.0 == y)
  {
    return 0.0;
  }
  else
  {
    return -1 * atan2(y, x);
  }
}

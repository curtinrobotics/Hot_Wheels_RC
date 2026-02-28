#include <Bluepad32.h>

const int servoPin = 32;
const int Forward = 25;
const int Back = 33;

// PWM Configuration for motors
const int PWM_FREQ = 5000;     // 5 KHz PWM frequency for motors
const int PWM_RESOLUTION = 8;  // 8-bit resolution (0-255)
const int FORWARD_PWM_CHANNEL = 0;
const int BACK_PWM_CHANNEL = 1;

// PWM Configuration for servo
const int SERVO_PWM_CHANNEL = 2;
const int SERVO_PWM_FREQ = 50;        // 50 Hz for standard servo
const int SERVO_PWM_RESOLUTION = 14;  // 14-bit for servo control

ControllerPtr myController = nullptr;
int Drive = 0;
int Reverse = 0;
int angle = 90;

const int DEADZONE = 30;
const int SPEED_LIMIT_PERCENT = 100;  // Limit speed to 40% of max

// Trigger threshold and mapping
const int TRIGGER_DEADZONE = 10;
const int TRIGGER_MAX = 1023;

// Servo pulse timing (in microseconds)
const int SERVO_MIN_US = 1000;
const int SERVO_MAX_US = 2000;

// Steering limit (degrees from center=90). Example: 30 -> range 60..120
const int STEER_LIMIT_DEG = 90;

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.print("CALLBACK: Controller is connected, index=");
    Serial.println(ctl->index());
    myController = ctl;

    ctl->setColorLED(0, 255, 0);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.print("CALLBACK: Controller is disconnected from index=");
    Serial.println(ctl->index());

    angle = 90;

    // Stop motors when controller disconnects
    ledcWrite(FORWARD_PWM_CHANNEL, 0);
    ledcWrite(BACK_PWM_CHANNEL, 0);

    // Center servo
    setServoAngle(90);

    myController = nullptr;
  }
}

void processController() {
  if (myController && myController->isConnected() && myController->hasData()) {
    int32_t axisX = myController->axisX();
    int32_t throttle = myController->throttle();
    int32_t brake = myController->brake();

    if (abs(axisX) < DEADZONE) {
      axisX = 0;
    }

    Drive = throttle;
    Reverse = brake;

    const int leftLimit = 90 + STEER_LIMIT_DEG;
    const int rightLimit = 90 - STEER_LIMIT_DEG;

    angle = map(axisX, -511, 512, leftLimit, rightLimit);
    angle = constrain(angle, rightLimit, leftLimit);
  }
}

void setServoAngle(int angle) {
  // Map angle (0-180) to microseconds (1000-2000)
  int pulseWidth = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  // Calculate duty cycle for 14-bit resolution at 50Hz
  // At 50Hz, period = 20ms = 20000 microseconds
  // For 14-bit: max value = 16383 (2^14 - 1)
  // Duty cycle = (pulseWidth / 20000) * 16383
  uint32_t dutyCycle = ((uint32_t)pulseWidth * 16384) / 20000;

  ledcWrite(SERVO_PWM_CHANNEL, dutyCycle);
}

void updateMotorSpeed() {
  int forwardSpeed = 0;
  int reverseSpeed = 0;

  // Calculate max speed based on limit
  int maxSpeed = (255 * SPEED_LIMIT_PERCENT) / 100;

  // Map trigger values to PWM duty cycle with speed limit
  if (Drive >= TRIGGER_DEADZONE) {
    forwardSpeed = map(Drive, TRIGGER_DEADZONE, TRIGGER_MAX, 0, maxSpeed);
    forwardSpeed = constrain(forwardSpeed, 0, maxSpeed);
  }

  if (Reverse >= TRIGGER_DEADZONE) {
    reverseSpeed = map(Reverse, TRIGGER_DEADZONE, TRIGGER_MAX, 0, maxSpeed);
    reverseSpeed = constrain(reverseSpeed, 0, maxSpeed);
  }

  // Prevent both motors from running simultaneously (prioritize forward)
  if (forwardSpeed > 0 && reverseSpeed > 0) {
    reverseSpeed = 0;
  }

  // Update PWM outputs
  ledcWrite(FORWARD_PWM_CHANNEL, forwardSpeed);
  ledcWrite(BACK_PWM_CHANNEL, reverseSpeed);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting RC Car Controller...");

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  // Turn on motor driver
  pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

  // Setup PWM for motor control
  ledcSetup(FORWARD_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(BACK_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Forward, FORWARD_PWM_CHANNEL);
  ledcAttachPin(Back, BACK_PWM_CHANNEL);

  // Setup PWM for servo control
  ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(servoPin, SERVO_PWM_CHANNEL);

  // Initialize motors to stopped
  ledcWrite(FORWARD_PWM_CHANNEL, 0);
  ledcWrite(BACK_PWM_CHANNEL, 0);

  // Center servo
  angle = 90;
  setServoAngle(angle);

  Serial.println("Setup complete. Waiting for Xbox controller...");
  Serial.println("Use left joystick X-axis for steering");
  Serial.println("Use triggers for variable speed control");
  Serial.printf("Speed limited to %d%% of maximum\n", SPEED_LIMIT_PERCENT);
}

void loop() {
  BP32.update();
  processController();

  // Update motor speeds based on trigger values
  updateMotorSpeed();

  // Update servo position using hardware PWM
  setServoAngle(angle);

  delay(1);
}
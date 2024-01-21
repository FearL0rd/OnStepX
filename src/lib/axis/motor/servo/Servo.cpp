// -----------------------------------------------------------------------------------
// axis servo motor

#include "Servo.h"

#ifdef SERVO_MOTOR_PRESENT

#include "../../../tasks/OnTask.h"
#include "../Motor.h"

ServoMotor *servoMotorInstance[9];
IRAM_ATTR void moveServoMotorAxis1() { servoMotorInstance[0]->move(); }
IRAM_ATTR void moveServoMotorAxis2() { servoMotorInstance[1]->move(); }
IRAM_ATTR void moveServoMotorAxis3() { servoMotorInstance[2]->move(); }
IRAM_ATTR void moveServoMotorAxis4() { servoMotorInstance[3]->move(); }
IRAM_ATTR void moveServoMotorAxis5() { servoMotorInstance[4]->move(); }
IRAM_ATTR void moveServoMotorAxis6() { servoMotorInstance[5]->move(); }
IRAM_ATTR void moveServoMotorAxis7() { servoMotorInstance[6]->move(); }
IRAM_ATTR void moveServoMotorAxis8() { servoMotorInstance[7]->move(); }
IRAM_ATTR void moveServoMotorAxis9() { servoMotorInstance[8]->move(); }

// constructor
ServoMotor::ServoMotor(uint8_t axisNumber, ServoDriver *Driver, Encoder *encoder, uint32_t encoderOrigin, bool encoderReverse, Feedback *feedback, ServoControl *control, long syncThreshold, bool useFastHardwareTimers) {
  if (axisNumber < 1 || axisNumber > 9) return;

  driverType = SERVO;
  strcpy(axisPrefix, "MSG: Servo_, ");
  axisPrefix[10] = '0' + axisNumber;
  this->axisNumber = axisNumber;
  this->encoder = encoder;
  this->feedback = feedback;
  this->control = control;
  this->syncThreshold = syncThreshold;

  if (axisNumber > 2) useFastHardwareTimers = false;
  this->useFastHardwareTimers = useFastHardwareTimers;

  this->driver = Driver;

  encoder->init();
  encoder->setOrigin(encoderOrigin);
  this->encoderReverse = encoderReverse;
  this->encoderReverseDefault = encoderReverse;

  feedback->getDefaultParameters(&default_param1, &default_param2, &default_param3, &default_param4, &default_param5, &default_param6);

  // attach the function pointers to the callbacks
  servoMotorInstance[axisNumber - 1] = this;
  switch (axisNumber) {
    case 1: callback = moveServoMotorAxis1; break;
    case 2: callback = moveServoMotorAxis2; break;
    case 3: callback = moveServoMotorAxis3; break;
    case 4: callback = moveServoMotorAxis4; break;
    case 5: callback = moveServoMotorAxis5; break;
    case 6: callback = moveServoMotorAxis6; break;
    case 7: callback = moveServoMotorAxis7; break;
    case 8: callback = moveServoMotorAxis8; break;
    case 9: callback = moveServoMotorAxis9; break;
  }

  // get the feedback control loop ready
  feedback->init(axisNumber, control, driver->getMotorControlRange());
}

bool ServoMotor::init() {
  if (axisNumber < 1 || axisNumber > 9) return false;

  driver->init();
  enable(false);

  // start the motion timer
  V(axisPrefix);
  VF("start task to track motion... ");
  char timerName[] = "Servo_";
  timerName[5] = '0' + axisNumber;
  taskHandle = tasks.add(0, 0, true, 0, callback, timerName);
  if (taskHandle) {
    VF("success");
    if (useFastHardwareTimers) {
      if (!tasks.requestHardwareTimer(taskHandle, 0)) {
        VF(" (no hardware timer!)");
      } else {
        maxFrequency = 1000000.0F/HAL_MAXRATE_LOWER_LIMIT;
      };
    }
    VLF("");
  } else {
    VLF("FAILED!");
    return false;
  }

  return true;
}

// set driver reverse state
void ServoMotor::setReverse(int8_t state) {
  feedback->setControlDirection(state);
  if (state == ON) encoderReverse = encoderReverseDefault; else encoderReverse = !encoderReverseDefault; 
}

// set driver parameters
void ServoMotor::setParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  feedback->setParameters(param1, param2, param3, param4, param5, param6);
}

// validate driver parameters
bool ServoMotor::validateParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  return feedback->validateParameters(param1, param2, param3, param4, param5, param6);
}

// sets motor enable on/off (if possible)
void ServoMotor::enable(bool state) {
  driver->enable(state);
  enabled = state;
}

// get the associated driver status
DriverStatus ServoMotor::getDriverStatus() {
  driver->updateStatus();
  return driver->getStatus();
}

// resets motor and target angular position in steps, also zeros backlash and index
void ServoMotor::resetPositionSteps(long value) {
  Motor::resetPositionSteps(value);
  if (syncThreshold == OFF) {
    encoder->write(value);
  } else {
    V(axisPrefix);
    VL("absolute encoder ignored reset position");
  }
}

// get instrument coordinate, in steps
long ServoMotor::getInstrumentCoordinateSteps() {
  return encoderRead() + indexSteps;
}

// set instrument coordinate, in steps
void ServoMotor::setInstrumentCoordinateSteps(long value) {
  noInterrupts();
  long i = value - motorSteps;
  interrupts();

  bool atHome = indexSteps == 0 && motorSteps == 0 && targetSteps == 0 && backlashSteps == 0;

  if (syncThreshold == OFF || atHome) {
    indexSteps = i;
    originIndexSteps = i;
    if (atHome) homeSet = true;
  } else {
    if (abs(originIndexSteps - i) < syncThreshold) {
      indexSteps = i;
    } else {
      V(axisPrefix); VL("absolute encoder ignored sync exceeds threshold");
    }
  }
}

// distance to target in steps (+/-)
long ServoMotor::getTargetDistanceSteps() {
  long encoderCounts = encoderRead();
  noInterrupts();
  long dist = targetSteps - encoderCounts;
  interrupts();
  return dist;
}

// set frequency (+/-) in steps per second negative frequencies move reverse in direction (0 stops motion)
void ServoMotor::setFrequencySteps(float frequency) {
  // negative frequency, convert to positive and reverse the direction
  int dir = 0;
  if (frequency > 0.0F) dir = 1; else if (frequency < 0.0F) { frequency = -frequency; dir = -1; }

  // if in backlash override the frequency
  if (inBacklash) frequency = backlashFrequency;

  if (frequency != currentFrequency) {
    // compensate for performace limitations by taking larger steps as needed
    if (frequency < maxFrequency) stepSize = 1; else
    if (frequency < maxFrequency*2) stepSize = 2; else
    if (frequency < maxFrequency*4) stepSize = 4; else
    if (frequency < maxFrequency*8) stepSize = 8; else
    if (frequency < maxFrequency*16) stepSize = 16; else
    if (frequency < maxFrequency*32) stepSize = 32; else
    if (frequency < maxFrequency*64) stepSize = 64; else
    if (frequency < maxFrequency*128) stepSize = 128; else stepSize = 256;

    // timer period in microseconds
    float period = 1000000.0F / (frequency/stepSize);

    // range is 0 to 134 seconds/step
    if (!isnan(period) && period <= 130000000.0F) {
      period *= 16.0F;
      lastPeriod = (unsigned long)lroundf(period);
    } else {
      lastPeriod = 0;
      frequency = 0.0F;
      dir = 0;
    }

    currentFrequency = frequency;
    velocityEstimate = -driver->getVelocityEstimate(currentFrequency*dir);

    // change the motor rate/direction
    noInterrupts();
    step = 0;
    interrupts();
    tasks.setPeriodSubMicros(taskHandle, lastPeriod);
  }

  noInterrupts();
  step = dir * stepSize;
  absStep = abs(step);
  interrupts();
}

float ServoMotor::getFrequencySteps() {
  if (lastPeriod == 0) return 0;
  return (16000000.0F / lastPeriod) * absStep;
}

// set slewing state (hint that we are about to slew or are done slewing)
void ServoMotor::setSlewing(bool state) {
  slewing = state;
}

// updates PID and sets servo motor power/direction
void ServoMotor::poll() {
  long encoderCounts = encoderRead();

  long encoderCountsOrig = encoderCounts;

  // for absolute encoders initialize the motor position at startup
  if (syncThreshold != OFF) {
    if (!motorStepsInitDone && encoder->ready && homeSet) {
      noInterrupts();
      motorSteps = encoderCounts;
      targetSteps = encoderCounts;
      backlashSteps = 0;
      interrupts();
      motorStepsInitDone = true;
    }
  }

  long motorCounts = 0;

  noInterrupts();
  motorCounts = motorSteps;
  interrupts();

  encoderCounts = encoderApplyFilter(encoderCounts - motorCounts) + motorCounts;

  control->set = motorCounts;
  control->in = encoderCounts;
  if (enabled) feedback->poll();

  float velocity = velocityEstimate + control->out;
  if (!enabled) velocity = 0.0F;

  // for virtual encoders set the velocity and direction
  if (encoder->isVirtual) {
    encoder->setVelocity(abs(velocity*driver->getVelocityScale()));
    volatile int8_t dir = -1;
    if (velocity < 0.0F) dir = 1;
    encoder->setDirection(&dir);
  }

  delta = motorCounts - encoderCounts;
  velocityPercent = (driver->setMotorVelocity(velocity)/driver->getMotorControlRange()) * 100.0F;
  if (driver->getMotorDirection() == DIR_FORWARD) control->directionHint = 1; else control->directionHint = -1;

  if (feedback->useVariableParameters) {
    feedback->variableParameters(fabs(velocityPercent));
  } else {
    if (!slewing && enabled) {
      if ((long)(millis() - lastSlewingTime) > SERVO_SLEWING_TO_TRACKING_DELAY) feedback->selectTrackingParameters(); else feedback->selectSlewingParameters();
    } else {
      lastSlewingTime = millis();
      feedback->selectSlewingParameters();
    } 
  }

  // if the driver has shutdown itself we should also shutdown
  if (driver->getStatus().fault && enabled) enable(false);

  if (velocityPercent < -33) wasBelow33 = true;
  if (velocityPercent > 33) wasAbove33 = true;

  if (millis() - lastCheckTime > 2000) {

    #ifndef SERVO_SAFETY_DISABLE
      // if above 33% power and we're not moving something is seriously wrong, so shut it down
      if (labs(encoderCounts - lastEncoderCounts) < 10 && abs(velocityPercent) >= 33) {
        D(axisPrefix);
        D("stall detected!"); D(" control->in = "); D(control->in); D(", control->set = "); D(control->set);
        D(", control->out = "); D(control->out); D(", velocity % = "); DL(velocityPercent);
        enable(false);
      }

      // if above 90% power and we're moving away from the target something is seriously wrong, so shut it down
      if (labs(encoderCounts - lastEncoderCounts) > lastTargetDistance && abs(velocityPercent) >= 90) {
        D(axisPrefix);
        DL("runaway detected, > 90% power while moving away from the target!");
        enable(false);
      }
      lastTargetDistance = labs(encoderCounts - lastEncoderCounts);

      // if we were below -33% and above 33% power in a one second period something is seriously wrong, so shut it down
      if (wasBelow33 && wasAbove33) {
        D(axisPrefix);
        DL("oscillation detected, below -33% and above 33% power in a 2 second period!");
        enable(false);
      }
    #endif

    wasAbove33 = false;
    wasBelow33 = false;
    lastEncoderCounts = encoderCounts;
    lastCheckTime = millis();
  }

  #if DEBUG != OFF && defined(DEBUG_SERVO) && DEBUG_SERVO != OFF
    if (axisNumber == DEBUG_SERVO) {
      static uint16_t count = 0;
      count++;
      if (count % 10 == 0) {
        char s[800];

        float spas = 0;
        if (axisNumber == 1) spas = AXIS1_STEPS_PER_DEGREE/3600.0F; else if (axisNumber == 2) spas = AXIS2_STEPS_PER_DEGREE/3600.0F;

        #if AXIS1_SERVO_FLTR == OFF
//        sprintf(s, "Servo%d_Delta: %6ld, Motor %6ld, Encoder %6ld, Servo%d_Power: %6.3f%%\r\n", (int)axisNumber, (motorCounts - encoderCounts), motorCounts, (long)encoderCounts, (int)axisNumber, velocityPercent);
//        sprintf(s, "Servo%d: Motor %6ld, Encoder %6ld\r\n", (int)axisNumber, motorCounts, (long)encoderCounts);
          sprintf(s, "Servo%d: DeltaAS: %0.2f, Servo%d_Power: %6.3f%%\r\n", (int)axisNumber, (motorCounts - encoderCounts)/spas, (int)axisNumber, velocityPercent);
        #else
//        sprintf(s, "Servo%d: Motor %6ld, Encoder %6ld, Encoder2 %6ld\r\n", (int)axisNumber, motorCounts, (long)encoderCounts, (long)encoderCountsOrig);
          sprintf(s, "Servo%d: DeltaAS: %0.2f, Delta2AS: %0.2f, Servo%d_Power: %6.3f%%\r\n", (int)axisNumber, (motorCounts - encoderCounts)/spas, (motorCounts - encoderCountsOrig)/spas, (int)axisNumber, velocityPercent);
        #endif

        D(s);
        UNUSED(spas);
      }
    }
  #endif

  UNUSED(encoderCountsOrig);
}

// sets dir as required and moves coord toward target at setFrequencySteps() rate
IRAM_ATTR void ServoMotor::move() {

  #if SERVO_SLEW_DIRECT == ON
    if (sync && !inBacklash) targetSteps += step;

    if (motorSteps > targetSteps) {
      motorSteps = targetSteps;
    } else

    if (motorSteps < targetSteps + backlashAmountSteps) {
      motorSteps = targetSteps + backlashAmountSteps;
    }

  #else
    if (sync && !inBacklash) targetSteps += step;

    if (motorSteps > targetSteps) {
      if (backlashSteps > 0) {
        backlashSteps -= absStep;
        inBacklash = backlashSteps > 0;
      } else {
        motorSteps -= absStep;
        inBacklash = false;
      }
    } else

    if (motorSteps < targetSteps || inBacklash) {
      if (backlashSteps < backlashAmountSteps) {
        backlashSteps += absStep;
        inBacklash = backlashSteps < backlashAmountSteps;
      } else {
        motorSteps += absStep;
        inBacklash = false;
      }
    }

  #endif
}

int32_t ServoMotor::encoderRead() {
  int32_t encoderCounts = encoder->read();


  if (encoderReverse) encoderCounts = -encoderCounts;
  return encoderCounts;
}

// set zero/origin of absolute encoders
uint32_t ServoMotor::encoderZero() {
  encoder->origin = 0;
  encoder->offset = 0;

  uint32_t zero = (uint32_t)(-encoder->read());
  encoder->origin = zero;

  return zero;
}

#endif

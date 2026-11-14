/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       motions.cpp                                               */
/*    Description:  Devices + every function shared between files.            */
/*                                                                            */
/*    Sections:                                                               */
/*      1. Devices (ports)                                                    */
/*      2. Math helpers                                                       */
/*      3. PID controller                                                     */
/*      4. Transmission                                                       */
/*      5. Drive output                                                       */
/*      6. Mechanisms (lift / claw / active toggle)                           */
/*      7. Motions (turnToHeading, turnToPoint, moveToPoint)                  */
/*      8. Wall reset                                                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "TOC.hpp"

#include <cmath>

using namespace vex;

/*============================================================================*/
/*                              1. DEVICES                                    */
/*      All ports are placeholders - change them to match your robot.         */
/*============================================================================*/

brain Brain;
controller Controller(primary);

// Drive: 55W total = 4x 11W + 2x 5.5W (3 motors per side). 5.5W motors use
// the normal `motor` class; their gear setting is ignored (always 200 rpm).
// The right side is the mirror image of the left, so every reverse flag on
// the right is the OPPOSITE of the same motor on the left. If a whole side
// drives backward, flip all three on that side together.
static motor leftFront(PORT11, ratio18_1, true);
static motor leftMiddle(PORT12, ratio18_1, true); // 5.5W
static motor leftBack(PORT13, ratio18_1, false);
static motor rightFront(PORT18, ratio18_1, false);
static motor rightMiddle(PORT19, ratio18_1, false); // 5.5W
static motor rightBack(PORT20, ratio18_1, true);
motor_group leftDrive(leftFront, leftMiddle, leftBack);
motor_group rightDrive(rightFront, rightMiddle, rightBack);

// Lift: 16.5W = 11W + 5.5W on a continuous cascade. Both motors drive the
// same cascade, so they MUST turn it the same way: if the lift jerks and
// barely moves, they're fighting - flip ONE of these flags. If the whole
// lift goes the wrong way, flip BOTH.
// NEVER put another motor object on port 1 or port 10: the reverse flag is
// stored in the motor itself, so the LAST object created on a port sets its
// direction and silently overrides these flags.
static motor liftMain(PORT1, ratio18_1, true);   // 11W
static motor liftHelper(PORT10, ratio18_1, false); // 5.5W
motor_group lift(liftMain, liftHelper);

// Claw: 11W.
static motor clawMotor(PORT16, ratio18_1, false);
motor_group claw(clawMotor);

// Active toggle: 5.5W. NOT port 10 - that's the lift's 5.5W motor. Port 8 is
// a guess (it's unused): set this to the toggle motor's real port.
static motor toggleMotor(PORT15, ratio18_1, false);
motor_group activeToggle(toggleMotor);

// Transmission shifter: one piston. Retracted = high gear (400 rpm),
// extended = low gear (200 rpm).
digital_out transmissionPiston(Brain.ThreeWirePort.A);

inertial imu(PORT14);

// Distance sensors: one on each side, two on the back, two on the front.
distance leftDistSensor(PORT3);
distance rightDistSensor(PORT5);
distance backLeftDistSensor(PORT2);
distance backRightDistSensor(PORT4);
distance frontLeftDistSensor(PORT6);
distance clawDistSensor(PORT9);

// Where the sensors sit, in inches from the tracking center. Measure these.
float lateralSensorOffset = 6.0; // side sensors: left/right of center
float backSensorOffset = 9.0;    // back sensors: behind center
float frontSensorOffset = 9.0;   // front sensors: ahead of center
float backSensorSpacing = 10.0;  // back sensors: left one to right one
                                 // (centered on the robot)

/*============================================================================*/
/*                            2. MATH HELPERS                                 */
/*============================================================================*/

double toRadians(double degrees) { return degrees * M_PI / 180.0; }

double toDegrees(double radians) { return radians * 180.0 / M_PI; }

// Keep a value between -limit and +limit.
double clamp(double value, double limit) {
  if (value > limit)
    return limit;
  if (value < -limit)
    return -limit;
  return value;
}

// Turn any angle into the range (-180, 180]. Used for "shortest way" errors.
double wrap180(double degrees) {
  double wrapped = std::fmod(degrees + 180.0, 360.0);
  if (wrapped <= 0.0)
    wrapped += 360.0;
  return wrapped - 180.0;
}

// Turn any angle into the range [0, 360).
double wrap360(double degrees) {
  double wrapped = std::fmod(degrees, 360.0);
  if (wrapped < 0.0)
    wrapped += 360.0;
  return wrapped;
}

/*============================================================================*/
/*                           3. PID CONTROLLER                                */
/*============================================================================*/

PID::PID(const PIDGains &gains) : gains(gains) {}

// Returns the output (volts) for the current error. Call once per LOOP_MS.
double PID::update(double error) {
  const double dt = LOOP_MS / 1000.0; // seconds per loop

  // Integral: only build it up when close to the target, and dump it when
  // we cross the target so it doesn't push us past it (anti-windup).
  if (std::fabs(error) < gains.integralRange)
    integral += error * dt;
  else
    integral = 0.0;
  if (error * prevError < 0.0)
    integral = 0.0;

  // Derivative: how fast the error is changing. Skip on the first run
  // because there is no previous error yet.
  double derivative = firstRun ? 0.0 : (error - prevError) / dt;

  prevError = error;
  firstRun = false;
  return gains.kP * error + gains.kI * integral + gains.kD * derivative;
}

void PID::reset() {
  integral = 0.0;
  prevError = 0.0;
  firstRun = true;
}

// Tracks how long a motion has been inside its tolerance. A motion is only
// "done" once it has stayed there for SETTLE_MS (not just passed through).
class SettleTimer {
public:
  bool update(bool inTolerance) {
    msInTolerance = inTolerance ? msInTolerance + LOOP_MS : 0;
    return msInTolerance >= SETTLE_MS;
  }

private:
  int msInTolerance = 0;
};

/*============================================================================*/
/*                            4. TRANSMISSION                                 */
/*============================================================================*/

// The single source of truth for which gear we are in. Starts HIGH because
// the piston powers on retracted, which is high gear.
static Gear currentGear = Gear::HIGH;

// While the brain's clock (ms) is below this, arcadeVolts outputs 0. This
// keeps the driver loop from re-powering the drive in the middle of a shift.
static uint32_t shiftPauseUntilMs = 0;

// Shift gears. If the gear actually changes, the drive is unpowered for
// SHIFT_PAUSE_MS around the piston firing so the gears engage cleanly.
// Blocks for SHIFT_PAUSE_MS in that case.
void setGear(Gear gear) {
  if (gear == currentGear) {
    transmissionPiston.set(gear == Gear::LOW); // make sure piston agrees
    return;
  }
  shiftPauseUntilMs = timer::system() + SHIFT_PAUSE_MS;
  leftDrive.stop(brakeType::coast);
  rightDrive.stop(brakeType::coast);
  this_thread::sleep_for(SHIFT_PAUSE_MS / 2);

  currentGear = gear;
  transmissionPiston.set(gear == Gear::LOW); // extended = low gear
  this_thread::sleep_for(SHIFT_PAUSE_MS / 2);
}

Gear getGear() { return currentGear; }

void toggleGear() { setGear(currentGear == Gear::LOW ? Gear::HIGH : Gear::LOW); }

// How many inches the robot travels per drive motor revolution. Changes
// with the gear, which is why odometry asks for this every loop.
double inchesPerMotorRev() {
  double wheelRpm =
      currentGear == Gear::HIGH ? HIGH_GEAR_WHEEL_RPM : LOW_GEAR_WHEEL_RPM;
  return M_PI * WHEEL_DIAMETER * (wheelRpm / DRIVE_MOTOR_RPM);
}

/*============================================================================*/
/*                             5. DRIVE OUTPUT                                */
/*============================================================================*/

// Arcade drive in volts. Positive forward = drive forward, positive turn =
// turn clockwise (right). Used by both driver control and autonomous.
void arcadeVolts(double forward, double turn) {
  // Mid-shift: let the drive coast (see setGear).
  if (timer::system() < shiftPauseUntilMs) {
    forward = 0.0;
    turn = 0.0;
  }

  double left = forward + turn;
  double right = forward - turn;

  // If either side asks for more than the motors can give, scale BOTH down
  // by the same amount so the robot still curves the way we asked.
  double biggest = std::fmax(std::fabs(left), std::fabs(right));
  if (biggest > MAX_VOLTS) {
    left *= MAX_VOLTS / biggest;
    right *= MAX_VOLTS / biggest;
  }

  // Negative volts spin the motors backward.
  leftDrive.spin(directionType::fwd, left, voltageUnits::volt);
  rightDrive.spin(directionType::fwd, right, voltageUnits::volt);
}

void stopDrive() {
  leftDrive.stop(brakeType::brake);
  rightDrive.stop(brakeType::brake);
}

// Dead-reckoning drive: both sides at `volts` (negative = backward) for `ms`,
// then brake. No PID and no target point - just "push for this long", e.g.
// to square up against a wall or nudge into a goal. Odometry keeps tracking
// the whole time.
void deaddriveforward(double volts, int ms) {
  arcadeVolts(clamp(volts, MAX_VOLTS), 0.0);
  this_thread::sleep_for(ms);
  stopDrive();
}

/*============================================================================*/
/*                      6. MECHANISMS (lift / claw / toggle)                  */
/*============================================================================*/

// Spin a mechanism at a voltage. 0 volts = stop and HOLD position. Clamped
// to +/-MAX_VOLTS so asking for more than the motor has can't happen.
void spinMechanism(motor_group &mech, double volts) {
  if (volts == 0.0)
    mech.stop(brakeType::hold);
  else
    mech.spin(directionType::fwd, clamp(volts, MAX_VOLTS), voltageUnits::volt);
}

// Turns a pair of buttons into a voltage: +volts, -volts, or 0.
double buttonVolts(bool forward, bool reverse, double volts) {
  if (forward)
    return volts;
  if (reverse)
    return -volts;
  return 0.0;
}

// Lift hard stop. The lift's encoder (the first motor in the group, port 1)
// is set to 0 deg when the program starts, with the lift all the way down
// (see pre_auton), and the lift is never driven below LIFT_BOTTOM_DEG. Every
// lift command goes through spinLift / runLiftFor, so the stop works in
// driver control AND auton.
//
// Positive volts = up (R1). This relies on the lift motors' reverse flags
// being right: if R1 LOWERS the lift, flip both flags, or this stop will
// block raising instead of lowering.
//
// Spin the lift at a voltage. 0 volts - or trying to lower it while it's at
// or below the bottom - holds it in place instead.
void spinLift(double volts) {
  bool atBottom = lift.position(rotationUnits::deg) <= LIFT_BOTTOM_DEG;
  if (volts == 0.0 || (volts < 0.0 && atBottom))
    lift.stop(brakeType::hold);
  else
    lift.spin(directionType::fwd, clamp(volts, MAX_VOLTS), voltageUnits::volt);
}

// Run the lift at a voltage for `ms`, then hold. The hard stop is checked
// every loop, so lowering stops the moment the lift reaches the bottom.
void runLiftFor(double volts, int ms) {
  timer elapsed;
  while (elapsed.time(timeUnits::msec) < ms) {
    spinLift(volts);
    this_thread::sleep_for(LOOP_MS);
  }
  spinLift(0.0);
}

// Current lift height in inches above the bottom.
double liftHeightInches() {
  return LIFT_BOTTOM_IN + lift.position(rotationUnits::deg) / LIFT_DEG_PER_INCH;
}

// Lift PID: move the lift to `heightIn` inches (LIFT_BOTTOM_IN = all the way
// down, LIFT_TOP_IN = all the way up), then hold it there. Targets outside
// that range are clamped to it. Blocks until it has been within
// LIFT_TOLERANCE_DEG for SETTLE_MS, or timeoutMs runs out. Goes through
// spinLift, so it can never drive below the bottom. To move the lift while
// driving, run it in a thread, e.g. thread([]() { spinToHeight(12); });
void spinToHeight(double heightIn, int timeoutMs, double maxVolts) {
  if (heightIn > LIFT_TOP_IN)
    heightIn = LIFT_TOP_IN;
  // The PID works in encoder degrees (0 = the bottom).
  double heightDeg = (heightIn - LIFT_BOTTOM_IN) * LIFT_DEG_PER_INCH;
  if (heightDeg < LIFT_BOTTOM_DEG)
    heightDeg = LIFT_BOTTOM_DEG;
  PID liftPID(LIFT_GAINS);
  SettleTimer settle;
  timer elapsed;

  while (elapsed.time(timeUnits::msec) < timeoutMs) {
    double error = heightDeg - lift.position(rotationUnits::deg);
    if (settle.update(std::fabs(error) < LIFT_TOLERANCE_DEG))
      break;
    spinLift(clamp(liftPID.update(error), maxVolts));
    this_thread::sleep_for(LOOP_MS);
  }
  spinLift(0.0); // hold at the height
}

/*============================================================================*/
/*                               7. MOTIONS                                   */
/*============================================================================*/

// Field heading from the robot's current position to (x, y). With reverse,
// it's the heading that points the robot's BACK at the point.
static double headingTo(double x, double y, bool reverse) {
  Pose pose = getPose();
  double heading = toDegrees(std::atan2(x - pose.x, y - pose.y));
  return wrap360(reverse ? heading + 180.0 : heading);
}

// Turn in place to a field heading (degrees, clockwise, 0 = +Y), going the
// way `dir` says. A forced direction is only kept until the robot is within
// 90 degrees of the target; after that it goes the shortest way, so a small
// overshoot is corrected back instead of spinning another full circle.
// (Default arguments live in TOC.hpp only - C++ doesn't allow them here too.)
void turnToHeading(double heading, TurnDirection dir, int timeoutMs,
                   double maxVolts) {
  PID turnPID(TURN_GAINS);
  SettleTimer settle;
  timer elapsed;

  // LONGEST: decide once, at the start, which way round is longer - the
  // opposite of the shortest way. (Positive error = clockwise.)
  if (dir == TurnDirection::LONGEST)
    dir = wrap180(heading - getPose().theta) > 0.0
              ? TurnDirection::COUNTERCLOCKWISE
              : TurnDirection::CLOCKWISE;
  bool forced = dir != TurnDirection::SHORTEST;

  while (elapsed.time(timeUnits::msec) < timeoutMs) {
    // Shortest way around: error is always between -180 and 180.
    double error = wrap180(heading - getPose().theta);
    if (forced) {
      // Clockwise = positive error in [0, 360), counterclockwise = negative
      // in (-360, 0].
      double cw = wrap360(heading - getPose().theta);
      double forcedError = dir == TurnDirection::CLOCKWISE
                               ? cw
                               : (cw == 0.0 ? 0.0 : cw - 360.0);
      if (std::fabs(forcedError) < 90.0)
        forced = false; // close now: shortest way from here on
      else
        error = forcedError;
    }
    if (settle.update(std::fabs(error) < TURN_TOLERANCE))
      break;
    arcadeVolts(0.0, clamp(turnPID.update(error), maxVolts));
    this_thread::sleep_for(LOOP_MS);
  }
  stopDrive();
}

// Turn in place to face (x, y), going the way `dir` says. With reverse, the
// back faces the point.
void turnToPoint(double x, double y, TurnDirection dir, bool reverse,
                 int timeoutMs, double maxVolts) {
  turnToHeading(headingTo(x, y, reverse), dir, timeoutMs, maxVolts);
}

// Drive to (x, y) while steering toward it. With reverse, drives backward.
void moveToPoint(double x, double y, int timeoutMs, bool reverse,
                 double maxVolts) {
  PID drivePID(DRIVE_GAINS);
  PID turnPID(STEER_GAINS); // gentle steering, not the in-place TURN_GAINS
  SettleTimer settle;
  timer elapsed;
  double targetHeading = headingTo(x, y, reverse);

  while (elapsed.time(timeUnits::msec) < timeoutMs) {
    Pose pose = getPose();
    double dx = x - pose.x;
    double dy = y - pose.y;
    double distance = std::hypot(dx, dy);

    // Distance to the point measured along the way the robot is facing.
    // This is negative if the point is behind us, so the robot will back
    // up (in reverse, or after overshooting) without any special cases. It
    // also slows the robot down automatically while it is still turning.
    double theta = toRadians(pose.theta);
    double forwardError = dx * std::sin(theta) + dy * std::cos(theta);

    // Once steering is locked, the robot can only fix error along the way
    // it is facing, so judge "done" by forward error. Otherwise a small
    // sideways miss would keep it waiting until the timeout.
    bool close = distance < STOP_STEERING_DIST;
    if (settle.update(close ? std::fabs(forwardError) < DRIVE_TOLERANCE
                            : distance < DRIVE_TOLERANCE))
      break;

    // Keep re-aiming at the point until we're close, then lock the heading
    // so tiny position errors don't make the robot spin in place.
    if (distance > STOP_STEERING_DIST)
      targetHeading = headingTo(x, y, reverse);
    double turnError = wrap180(targetHeading - pose.theta);

    arcadeVolts(clamp(drivePID.update(forwardError), maxVolts),
                clamp(turnPID.update(turnError), maxVolts));
    this_thread::sleep_for(LOOP_MS);
  }
  stopDrive();
}

/*============================================================================*/
/*                              8. WALL RESET                                 */
/*============================================================================*/

// Reads a distance sensor in inches. Returns -1 if it doesn't see a wall.
static double readInches(distance &sensor) {
  if (!sensor.isObjectDetected())
    return -1.0;
  double inches = sensor.objectDistance(distanceUnits::in);
  return inches <= DIST_MAX_VALID ? inches : -1.0;
}

// Given one sensor reading and the robot heading, work out which wall the
// sensor beam hits and where the robot's center must be. Writes the answer
// into pose.x or pose.y (whichever axis that wall measures).
//   sensorRight   - inches the sensor sits right of center (negative = left)
//   sensorForward - inches the sensor sits forward of center (negative =
//                   behind)
//   facing        - degrees the sensor points, relative to robot forward
//                   (0 = front, 90 = right, 180 = back, 270 = left)
static void fixFromWall(double sensorRight, double sensorForward,
                        double facing, double reading, Pose &pose) {
  double h = toRadians(pose.theta);

  // Where the sensor sits relative to the robot center, in field axes.
  // Robot right = (cos h, -sin h), robot forward = (sin h, cos h).
  double offsetX = sensorRight * std::cos(h) + sensorForward * std::sin(h);
  double offsetY = -sensorRight * std::sin(h) + sensorForward * std::cos(h);

  // Direction the sensor beam points, in field axes.
  double beam = toRadians(pose.theta + facing);
  double beamX = std::sin(beam);
  double beamY = std::cos(beam);

  // The beam mostly points along X or Y: that tells us which wall it hits.
  // center = wall - (sensor offset + beam length along that axis)
  if (std::fabs(beamX) > std::fabs(beamY)) {
    double wallX = beamX > 0 ? FIELD_HALF : -FIELD_HALF;
    pose.x = wallX - (offsetX + reading * beamX);
  } else {
    double wallY = beamY > 0 ? FIELD_HALF : -FIELD_HALF;
    pose.y = wallY - (offsetY + reading * beamY);
  }
}

// Square up off the wall behind the robot and reset heading + position.
// Call while the robot is stopped with its back facing (roughly) a wall.
//
//  1. The OLD heading is rounded to the nearest 90 degrees. That tells us
//     which wall is behind us (0 -> bottom wall, 90 -> left wall, ...).
//  2. The two back sensors give the exact angle to that wall: if one reads
//     longer than the other, the robot is crooked by atan(diff / spacing).
//  3. With the true heading known, the back sensors give the distance to
//     the back wall, and the closer side sensor gives the side wall.
bool resetHeadingOffBackWall() {
  double backLeft = readInches(backLeftDistSensor);
  double backRight = readInches(backRightDistSensor);
  if (backLeft < 0 || backRight < 0)
    return false; // can't see the wall - leave the pose alone

  Pose pose = getPose();

  // 1 + 2: true heading = which wall + how crooked we are against it.
  double wallHeading = std::round(pose.theta / 90.0) * 90.0;
  double skew =
      toDegrees(std::atan2(backLeft - backRight, backSensorSpacing));
  pose.theta = wrap360(wallHeading + skew);

  // 3a: back wall position = average of both back sensors. Each sits
  // backSensorOffset behind center and half the spacing left/right of it,
  // pointing backward (180).
  Pose fromLeft = pose;
  Pose fromRight = pose;
  fixFromWall(-backSensorSpacing / 2, -backSensorOffset, 180.0, backLeft,
              fromLeft);
  fixFromWall(backSensorSpacing / 2, -backSensorOffset, 180.0, backRight,
              fromRight);
  pose.x = (fromLeft.x + fromRight.x) / 2.0;
  pose.y = (fromLeft.y + fromRight.y) / 2.0;

  // 3b: side wall position from whichever side sensor is closer (closer
  // readings are more accurate). Skipped if neither sees a wall. Side
  // sensors sit lateralSensorOffset left/right of center, pointing out.
  double left = readInches(leftDistSensor);
  double right = readInches(rightDistSensor);
  if (left >= 0 && (right < 0 || left <= right))
    fixFromWall(-lateralSensorOffset, 0.0, 270.0, left, pose);
  else if (right >= 0)
    fixFromWall(lateralSensorOffset, 0.0, 90.0, right, pose);

  setPose(pose);
  return true;
}

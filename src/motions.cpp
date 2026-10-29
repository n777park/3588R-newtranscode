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
static motor leftFront(PORT1, ratio18_1, true);
static motor leftMiddle(PORT2, ratio18_1, true); // 5.5W
static motor leftBack(PORT3, ratio18_1, true);
static motor rightFront(PORT4, ratio18_1, false);
static motor rightMiddle(PORT5, ratio18_1, false); // 5.5W
static motor rightBack(PORT6, ratio18_1, false);
motor_group leftDrive(leftFront, leftMiddle, leftBack);
motor_group rightDrive(rightFront, rightMiddle, rightBack);

// Lift: 16.5W = 11W + 5.5W on a continuous cascade.
static motor liftMain(PORT7, ratio18_1, false);  // 11W
static motor liftHelper(PORT8, ratio18_1, true); // 5.5W
motor_group lift(liftMain, liftHelper);

// Claw: 11W.
static motor clawMotor(PORT9, ratio18_1, false);
motor_group claw(clawMotor);

// Active toggle: 5.5W.
static motor toggleMotor(PORT10, ratio18_1, false);
motor_group activeToggle(toggleMotor);

// Transmission shifter: one piston. Extended = high gear (400 rpm).
digital_out transmissionPiston(Brain.ThreeWirePort.A);

inertial imu(PORT11);

// Distance sensors: one on each side, two on the back. Offsets are inches
// from the tracking center (+x right, +y forward). Measure and update them.
static distance leftDistSensor(PORT12);
static distance rightDistSensor(PORT13);
static distance backLeftDistSensor(PORT14);
static distance backRightDistSensor(PORT15);
DistanceMount leftDistance{leftDistSensor, -6.0, 0.0, 270.0};
DistanceMount rightDistance{rightDistSensor, 6.0, 0.0, 90.0};
DistanceMount backLeftDistance{backLeftDistSensor, -5.0, -ROBOT_LENGTH / 2,
                               180.0};
DistanceMount backRightDistance{backRightDistSensor, 5.0, -ROBOT_LENGTH / 2,
                                180.0};

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

// The single source of truth for which gear we are in.
static Gear currentGear = Gear::LOW;

void setGear(Gear gear) {
  currentGear = gear;
  transmissionPiston.set(gear == Gear::HIGH);
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

/*============================================================================*/
/*                      6. MECHANISMS (lift / claw / toggle)                  */
/*============================================================================*/

// Spin a mechanism at a voltage. 0 volts = stop and HOLD position.
void spinMechanism(motor_group &mech, double volts) {
  if (volts == 0.0)
    mech.stop(brakeType::hold);
  else
    mech.spin(directionType::fwd, volts, voltageUnits::volt);
}

// Turns a pair of buttons into a voltage: +volts, -volts, or 0.
double buttonVolts(bool forward, bool reverse, double volts) {
  if (forward)
    return volts;
  if (reverse)
    return -volts;
  return 0.0;
}

// PID a mechanism to a motor angle (degrees), then hold it there.
void moveMechanismTo(motor_group &mech, double targetDeg,
                     const PIDGains &gains, int timeoutMs) {
  PID pid(gains);
  SettleTimer settle;
  timer elapsed;

  while (elapsed.time(timeUnits::msec) < timeoutMs) {
    double error = targetDeg - mech.position(rotationUnits::deg);
    if (settle.update(std::fabs(error) < MECH_TOLERANCE))
      break;
    // Spin directly (not spinMechanism) so an output of exactly 0 volts
    // mid-motion doesn't switch to hold.
    double volts = clamp(pid.update(error), MAX_VOLTS);
    mech.spin(directionType::fwd, volts, voltageUnits::volt);
    this_thread::sleep_for(LOOP_MS);
  }
  spinMechanism(mech, 0.0);
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

// Turn in place to a field heading (degrees, clockwise, 0 = +Y).
void turnToHeading(double heading, int timeoutMs, double maxVolts) {
  PID turnPID(TURN_GAINS[(int)getGear()]);
  SettleTimer settle;
  timer elapsed;

  while (elapsed.time(timeUnits::msec) < timeoutMs) {
    // Shortest way around: error is always between -180 and 180.
    double error = wrap180(heading - getPose().theta);
    if (settle.update(std::fabs(error) < TURN_TOLERANCE))
      break;
    arcadeVolts(0.0, clamp(turnPID.update(error), maxVolts));
    this_thread::sleep_for(LOOP_MS);
  }
  stopDrive();
}

// Turn in place to face (x, y). With reverse, the back faces the point.
void turnToPoint(double x, double y, bool reverse, int timeoutMs,
                 double maxVolts) {
  turnToHeading(headingTo(x, y, reverse), timeoutMs, maxVolts);
}

// Drive to (x, y) while steering toward it. With reverse, drives backward.
void moveToPoint(double x, double y, bool reverse, int timeoutMs,
                 double maxVolts) {
  PID drivePID(DRIVE_GAINS[(int)getGear()]);
  PID turnPID(TURN_GAINS[(int)getGear()]);
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
static double readInches(const DistanceMount &mount) {
  if (!mount.sensor.isObjectDetected())
    return -1.0;
  double inches = mount.sensor.objectDistance(distanceUnits::in);
  return inches <= DIST_MAX_VALID ? inches : -1.0;
}

// Given one sensor reading and the robot heading, work out which wall the
// sensor beam hits and where the robot's center must be. Writes the answer
// into pose.x or pose.y (whichever axis that wall measures).
static void fixFromWall(const DistanceMount &mount, double reading,
                        Pose &pose) {
  double h = toRadians(pose.theta);

  // Where the sensor sits relative to the robot center, in field axes.
  // Robot right = (cos h, -sin h), robot forward = (sin h, cos h).
  double offsetX = mount.x * std::cos(h) + mount.y * std::sin(h);
  double offsetY = -mount.x * std::sin(h) + mount.y * std::cos(h);

  // Direction the sensor beam points, in field axes.
  double beam = toRadians(pose.theta + mount.facing);
  double beamX = std::sin(beam);
  double beamY = std::cos(beam);

  // The beam mostly points along X or Y: that tells us which wall it hits.
  // center = wall - (sensor offset + beam length along that axis)
  if (std::fabs(beamX) > std::fabs(beamY)) {
    double wallX = beamX > 0 ? FIELD_SIZE : 0.0;
    pose.x = wallX - (offsetX + reading * beamX);
  } else {
    double wallY = beamY > 0 ? FIELD_SIZE : 0.0;
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
  double backLeft = readInches(backLeftDistance);
  double backRight = readInches(backRightDistance);
  if (backLeft < 0 || backRight < 0)
    return false; // can't see the wall - leave the pose alone

  Pose pose = getPose();

  // 1 + 2: true heading = which wall + how crooked we are against it.
  double wallHeading = std::round(pose.theta / 90.0) * 90.0;
  double spacing = backRightDistance.x - backLeftDistance.x;
  double skew = toDegrees(std::atan2(backLeft - backRight, spacing));
  pose.theta = wrap360(wallHeading + skew);

  // 3a: back wall position = average of both back sensors.
  Pose fromLeft = pose;
  Pose fromRight = pose;
  fixFromWall(backLeftDistance, backLeft, fromLeft);
  fixFromWall(backRightDistance, backRight, fromRight);
  pose.x = (fromLeft.x + fromRight.x) / 2.0;
  pose.y = (fromLeft.y + fromRight.y) / 2.0;

  // 3b: side wall position from whichever side sensor is closer (closer
  // readings are more accurate). Skipped if neither sees a wall.
  double left = readInches(leftDistance);
  double right = readInches(rightDistance);
  if (left >= 0 && (right < 0 || left <= right))
    fixFromWall(leftDistance, left, pose);
  else if (right >= 0)
    fixFromWall(rightDistance, right, pose);

  setPose(pose);
  return true;
}

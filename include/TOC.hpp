/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       TOC.hpp  ("Table Of Contents")                            */
/*    Description:  The ONE shared header for the whole project.              */
/*                                                                            */
/*    Every constant, device, type, and function that is used by more than    */
/*    one file is declared here exactly once (Single Source of Truth).        */
/*    If you need to tune a number, it lives in this file - nowhere else.     */
/*                                                                            */
/*    Coordinate system used everywhere:                                      */
/*      - Units are inches and degrees.                                       */
/*      - Origin (0,0) is the CENTER of the field, like a graph. Seen from    */
/*        the audience: +Y = north (away), -Y = south (audience wall),        */
/*        +X = east, -X = west (red Alliance Station wall). Walls are at      */
/*        +/-FIELD_HALF.                                                      */
/*      - Heading 0 faces +Y, and heading increases CLOCKWISE (same as the    */
/*        inertial sensor), so heading 90 faces +X.                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#pragma once

#include "vex.h"

/*============================================================================*/
/*                              ROBOT GEOMETRY                                */
/*============================================================================*/

// Drive wheel diameter in inches. 2.75" wheels rarely measure exactly 2.75":
// drive a known distance, compare to what odometry says, and adjust this.
constexpr double WHEEL_DIAMETER = 2.75;

// Distance between the left and right wheel contact patches, in inches.
// The frame is 25 holes wide (24 gaps x 0.5" = 12"). Measure the real
// center-to-center distance of your wheels and put it here.
constexpr double TRACK_WIDTH = 12.0;

// Robot length front-to-back, in inches (used for distance sensor offsets).
constexpr double ROBOT_LENGTH = 18.0;

// Robot width side-to-side, in inches (outside of the frame, not the wheels).
constexpr double ROBOT_WIDTH = 12.5;

// Inside size of the field walls, in inches. A VEX field is nominally 12ft,
// but the inside of the walls is about 140.4". Measure your field.
constexpr double FIELD_SIZE = 140.4;

// Center of the field to any wall (70.2"). West wall x = -FIELD_HALF,
// south wall y = -FIELD_HALF, and so on.
constexpr double FIELD_HALF = FIELD_SIZE / 2;

/*============================================================================*/
/*                               TRANSMISSION                                 */
/*============================================================================*/

// Free speed of every drive motor's output shaft (11W green cartridge and
// 5.5W motors are both 200 rpm).
constexpr double DRIVE_MOTOR_RPM = 200.0;

// Wheel rpm in each gear. One piston shifts between the two.
constexpr double LOW_GEAR_WHEEL_RPM = 200.0;
constexpr double HIGH_GEAR_WHEEL_RPM = 400.0;

// The two gears.
enum class Gear { LOW, HIGH };

// Drive power is cut for this long around a shift so the gears aren't
// loaded while they engage (the piston fires halfway through).
constexpr int SHIFT_PAUSE_MS = 80;

/*============================================================================*/
/*                               PID TUNING                                   */
/*============================================================================*/

// One set of PID constants.
struct PIDGains {
  double kP;            // proportional: output per unit of error
  double kI;            // integral: output per unit of accumulated error
  double kD;            // derivative: output per unit of error change / sec
  double integralRange; // only integrate when |error| is smaller than this
};

// All gains below are STARTING POINTS - tune them on the real robot.
// Integral and derivative are per second (see PID::update in motions.cpp).

// Auton only runs in high gear (400 rpm), so there is one set of gains,
// tuned for high gear. Don't run motions in low gear without retuning.

// Drive (forward/back) gains. Error is in inches, output is volts.
constexpr PIDGains DRIVE_GAINS = {0.7, 0.0, 0.08, 3.0};

// Turn gains. Error is in degrees, output is volts.
constexpr PIDGains TURN_GAINS = {0.18, 0.03, 0.012, 10.0};

/*============================================================================*/
/*                         MOTION / LOOP SETTINGS                             */
/*============================================================================*/

constexpr int LOOP_MS = 10;          // period of every control loop (ms)
constexpr double MAX_VOLTS = 12.0;   // V5 motors max out at 12 volts

constexpr double DRIVE_TOLERANCE = 1.0;  // inches: "close enough" to a point
constexpr double TURN_TOLERANCE = 1.5;   // degrees: "close enough" to a heading
constexpr int SETTLE_MS = 100;           // must stay in tolerance this long

// When moveToPoint is closer than this, it stops steering. This prevents
// the robot from spinning in circles when it is right on top of the target.
constexpr double STOP_STEERING_DIST = 6.0;

/*============================================================================*/
/*                      KALMAN FILTER NOISE SETTINGS                          */
/*============================================================================*/

// How much uncertainty is added per inch driven (wheel slip, scrub).
constexpr double EKF_Q_XY_PER_IN = 0.02;
// How much heading uncertainty is added per radian of wheel-measured turn.
constexpr double EKF_Q_THETA_PER_RAD = 0.05;
// Small constant noise added every step so the filter never "freezes".
constexpr double EKF_Q_MIN = 1e-6;
// How noisy the inertial sensor heading is (radians^2). Smaller = trust more.
constexpr double EKF_R_IMU = 1e-4;

/*============================================================================*/
/*                         DISTANCE SENSOR SETTINGS                           */
/*============================================================================*/

// Readings above this many inches are ignored (too far / likely not a wall).
constexpr double DIST_MAX_VALID = 70.0;

/*============================================================================*/
/*                        DRIVER CONTROL SETTINGS                             */
/*============================================================================*/

constexpr double JOYSTICK_DEADBAND = 5.0; // percent; ignore tiny stick drift
constexpr double LIFT_VOLTS = 12.0;       // lift speed when a button is held
constexpr double CLAW_VOLTS = 10.0;       // claw speed when a button is held
constexpr double TOGGLE_VOLTS = 12.0;     // active toggle speed

/*============================================================================*/
/*                      STACKING (lift / claw settings)                       */
/*        ALL PLACEHOLDERS - time them on the robot with a stopwatch.         */
/*============================================================================*/

// The lift is TIME-based (no PID). Each height is "ms of lifting at full
// LIFT_TIMED_VOLTS, starting from the very bottom". The skills code keeps
// track of the current height in those ms and runs the lift up or down for
// the difference.
constexpr double LIFT_TIMED_VOLTS = 12.0;
constexpr int LIFT_GRAB_MS = 0;    // all the way down: claw at floor height
constexpr int LIFT_CARRY_MS = 150; // just off the floor while driving
constexpr int LIFT_LOAD_MS = 250;  // claw lined up with loader opening
// Height to place an object on top of a stack that is already `level` pins
// tall: LIFT_PLACE_BASE_MS + level * LIFT_MS_PER_LEVEL. (level 0 = empty
// goal.) One Pin/Cup level is about 6.5" tall.
constexpr int LIFT_PLACE_BASE_MS = 350;
constexpr int LIFT_MS_PER_LEVEL = 250;
// Gravity helps going down, so the same distance down usually takes less
// time. Down time = up time x this.
constexpr double LIFT_DOWN_SCALE = 0.8;
// Extra time to keep pushing down when going to the bottom, so the lift
// really reaches it and the tracked height is exact again.
constexpr int LIFT_BOTTOM_EXTRA_MS = 150;

// Ratchet claw (one 11W motor, banded shut). Spinning IN turns the wheels
// to pull an object in; spinning OUT locks the wheels on the ratchet so the
// motor opens the jaws instead. Stopped, the bands close it. If IN and OUT
// are swapped on the robot, flip the claw motor's reverse flag in
// motions.cpp.
constexpr double CLAW_GRAB_VOLTS = 8.0;     // spin in (grab)
constexpr double CLAW_RELEASE_VOLTS = 12.0; // spin out (open, let go)
constexpr int CLAW_OPEN_MS = 250;           // time for the jaws to open
constexpr double CLAW_SPIN_OUT_IN = 4.0; // at the loader: inches backed out
                                         // before the claw stops spinning

constexpr double TOGGLE_FLIP_VOLTS = 12.0; // active toggle spin to flip
constexpr int TOGGLE_FLIP_MS = 600;        // how long to spin it
constexpr int LOAD_WAIT_MS = 800; // time for the loader to drop a match load

/*============================================================================*/
/*                                 TYPES                                      */
/*============================================================================*/

// Where the robot is on the field.
struct Pose {
  double x;     // inches
  double y;     // inches
  double theta; // degrees, clockwise, 0 = facing +Y
};

// A distance sensor plus where it is mounted on the robot. Offsets are
// measured from the robot's tracking center: +x to the right, +y forward.
struct DistanceMount {
  vex::distance &sensor;
  double x;      // inches right of center (negative = left)
  double y;      // inches forward of center (negative = behind)
  double facing; // degrees the sensor points, relative to robot forward
                 // (0 = front, 90 = right, 180 = back, 270 = left)
};

// A PID controller. Create one per motion, call update() every loop.
class PID {
public:
  explicit PID(const PIDGains &gains);
  double update(double error); // returns the output for this error
  void reset();                // clears integral and derivative history

private:
  PIDGains gains;
  double integral = 0.0;
  double prevError = 0.0;
  bool firstRun = true;
};

// Extended Kalman Filter tracking [x, y, theta]. Wheel encoders predict
// motion; the inertial sensor corrects heading. See EKF.cpp.
class EKF {
public:
  void reset(const Pose &pose);                 // jump to a known pose
  void predict(double dLeft, double dRight);    // wheel travel (inches)
  void updateHeading(double measuredHeadingDeg); // inertial sensor reading
  Pose getPose() const;                         // current best estimate

private:
  double state[3] = {0, 0, 0};                    // x, y, theta (radians)
  double P[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // covariance
};

/*============================================================================*/
/*                                DEVICES                                     */
/*              (defined in motions.cpp - ports are set there)                */
/*============================================================================*/

extern vex::brain Brain;
extern vex::controller Controller;

extern vex::motor_group leftDrive;  // 2x 11W + 1x 5.5W
extern vex::motor_group rightDrive; // 2x 11W + 1x 5.5W
extern vex::motor_group lift;       // 11W + 5.5W continuous cascade
extern vex::motor_group claw;       // 11W
extern vex::motor_group activeToggle; // 5.5W

extern vex::digital_out transmissionPiston;
extern vex::inertial imu;

// Raw distance sensors, e.g. leftDistSensor.objectDistance(vex::inches).
extern vex::distance leftDistSensor;
extern vex::distance rightDistSensor;
extern vex::distance backLeftDistSensor;
extern vex::distance backRightDistSensor;

// The same sensors plus where they're mounted (used by the wall reset).
extern DistanceMount leftDistance;
extern DistanceMount rightDistance;
extern DistanceMount backLeftDistance;
extern DistanceMount backRightDistance;

/*============================================================================*/
/*                     SHARED FUNCTIONS  (motions.cpp)                        */
/*============================================================================*/

// ---- Math helpers ----
double toRadians(double degrees);
double toDegrees(double radians);
double clamp(double value, double limit);  // keeps value in [-limit, limit]
double wrap180(double degrees);            // any angle -> (-180, 180]
double wrap360(double degrees);            // any angle -> [0, 360)

// ---- Transmission ----
void setGear(Gear gear);
Gear getGear();
void toggleGear();
double inchesPerMotorRev(); // depends on the current gear

// ---- Drive output ----
void arcadeVolts(double forward, double turn); // used by driver AND autons
void stopDrive();

// ---- Mechanisms (lift, claw, active toggle) ----
void spinMechanism(vex::motor_group &mech, double volts); // 0 = hold
double buttonVolts(bool forward, bool reverse, double volts);

// ---- Motions (all PID, all use odometry) ----
void turnToHeading(double heading, int timeoutMs = 2000,
                   double maxVolts = MAX_VOLTS);
void turnToPoint(double x, double y, bool reverse = false,
                 int timeoutMs = 2000, double maxVolts = MAX_VOLTS);
void moveToPoint(double x, double y, bool reverse = false,
                 int timeoutMs = 4000, double maxVolts = MAX_VOLTS);

// ---- Wall reset ----
bool resetHeadingOffBackWall(); // true if both back sensors saw a wall

/*============================================================================*/
/*                       ODOMETRY  (odom.cpp)                                 */
/*============================================================================*/

void startOdometry();         // call once after the IMU is calibrated
Pose getPose();               // where the robot thinks it is right now
void setPose(const Pose &pose); // tell the robot where it is

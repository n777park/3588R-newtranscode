/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odom.cpp                                                  */
/*    Description:  Odometry - keeps track of where the robot is.             */
/*                                                                            */
/*    A background thread runs every LOOP_MS. Each time it:                   */
/*      1. Reads how far each side of the drive turned since last time.       */
/*      2. Converts that to inches using the CURRENT transmission gear.       */
/*      3. Feeds it to the Kalman filter (predict step).                      */
/*      4. Feeds the inertial sensor heading to the filter (update step).     */
/*                                                                            */
/*    There are no tracking wheels: the drive motor encoders are used, so     */
/*    the gear ratio from the transmission matters (see inchesPerMotorRev).   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "TOC.hpp"

// The one and only position estimate. Only this file touches it directly;
// everyone else goes through getPose() / setPose().
static EKF filter;

// Protects `filter` and `imuOffset`, since the odometry thread and the
// motion code can both use them.
static vex::mutex poseLock;

// Last encoder readings (motor revolutions), so we can find the change.
static double prevLeftRev = 0.0;
static double prevRightRev = 0.0;

// The inertial sensor starts at 0 when calibrated. This offset turns its
// reading into a field heading: fieldHeading = imu.rotation() + imuOffset.
static double imuOffset = 0.0;

// The background loop that keeps the pose up to date.
static int odometryTask() {
  while (true) {
    // 1. Read encoders and find how far each side moved since last loop.
    double leftRev = leftDrive.position(vex::rotationUnits::rev);
    double rightRev = rightDrive.position(vex::rotationUnits::rev);
    double dLeft = (leftRev - prevLeftRev) * inchesPerMotorRev();
    double dRight = (rightRev - prevRightRev) * inchesPerMotorRev();
    prevLeftRev = leftRev;
    prevRightRev = rightRev;

    poseLock.lock();
    // 2. Predict: move the estimate using the wheels.
    filter.predict(dLeft, dRight);
    // 3. Update: correct heading with the inertial sensor.
    filter.updateHeading(imu.rotation(vex::rotationUnits::deg) + imuOffset);
    poseLock.unlock();

    vex::this_thread::sleep_for(LOOP_MS);
  }
  return 0;
}

// Start tracking. Call once, AFTER the inertial sensor finishes calibrating.
void startOdometry() {
  prevLeftRev = leftDrive.position(vex::rotationUnits::rev);
  prevRightRev = rightDrive.position(vex::rotationUnits::rev);
  setPose({0, 0, 0});
  vex::thread odomThread(odometryTask);
}

// Where the robot thinks it is right now.
Pose getPose() {
  poseLock.lock();
  Pose pose = filter.getPose();
  poseLock.unlock();
  return pose;
}

// Tell the robot exactly where it is (start of auton, after a wall reset...).
void setPose(const Pose &pose) {
  poseLock.lock();
  filter.reset(pose);
  // Re-line-up the inertial sensor so it agrees with the new heading.
  imuOffset = pose.theta - imu.rotation(vex::rotationUnits::deg);
  poseLock.unlock();
}

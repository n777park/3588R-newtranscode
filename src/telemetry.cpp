/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       telemetry.cpp                                             */
/*    Description:  Logging for the field visualizer (Documents/VEX/visualizer).*/
/*                                                                            */
/*    Uses vexlog.hpp (copied from visualizer/brain). Line format and stream  */
/*    names follow visualizer/docs/log-format.md, so the visualizer's field   */
/*    view and Diagnose tab understand them.                                  */
/*                                                                            */
/*      SD card  - every odometry loop (10 ms) -> log_NNN.txt. Copy it off    */
/*                 the card and use Playback -> Import SD log.                */
/*      USB      - every TELEMETRY_LIVE_EVERY-th sample, for a live view      */
/*                 with the robot plugged into the laptop (server.py).        */
/*      VEXlink  - optional: pose at 10 Hz to the worker brain, so the live   */
/*                 view works untethered (see TELEMETRY_LINK_* in TOC.hpp).   */
/*                                                                            */
/*    Logging only formats text into RAM; a low-priority thread does the      */
/*    slow SD / USB writes, so it never slows the odometry loop down.         */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "TOC.hpp"
#include "vexlog.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>

using namespace vex;

// Created in startTelemetry(), so their schema lines are written first.
static vexlog::Stream *poseStream = nullptr;
static vexlog::Stream *imuStream = nullptr;
static vexlog::Stream *driveStream = nullptr;
static vexlog::Stream *motorStream = nullptr;
static vexlog::Stream *distStream = nullptr;
static vexlog::Stream *liftStream = nullptr;

// Lift target and PID output while spinToHeight runs; NAN when it isn't.
static volatile double liftRefDeg = NAN;
static volatile double liftPidVolts = NAN;

// Latest pose for the VEXlink thread (written by the odometry thread).
static volatile double linkX = 0.0, linkY = 0.0, linkTheta = 0.0;

// A command this close to 12 V means the motors are maxed out.
static const double SATURATED_VOLTS = 11.5;

// Distance sensor in inches, or NAN (a gap in the plot) if it sees nothing.
static double distOrNan(distance &sensor) {
  if (!sensor.isObjectDetected())
    return NAN;
  return sensor.objectDistance(distanceUnits::in);
}

// Sends the pose to the worker brain 10 times a second, as "x,y,theta\n" -
// the plain CSV the worker already relays to the visualizer.
static int linkTask() {
  static serial_link link(TELEMETRY_LINK_PORT - 1, // SDK ports are 0-based
                          TELEMETRY_LINK_NAME,
                          linkType::manager);
  char packet[64];
  while (true) {
    if (link.isLinked()) {
      int n = snprintf(packet, sizeof packet, "%.3f,%.3f,%.3f\n",
                            (double)linkX, (double)linkY, (double)linkTheta);
      if (n > 0 && n < (int)sizeof packet)
        link.send(packet, n);
    }
    this_thread::sleep_for(100);
  }
  return 0;
}

void startTelemetry() {
  vexlog::Logger &log = vexlog::get();

  // Declare every stream before opening the card, so the schemas come first.
  // The last argument thins that stream out on the live link even further.
  poseStream = &log.stream("pose", {"x", "y", "theta"});
  imuStream = &log.stream("imu", {"heading"});
  driveStream = &log.stream(
      "drive", {"l_vel", "r_vel", "l_cmd", "r_cmd", "batt", "sat"});
  motorStream = &log.stream("motors",
                            {"l_cur", "r_cur", "lift_cur", "claw_cur",
                             "l_temp", "r_temp", "lift_temp", "claw_temp"},
                            10);
  distStream = &log.stream(
      "dist", {"left", "right", "back_l", "back_r", "front_l", "front_r"}, 2);
  liftStream = &log.stream("lift", {"pos", "ref", "fb"});

  if (Brain.SDcard.isInserted() && !log.openSD())
    std::printf("telemetry: SD card found but log file could not open\n");

  // USB serial to the laptop (only goes anywhere while plugged in).
  log.setLiveSink(
      [](const char *data, size_t len) {
        std::fwrite(data, 1, len, stdout);
        std::fflush(stdout);
      },
      TELEMETRY_LIVE_EVERY);
  log.startFlushTask();

  if (TELEMETRY_LINK_ENABLED) {
    thread t(linkTask);
  }
}

void telemetryLoopStart() { vexlog::get().loopStart(); }

void logTelemetry(const Pose &pose) {
  if (!poseStream)
    return; // startTelemetry() hasn't run

  linkX = pose.x;
  linkY = pose.y;
  linkTheta = pose.theta;
  poseStream->log({pose.x, pose.y, pose.theta});

  imuStream->log({imu.rotation(rotationUnits::deg)});

  // Wheel speed in inches per second (motor rpm -> wheel travel).
  double inPerSecPerRpm = inchesPerMotorRev() / 60.0;
  double lVel = leftDrive.velocity(velocityUnits::rpm) * inPerSecPerRpm;
  double rVel = rightDrive.velocity(velocityUnits::rpm) * inPerSecPerRpm;
  double lCmd = leftDrive.voltage(voltageUnits::volt);
  double rCmd = rightDrive.voltage(voltageUnits::volt);
  double sat = (std::fabs(lCmd) >= SATURATED_VOLTS ||
                std::fabs(rCmd) >= SATURATED_VOLTS)
                   ? 1.0
                   : 0.0;
  driveStream->log({lVel, rVel, lCmd, rCmd, Brain.Battery.voltage(), sat});

  motorStream->log({leftDrive.current(currentUnits::amp),
                    rightDrive.current(currentUnits::amp),
                    lift.current(currentUnits::amp),
                    claw.current(currentUnits::amp),
                    leftDrive.temperature(temperatureUnits::celsius),
                    rightDrive.temperature(temperatureUnits::celsius),
                    lift.temperature(temperatureUnits::celsius),
                    claw.temperature(temperatureUnits::celsius)});

  distStream->log({distOrNan(leftDistSensor), distOrNan(rightDistSensor),
                   distOrNan(backLeftDistSensor),
                   distOrNan(backRightDistSensor),
                   distOrNan(frontLeftDistSensor),
                   distOrNan(frontRightDistSensor)});

  liftStream->log({lift.position(rotationUnits::deg), (double)liftRefDeg,
                   (double)liftPidVolts});

  vexlog::get().loopEnd(); // "loop" stream: odometry loop timing
}

void telemetryEvent(const char *fmt, ...) {
  char text[120];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof text, fmt, ap);
  va_end(ap);
  vexlog::get().event("%s", text);
}

void telemetryLift(double refDeg, double pidVolts) {
  liftRefDeg = refDeg;
  liftPidVolts = pidVolts;
}

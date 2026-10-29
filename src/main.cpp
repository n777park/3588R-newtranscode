/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Nathan Park                                               */
/*    Created:      10/28/2026, 5:56:05 PM                                    */
/*    Description:  Competition entry point: autons, auton selector, and      */
/*                  driver control. Shared code lives in motions.cpp and is   */
/*                  declared in TOC.hpp.                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "TOC.hpp"

#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

/*============================================================================*/
/*                              AUTON ROUTINES                                */
/*    Placeholders that show how to use the motions - replace with real ones. */
/*============================================================================*/

// Match auton starting on the left side.
static void matchLeft() {
  setPose({24.0, 9.0, 0.0}); // where the robot starts: set this FIRST
  moveToPoint(24.0, 48.0);
  moveMechanismTo(lift, 600.0, LIFT_GAINS, 1500);
  turnToPoint(48.0, 48.0);
  moveToPoint(48.0, 48.0);
}

// Match auton starting on the right side (mirror of the left side).
static void matchRight() {
  setPose({FIELD_SIZE - 24.0, 9.0, 0.0});
  moveToPoint(FIELD_SIZE - 24.0, 48.0);
  moveMechanismTo(lift, 600.0, LIFT_GAINS, 1500);
  turnToPoint(FIELD_SIZE - 48.0, 48.0);
  moveToPoint(FIELD_SIZE - 48.0, 48.0);
}

// Skills auton. Uses a wall reset to wipe out drift partway through.
static void skills() {
  setPose({FIELD_SIZE / 2, 9.0, 0.0});
  setGear(Gear::HIGH);
  moveToPoint(FIELD_SIZE / 2, 72.0);
  setGear(Gear::LOW);
  moveToPoint(FIELD_SIZE / 2, 20.0, true); // back up toward the start wall
  resetHeadingOffBackWall();
  turnToHeading(90.0);
}

// Does nothing. Safe default if no auton is chosen.
static void doNothing() {}

// Test / tuning routine: drive a square-ish path and come back.
static void test() {
  setPose({0.0, 0.0, 0.0});
  moveToPoint(0.0, 24.0);
  turnToHeading(90.0);
  moveToPoint(24.0, 24.0);
  turnToPoint(0.0, 0.0);
  moveToPoint(0.0, 0.0);
  turnToHeading(0.0);
}

/*============================================================================*/
/*                              AUTON SELECTOR                                */
/*============================================================================*/

// Button labels, in screen order. To add an auton: add its name here AND a
// matching case in autonomous() below (same position = same number).
static const char *AUTON_NAMES[] = {
    "Match Left",  // 0
    "Match Right", // 1
    "Skills",      // 2
    "Do Nothing",  // 3
    "Test",        // 4
};
static const int AUTON_COUNT = sizeof(AUTON_NAMES) / sizeof(AUTON_NAMES[0]);

static const int SCREEN_WIDTH = 480;  // V5 brain screen size in pixels
static const int SCREEN_HEIGHT = 272;
static const int ROW_HEIGHT = SCREEN_HEIGHT / AUTON_COUNT;

// Index into AUTON_NAMES. Starts on "Do Nothing" so a forgotten selection is safe.
static int selectedAuton = 3;

// Draw one full-width button per auton. The selected one is green.
static void drawSelector() {
  Brain.Screen.clearScreen();
  for (int i = 0; i < AUTON_COUNT; i++) {
    int top = i * ROW_HEIGHT;
    Brain.Screen.setPenColor(white);
    if (i == selectedAuton)
      Brain.Screen.setFillColor(green);
    else
      Brain.Screen.setFillColor("#303030");
    Brain.Screen.drawRectangle(0, top, SCREEN_WIDTH, ROW_HEIGHT - 2);
    // `false` = transparent text background, so the button color shows.
    Brain.Screen.printAt(20, top + ROW_HEIGHT / 2 + 6, false, "%s",
                         AUTON_NAMES[i]);
  }
}

// Screen touch: whichever row was tapped becomes the selected auton.
static void onScreenPressed() {
  int row = Brain.Screen.yPosition() / ROW_HEIGHT;
  if (row >= 0 && row < AUTON_COUNT) {
    selectedAuton = row;
    drawSelector();
  }
}

/*============================================================================*/
/*                           COMPETITION CALLBACKS                            */
/*============================================================================*/

// Runs once at power-on. Calibrate sensors, start odometry, show selector.
void pre_auton(void) {
  setGear(Gear::LOW);

  // The robot must sit still while the inertial sensor calibrates (~2 s).
  // The short wait gives isCalibrating() time to turn true, so the loop
  // can't exit before calibration has actually started.
  imu.calibrate();
  wait(100, msec);
  while (imu.isCalibrating())
    wait(20, msec);

  startOdometry();

  drawSelector();
  Brain.Screen.pressed(onScreenPressed);
}

// Run whichever auton is selected on the brain screen.
// Case numbers must match the positions in AUTON_NAMES.
void autonomous(void) {
  switch (selectedAuton) {
  case 0:
    matchLeft();
    break;
  case 1:
    matchRight();
    break;
  case 2:
    skills();
    break;
  case 3:
    doNothing();
    break;
  case 4:
    test();
    break;
  default:
    break; // unknown selection: do nothing (safe)
  }
}

// Joystick percent -> volts, ignoring tiny amounts of stick drift.
static double stickVolts(double percent) {
  if (std::fabs(percent) < JOYSTICK_DEADBAND)
    return 0.0;
  return percent * MAX_VOLTS / 100.0;
}

// Driver wall reset: rumble the controller if the back sensors can't see a
// wall, so the driver knows it didn't work.
static void driverWallReset() {
  if (!resetHeadingOffBackWall())
    Controller.rumble("--");
}

// Driver control. Button choices are placeholders - change as you like.
//   Left stick Y  : forward / back      Right stick X : turn
//   R1 / R2       : lift up / down      L1 / L2       : claw close / open
//   Up / Down     : active toggle       A             : shift gear
//   Y             : wall reset
void usercontrol(void) {
  int loopCount = 0;
  while (true) {
    // Arcade drive.
    arcadeVolts(stickVolts(Controller.Axis3.position()),
                stickVolts(Controller.Axis1.position()));

    // Mechanisms: hold a button to move, release to hold position.
    spinMechanism(lift, buttonVolts(Controller.ButtonR1.pressing(),
                                    Controller.ButtonR2.pressing(),
                                    LIFT_VOLTS));
    spinMechanism(claw, buttonVolts(Controller.ButtonL1.pressing(),
                                    Controller.ButtonL2.pressing(),
                                    CLAW_VOLTS));
    spinMechanism(activeToggle, buttonVolts(Controller.ButtonUp.pressing(),
                                            Controller.ButtonDown.pressing(),
                                            TOGGLE_VOLTS));

    // Show the pose on the controller about 5 times a second (the
    // controller screen is slow, so don't update it every loop).
    if (++loopCount % 20 == 0) {
      Pose pose = getPose();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("%5.1f %5.1f %5.1f %s ", pose.x, pose.y,
                              pose.theta, getGear() == Gear::HIGH ? "H" : "L");
    }

    wait(LOOP_MS, msec);
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // One-press buttons (registered once here so they never double up).
  Controller.ButtonA.pressed(toggleGear);
  Controller.ButtonY.pressed(driverWallReset);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

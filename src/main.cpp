/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:                                                      */
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

/*----------------------------------------------------------------------------*/
/*                              SKILLS                                        */
/*                                                                            */
/*  Field points from the V5 Override manual v2.1, Appendix A ("Field        */
/*  Element Locations" / "Scoring Object Locations"), Robot Skills setup     */
/*  (RSC4: all Goals start empty). Origin = bottom-left corner in the        */
/*  audience view, which is the corner between the red Alliance Station and  */
/*  the audience. Red Alliance Station is the LEFT wall (x = 0).             */
/*----------------------------------------------------------------------------*/

// Back straight up `inches` (to clear a goal/wall before turning).
static void backUp(double inches) {
  Pose pose = getPose();
  double h = toRadians(pose.theta);
  moveToPoint(pose.x - inches * std::sin(h), pose.y - inches * std::cos(h),
              true, 1500);
}

static void openClaw() { moveMechanismTo(claw, CLAW_OPEN_DEG, CLAW_GAINS, 500); }
static void closeClaw() {
  moveMechanismTo(claw, CLAW_CLOSED_DEG, CLAW_GAINS, 500);
}
static void liftTo(double deg) {
  moveMechanismTo(lift, deg, LIFT_GAINS, 1500);
}

// Drive the robot's center to (cx, cy), aim the claw at the Pin+Cup at
// (ox, oy) and pick it up.
static void grabFromFloor(double cx, double cy, double ox, double oy) {
  liftTo(LIFT_GRAB_DEG);
  openClaw();
  moveToPoint(cx, cy);
  turnToPoint(ox, oy);
  closeClaw();
  liftTo(LIFT_CARRY_DEG);
}

// Drive the robot's center to (cx, cy), aim the claw at the goal at
// (gx, gy), which already has `level` Pins on it, and drop the held object.
static void placeOnGoal(double cx, double cy, double gx, double gy,
                        int level) {
  liftTo(LIFT_PLACE_BASE_DEG + level * LIFT_DEG_PER_LEVEL);
  moveToPoint(cx, cy);
  turnToPoint(gx, gy);
  openClaw();
  backUp(8.0);
  liftTo(LIFT_CARRY_DEG);
}

// Get a match load from the bottom-left red Loader (west wall, half a tile
// up: y = tiles(0.5) = 11.34). The robot's corners are 10.96" from its
// center (sqrt(9^2 + 6.25^2)), so it can't turn on the loader's line
// without a corner hitting the south wall (only 0.38" spare). Instead it
// turns west 14.5" off the wall, angles down into the loader, squares up
// (a small turn, so the corners barely move), and backs out north-east to
// (26, 16) where it has room to turn toward a goal.
static void getMatchLoad() {
  liftTo(LIFT_LOAD_DEG);
  openClaw();
  moveToPoint(tiles(1) + 6.89, tiles(0.5) + 3.16); // line-up (30, 14.5)
  turnToHeading(270.0);
  moveToPoint(3.74 + 10.5, tiles(0.5)); // loader is 3.74" off the wall
  turnToHeading(270.0); // square up so the claw is on the loader
  wait(LOAD_WAIT_MS, msec);
  closeClaw();
  moveToPoint(tiles(1) + 2.89, tiles(0.5) + 4.66, true); // back out (26, 16)
}

// Spin the active toggle to flip the Toggle in front of us to red, then
// back off (a Toggle touching a Robot doesn't count as set - SC4).
static void spinToggle() {
  activeToggle.spin(directionType::fwd, TOGGLE_FLIP_VOLTS, voltageUnits::volt);
  wait(TOGGLE_FLIP_MS, msec);
  activeToggle.stop(brakeType::coast);
  backUp(8.0);
}

// Skills route:
//   1. Flip the left Toggle to red.
//   2. Preload Pin onto red Goal 1 (level 1).
//   3. YY Pin+Cup closest to (0, 0) onto Goal 1 (level 2).
//   4. Second YY Pin+Cup onto Goal 1 (level 3).
//   5. Match loads onto Goal 1 until it is 5 tall.
//   6. Remaining match loads onto red Goal 2.
//   7. Flip the bottom Toggle to red, then park in the Midfield.
//
// Match loads (RSC4): 3 red/yellow Pins, 4 blue/yellow Pins, 7 Cups.
// Goal 1 gets 2 of them, the other 5 go on Goal 2.
//
// Everything is in inches. tiles(n) = inches to tile seam n (see TOC.hpp),
// so (tiles(1), tiles(2)) is "1 tile over, 2 tiles up". Game objects sit
// right on tile seams:
//   red Goal 1  (tiles(1), tiles(2))     red Goal 2  (tiles(2), tiles(1))
//   YY 1        (tiles(1), tiles(1))     YY 2        (tiles(2), tiles(2))
//   left Toggle (wall, tiles(3))         bottom Toggle (tiles(3), wall)
//
// Robot is 18" long x 12.5" wide. Every moveToPoint / first pair of numbers
// below is where the robot's CENTER goes, written as a tile seam +/- an
// offset in inches. For grabs and places, that center is 10.5" back from
// the object (9" half length + ~1.5" to the middle of the object in the
// claw), on the line the robot drives in on, so the claw ends on the
// object. Paths leave at least ~2" between the robot's sides and the goals.
static void skills() {
  // Start in the left Quadrant (RSC2) directly in front of the left Toggle
  // (left wall, y = tiles(3)), facing it with the active toggle almost
  // touching it, holding the red/yellow Preload Pin in the claw. Center x =
  // ~2" Toggle depth + 9" half length.
  setPose({11.0, tiles(3), 270.0});
  resetHeadingOffBackWall(); // make sure the robot is facing the wall
  closeClaw();

  // 1. Left Toggle -> red, without driving. Owns the yellow halves stacked
  //    on Goal 1. Backs up 8".
  spinToggle();

  // 2. Preload straight into empty red Goal 1, coming in from the north.
  int goal1Level = 0;
  placeOnGoal(tiles(1) - 1.81, tiles(2) + 10.34, tiles(1), tiles(2),
              goal1Level++);

  // 3. YY 1. Go around the WEST side of Goal 1 first (a straight line
  //    would drive through it), then come in from the north-west.
  moveToPoint(10.0, tiles(1.75));
  grabFromFloor(tiles(1) - 6.44, tiles(1) + 8.30, tiles(1), tiles(1));
  placeOnGoal(tiles(1) - 4.09, tiles(2) - 9.67, tiles(1), tiles(2),
              goal1Level++); // from the south

  // 4. YY 2, then back to Goal 1 from the east.
  grabFromFloor(tiles(2) - 9.18, tiles(2) - 5.09, tiles(2), tiles(2));
  placeOnGoal(tiles(1) + 9.89, tiles(2) - 3.51, tiles(1), tiles(2),
              goal1Level++);

  // 5. Match load loop on Goal 1. Each load must be a Cup with a Pin in it
  //    so it nests on the Pin below. Places from the south (coming from
  //    the loader back-out point (26, 16)).
  int loadsUsed = 0;
  while (goal1Level < 5 && loadsUsed < 7) {
    getMatchLoad();
    placeOnGoal(tiles(1) + 0.99, tiles(2) - 10.46, tiles(1), tiles(2),
                goal1Level++);
    loadsUsed++;
  }

  // 6. Rest of the match loads on red Goal 2 (bottom Quadrant), coming in
  //    from the south-west. The FIRST load here must be a bare Pin (a Cup
  //    in an empty goal isn't Placed - SC2), the rest are Cup+Pin.
  int goal2Level = 0;
  while (goal2Level < 5 && loadsUsed < 7) {
    getMatchLoad();
    placeOnGoal(tiles(2) - 9.93, tiles(1) - 3.42, tiles(2), tiles(1),
                goal2Level++);
    loadsUsed++;
  }

  // 7. Bottom Toggle (bottom wall, x = tiles(3)) -> red. Pass UNDER Goal 2
  //    (10" off the wall), line up square to the wall, and stop with the
  //    front ~2" off it (same as the start).
  liftTo(LIFT_GRAB_DEG); // lower the lift so the robot is stable
  moveToPoint(tiles(2), 10.0);
  moveToPoint(tiles(3), tiles(1) - 1.0);
  turnToHeading(180.0);
  moveToPoint(tiles(3), 11.0);
  spinToggle(); // backs up 8"

  // Park in the Midfield (+8), in the middle of the tile just outside the
  // Midfield's bottom-left edge. The front of the robot is over the
  // Midfield line while staying ~15" from the tall center Goal and ~16"
  // from the Pins on the diamond's corners.
  moveToPoint(tiles(2.5), tiles(2.5));
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

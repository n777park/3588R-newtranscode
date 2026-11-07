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
  // Where the robot starts: set this FIRST. Facing north, so the left
  // sensor sees the west wall (6" = sensor to robot center) and the back
  // sensor sees the south wall (9" = half the robot length).
  setPose({-FIELD_HALF + leftDistSensor.objectDistance(inches) + 6.0,
           -FIELD_HALF + backLeftDistSensor.objectDistance(inches) + 9.0,
           0.0});
  turnToPoint(-48.0, -24.0);
  moveToPoint(-48.0, -24.0);
  spinMechanism(lift, LIFT_VOLTS); // raise the lift for 0.5 s
  wait(500, msec);
  spinMechanism(lift, 0.0);
  turnToPoint(-24.0, -24.0);
  moveToPoint(-24.0, -24.0);
  // Claw Picks up
  turnToPoint(-48.0, -24.0);
  moveToPoint(-48.0, -24.0);
  // Claw Places down
  turnToPoint(-48.0, -48.0);
  moveToPoint(-48.0, -48.0);
  //Claw Picks up
  turnToPoint(-48.0, -24.0);
  moveToPoint(-48.0, -24.0);
  //Claw Places down
}

// Match auton starting on the right side (mirror of the left side).
static void matchRight() {
  // Facing north: right sensor sees the east wall, back sees the south wall.
  setPose({FIELD_HALF - rightDistSensor.objectDistance(inches) - 6.0,
           -FIELD_HALF + backRightDistSensor.objectDistance(inches) + 9.0,
           0.0});
  moveToPoint(FIELD_HALF - 24.0, -FIELD_HALF + 48.0);
  spinMechanism(lift, LIFT_VOLTS); // raise the lift for 0.5 s
  wait(500, msec);
  spinMechanism(lift, 0.0);
  turnToPoint(FIELD_HALF - 48.0, -FIELD_HALF + 48.0);
  moveToPoint(FIELD_HALF - 48.0, -FIELD_HALF + 48.0);
}

/*----------------------------------------------------------------------------*/
/*                              SKILLS                                        */
/*                                                                            */
/*  Field points from the V5 Override manual v2.1, Appendix A ("Field        */
/*  Element Locations" / "Scoring Object Locations"), Robot Skills setup     */
/*  (RSC4: all Goals start empty). Origin = field center. Red Alliance       */
/*  Station is the WEST wall (x = -FIELD_HALF), the audience is the SOUTH    */
/*  wall (y = -FIELD_HALF). The whole route is in the south-west corner.     */
/*----------------------------------------------------------------------------*/

// Back straight up `inches` (to clear a goal/wall before turning).
static void backUp(double inches) {
  Pose pose = getPose();
  double h = toRadians(pose.theta);
  moveToPoint(pose.x - inches * std::sin(h), pose.y - inches * std::cos(h),
              true, 1500);
}

// The claw is a ratchet claw on one 11W motor, banded shut:
//  - Spin IN:  the wheels spin and pull the object in; the bands keep the
//              jaws squeezed on it.
//  - Spin OUT: the ratchet locks the wheels, so the motor opens the jaws
//              instead and the object is let go.
//  - Stopped:  the bands close it by themselves (passive grip). It must
//              COAST when stopped - holding would keep the jaws open.
static void clawIntake() {
  claw.spin(directionType::fwd, CLAW_GRAB_VOLTS, voltageUnits::volt);
}
static void clawOpen() {
  claw.spin(directionType::rev, CLAW_RELEASE_VOLTS, voltageUnits::volt);
}
static void clawStop() { claw.stop(brakeType::coast); }

// Time-based lift. liftHeightMs is where the lift is, in "ms of lifting from
// the very bottom" (see LIFT_* in TOC.hpp). Skills starts with the lift all
// the way down, so it starts at 0.
static int liftHeightMs = 0;

// Run the lift up or down for the time it takes to get from the current
// height to targetMs, then hold it there. Going to the bottom (0) pushes
// down a little extra so the lift really bottoms out, which wipes out any
// timing error that has built up.
static void liftTo(int targetMs) {
  int diff = targetMs - liftHeightMs;
  if (diff > 0) {
    lift.spin(directionType::fwd, LIFT_TIMED_VOLTS, voltageUnits::volt);
    wait(diff, msec);
  } else if (diff < 0 || targetMs == 0) {
    int downMs = (int)(-diff * LIFT_DOWN_SCALE);
    if (targetMs == 0)
      downMs += LIFT_BOTTOM_EXTRA_MS;
    lift.spin(directionType::rev, LIFT_TIMED_VOLTS, voltageUnits::volt);
    wait(downMs, msec);
  }
  lift.stop(brakeType::hold);
  liftHeightMs = targetMs;
}

// Turn in place to face (x, y), THEN drive straight to it. Plain moveToPoint
// turns while it drives, which swings the robot's corners (10.96" from its
// center) through a wide arc. Every turn in the route happens at a spot
// with that much room; every drive is a straight line.
static void driveTo(double x, double y) {
  turnToPoint(x, y);
  moveToPoint(x, y);
}

// Drive the robot's center to (cx, cy), aim the claw at the Pin+Cup at
// (ox, oy) and pick it up. (cx, cy) is on the line from where the robot is
// to the object, so the last turn is tiny and the corners don't sweep the
// object over. The claw spins in the whole way so the wheels pull the
// object in as the robot reaches it, and keeps spinning while it's lifted
// off the floor, then stops (the bands hold it).
static void grabFromFloor(double cx, double cy, double ox, double oy) {
  liftTo(LIFT_GRAB_MS);
  clawIntake();
  driveTo(cx, cy);
  turnToPoint(ox, oy);
  liftTo(LIFT_CARRY_MS); // still spinning
  clawStop();
}

// Drive the robot's center to (cx, cy), aim the claw at the goal at
// (gx, gy), which already has `level` Pins on it, and let go of the held
// object: spin out to open the jaws, keep them open while backing away so
// they don't drag the stack, then stop and let the bands close them.
static void placeOnGoal(double cx, double cy, double gx, double gy,
                        int level) {
  liftTo(LIFT_PLACE_BASE_MS + level * LIFT_MS_PER_LEVEL);
  turnToPoint(gx, gy);
  driveTo(cx, cy);
  clawOpen();
  wait(CLAW_OPEN_MS, msec);
  backUp(8.0); // still open
  clawStop();
  liftTo(LIFT_CARRY_MS);
}

// Get a match load from the south-west red Loader: on the west wall, 3.74"
// out from it and 11.44" up from the south wall, so it's in the corner at
// (-66.46, -58.76). The robot's center has to be on the loader's line,
// y = -58.86, but it can't TURN on that line without a corner hitting the
// south wall (only 0.38" spare). Instead it turns west at a line-up point
// 14.5" off the south wall, angles down into the loader, squares up (a
// small turn, so the corners barely move), and backs out north-east to
// (-44.20, -54.20) where it has room to turn toward a goal.
//
// The claw spins in starting right before the robot drives in, and keeps
// spinning (the motor command stays on while the drive moves) through the
// wait and the first CLAW_SPIN_OUT_IN inches of backing out, so the load is
// pulled in the whole time. Then it stops and the bands hold the load.
static void getMatchLoad() {
  liftTo(LIFT_LOAD_MS);
  driveTo(-40.20, -55.70); // line-up
  turnToHeading(270.0);
  clawIntake();
  moveToPoint(-55.96, -58.86); // claw on the loader
  turnToHeading(270.0);        // square up so the claw is on the loader
  wait(LOAD_WAIT_MS, msec);
  backUp(CLAW_SPIN_OUT_IN); // still spinning
  clawStop();
  moveToPoint(-44.20, -54.20, true); // back out
}

// Spin the active toggle to flip the Toggle in front of us to red, then
// back off (a Toggle touching a Robot doesn't count as set - SC4).
static void spinToggle(float timeoutMs=500) {
  activeToggle.spin(directionType::fwd, TOGGLE_FLIP_VOLTS, voltageUnits::volt);
  wait(timeoutMs, msec);
  activeToggle.stop(brakeType::coast);
  backUp(8.0);
}

// Skills route:
//   1. Flip the west Toggle to red.
//   2. Preload Pin onto red Goal 1 (level 1).
//   3. YY Pin+Cup closest to the south-west corner onto Goal 1 (level 2).
//   4. Second YY Pin+Cup onto Goal 1 (level 3).
//   5. Match loads onto Goal 1 until it is 5 tall.
//   6. Remaining match loads onto red Goal 2.
//   7. Flip the south Toggle to red, then park in the Midfield.
//
// Match loads (RSC4): 3 red/yellow Pins, 4 blue/yellow Pins, 7 Cups.
// Goal 1 gets 2 of them, the other 5 go on Goal 2.
//
// Everything is in inches, (0, 0) = field center, walls at +/-70.2. Tiles
// are counted as 24" (really ~23.55", so objects 2 tiles out are ~0.9"
// closer to the center than the code thinks). Game objects sit where tiles
// meet:
//   red Goal 1  (-48, -24)          red Goal 2   (-24, -48)
//   YY 1        (-48, -48)          YY 2         (-24, -24)
//   west Toggle (-70.2, 0)          south Toggle (0, -70.2)
//   Loader      (-66.46, -58.76)    (see getMatchLoad)
// Points near the walls (start, loader, toggles) are measured from the
// real wall, not the 24" grid. Also in the way: Pin+Cups against the west
// wall at y = -24 and the south wall at x = -24 (they stick out ~3.2" from
// the wall), and Pins on the Midfield corners (-24, 0) and (0, -24).
//
// Robot is 18" long x 12.5" wide. Every point below is where the robot's
// CENTER goes. For grabs and places, that center is 10.5" back from the
// object (9" half length + ~1.5" to the middle of the object in the claw),
// on the line from the previous point, so the claw ends on the object.
//
// The middle of the tile between Goal 1, Goal 2, YY 1 and YY 2,
// (-36, -36), is the "hub": the only spot with room to turn that
// reaches all four.
static void skills() {
 
  // Facing the west wall, so the left sensor sees the south wall (6" =
  // sensor to robot center). x is fixed: no sensor faces the west wall.
  setPose({-FIELD_HALF + backLeftDistSensor.objectDistance(inches) + 9.0,
           -FIELD_HALF + leftDistSensor.objectDistance(inches) + 6.0,
           270.0});

  // 1. West Toggle -> red, without driving. Owns the yellow halves stacked
  //    on Goal 1. Backs up 8" to (-51.20, 0).
  spinToggle();

  // 2. Preload straight into empty red Goal 1, coming in from the north.
  int goal1Level = 0;
  placeOnGoal(-49.81, -13.66, -48.0, -24.0, goal1Level++);

  // 3. YY 1. Goal 1 is in the way and the gap between it and the wall
  //    Pin+Cups is too narrow, so go around its EAST side: across, then
  //    straight south down the middle of the tile column to the hub, then
  //    in diagonally from the north-east.
  driveTo(-36.0, -8.0);
  driveTo(-36.0, -36.0); // hub
  grabFromFloor(-40.58, -40.58, -48.0, -48.0);
  placeOnGoal(-43.61, -33.53, -48.0, -24.0, goal1Level++); // from the SE

  // 4. YY 2 (in from the south-west), then onto Goal 1 from the east.
  grabFromFloor(-31.20, -31.64, -24.0, -24.0);
  placeOnGoal(-38.49, -28.45, -48.0, -24.0, goal1Level++);

  // 5. Match load loop on Goal 1. Each load must be a Cup with a Pin in it
  //    so it nests on the Pin below. Places from the south (coming from
  //    the loader back-out point).
  int loadsUsed = 0;
  while (goal1Level < 5 && loadsUsed < 7) {
    getMatchLoad();
    placeOnGoal(-47.01, -34.46, -48.0, -24.0, goal1Level++);
    loadsUsed++;
  }

  // 6. Rest of the match loads on red Goal 2 (south Quadrant), coming in
  //    from the south-west. The FIRST load here must be a bare Pin (a Cup
  //    in an empty goal isn't Placed - SC2), the rest are Cup+Pin.
  int goal2Level = 0;
  while (goal2Level < 5 && loadsUsed < 7) {
    getMatchLoad();
    placeOnGoal(-33.93, -51.42, -24.0, -48.0, goal2Level++);
    loadsUsed++;
  }

  // 7. South Toggle (south wall, x = 0) -> red. Going SOUTH of Goal 2
  //    would scrape the south wall Pin+Cups, so go NORTH of it: back to
  //    the hub, east to just south of the Midfield corner Pin, then
  //    straight south to the toggle and stop with the front ~2" off the
  //    wall (same as the start).
  liftTo(LIFT_GRAB_MS); // lower the lift so the robot is stable
  driveTo(-36.0, -36.0); // hub
  driveTo(0.0, -38.0);
  turnToHeading(180.0);
  moveToPoint(0.0, -59.20);
  spinToggle(); // backs up 8" to (0, -51.20)

  // Park in the Midfield (+8): up the middle of the tile column west of
  // the south Midfield Pin, then north into the diamond (|x| + |y| <
  // 23.5). Ends with the front inside it, ~3.5" from the tall center Goal
  // and ~5" from the Midfield corner Pins.
  driveTo(-12.0, -36.0);
  driveTo(-12.0, -12.0);
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
  setGear(Gear::HIGH); // piston retracted = 400 rpm

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
//   R1 / R2       : lift up / down      L1 / L2       : claw intake / open
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
    // Claw: not spinMechanism, because the claw must COAST when released so
    // its bands can close it (holding would keep the jaws open).
    double clawVolts = buttonVolts(Controller.ButtonL1.pressing(),
                                   Controller.ButtonL2.pressing(), CLAW_VOLTS);
    if (clawVolts == 0.0)
      claw.stop(brakeType::coast);
    else
      claw.spin(directionType::fwd, clawVolts, voltageUnits::volt);
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

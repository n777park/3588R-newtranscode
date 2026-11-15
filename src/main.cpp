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

// The claw is a ratchet claw on one 11W motor, banded shut:
//  - Spin IN:  the wheels spin and pull the object in; the bands keep the
//              jaws squeezed on it.
//  - Spin OUT: the ratchet locks the wheels, so the motor opens the jaws
//              instead and the object is let go.
//  - Stopped:  the bands close it by themselves (passive grip). It must
//              COAST when stopped - holding would keep the jaws open.
// (Up here, above the autons, because C++ needs a function defined before
// the first code that calls it.)
// True from clawOpen() until the auto clamp grabs something - or until the
// auton calls clawIntake() / clawStop() itself, which cancels it.
static volatile bool clampArmed = false;

void clawIntake() {
  clampArmed = false;
  claw.spin(directionType::fwd, CLAW_GRAB_VOLTS, voltageUnits::volt);
}
void clawOpen() {
  clampArmed = true; // the auto clamp now waits for a pin (see below)
  claw.spin(directionType::rev, CLAW_RELEASE_VOLTS, voltageUnits::volt);
}
void clawStop() {
  clampArmed = false;
  claw.stop(brakeType::coast);
}

// The auto clamp is switched off until the brain's clock (ms) reaches this.
static volatile uint32_t autoClampOffUntilMs = 0;

// Turn the auto clamp OFF for `ms` milliseconds. USE IT WHEN RELEASING A
// PIN: releasing calls clawOpen(), which arms the auto clamp, and the pin is
// still right in front of the sensor - so without this it would grab the
// pin straight back. It turns itself back on afterwards.
//   disableAutoClamp(2000);  // off for 2 s
//   clawOpen();              // let the pin go
void disableAutoClamp(int ms) { autoClampOffUntilMs = timer::system() + ms; }

// Turn the auto clamp back on right away (ends a disableAutoClamp early).
void enableAutoClamp() { autoClampOffUntilMs = 0; }

// Auto clamp, AUTONOMOUS ONLY (background task started in pre_auton):
//   1. clawOpen() arms it - the jaws open and it starts watching.
//   2. Once clawDistSensor has seen something within CLAW_CLAMP_IN for
//      CLAW_CLAMP_CONFIRM_MS (a pin is in reach), it calls clawStop() then
//      clawIntake() to close on it. The claw keeps spinning in until the
//      auton calls clawStop().
//   3. Then it's disarmed until the next clawOpen().
// Calling clawIntake() or clawStop() yourself while it's armed cancels it.
// Does nothing while disableAutoClamp() has it switched off.
static int autoClampTask() {
  int closeMs = 0; // how long the pin has been close enough
  while (true) {
    bool close = clawDistSensor.isObjectDetected() &&
                 clawDistSensor.objectDistance(inches) <= CLAW_CLAMP_IN;
    bool enabled = timer::system() >= autoClampOffUntilMs;
    if (Competition.isAutonomous() && enabled && clampArmed && close) {
      closeMs += 10;
      if (closeMs >= CLAW_CLAMP_CONFIRM_MS) {
        clawStop();   // stop opening: the bands start closing the jaws
        clawIntake(); // spin in: the wheels pull the pin in and grip it
        closeMs = 0;
      }
    } else {
      closeMs = 0;
    }
    vex::wait(10, msec);
  }
  return 0;
}

/*============================================================================*/
/*                              AUTON ROUTINES                                */
/*    Placeholders that show how to use the motions - replace with real ones. */
/*============================================================================*/

// Match auton starting on the left side.
static void matchLeft() {
  // Where the robot starts: set this FIRST. Facing north, so the left
  // sensor sees the west wall (6" = sensor to robot center) and the back
  // sensor sees the south wall (9" = half the robot length).
  vex::wait(1000,msec);
  // clawOpen(); // open the claw so it can grab a pin
  
  // return;
  
  spinToHeight(10.0, 600); // raise the lift for 0.5 s
  // reset the inertial sensor to 0 deg (facing north)
  thread spinToggle = thread([]() {
    clawIntake(); // start the claw spinning in
    spinMechanism(activeToggle, TOGGLE_VOLTS); // start the toggle spinning in
    vex::wait(500, msec); // let it run for 3.0 s, then stop
    spinMechanism(activeToggle, 0.0);
    clawStop(); // stop the claw after 0.5 s
  });
  // vex::wait(100000,msec);
  // turnToPoint(-48.0, -24.0);
  // vex::wait(1000000,msec)
  vex::wait(400,msec);
  deaddriveforward(5.0, 600);
  setPose({
           -FIELD_HALF + backLeftDistSensor.objectDistance(inches) + 9.0,
           FIELD_HALF - leftDistSensor.objectDistance(inches) - 4.0,
           90.0});
  // return;
  resetHeadingOffBackWall(); 
  moveToPoint(-48.0, 0.0, 800);
  // turnToHeading(90.0, TurnDirection::SHORTEST, 800);
  // setPose({
  //          -FIELD_HALF + backLeftDistSensor.objectDistance(inches) + 9.0,
  //          FIELD_HALF - leftDistSensor.objectDistance(inches) - 4.0,
  //          getPose().theta}); // keep the heading it has now
  // return;
  turnToPoint(-48.0, -24.0, TurnDirection::SHORTEST,false, 800);
    clawStop(); // stop the claw after 0.5 s

  moveToPoint(-48.0, -17.0, 900);
  spinToHeight(6.0, 400);
  // return;

  disableAutoClamp(1000); // releasing - don't let the auto clamp re-grab it
  clawOpen();
  // Claw Picks up
  deaddriveforward(-7.0, 600);
  clawStop();
  turnToHeading(180.0, TurnDirection::SHORTEST, 800);
  { // reset x from the right distance sensor; keep the current y and heading
    Pose current = getPose();
    setPose({-FIELD_HALF + rightDistSensor.objectDistance(inches) + 4.0,
             current.y,
             current.theta});
  }
  spinToHeight(0.0, 600); // lower the lift for 0
  turnToPoint(-24.0, -24.0, TurnDirection::SHORTEST, false, 800);
  clawOpen();
  moveToPoint(-23.0, -24.0, 1300, false, 5.0); // move to the pin
  clawStop();
  clawIntake();
  spinToHeight(16.0, 1200); // raise the lift for 0.5 s
  clawStop();
  // Claw Places down
  turnToPoint(-48.0, -24.0, TurnDirection::SHORTEST, false, 800);
  moveToPoint(-48.0, -24.0,900);
  spinToHeight(13.0, 300); // lower the lift for 0.5 s
  disableAutoClamp(1000); // releasing - don't let the auto clamp re-grab it
  clawOpen();
  //Claw Picks up
  // turnToPoint(-24.0, -24.0);
  thread closeclaw = thread([]() {
    vex::wait(500, msec);
    clawStop();

  });
  deaddriveforward(-5.0, 600);
  spinToHeight(0.0, 1200); // lower the lift for 0.5 s
  turnToPoint(-48.0, -48.0);
  clawOpen();
  //Claw Places down
  moveToPoint(-46.0, -48.0, 900, false, 4.0);
  clawStop();

  clawIntake();
  spinToHeight(18.0, 1600); // raise the lift for
  clawStop();
  turnToPoint(-48.0, -24.0);
  moveToPoint(-48.0, -24.0);
  disableAutoClamp(1500); // releasing - don't let the auto clamp re-grab it
  clawOpen();
}
//bottom
static void matchBottom() {
    // Facing north: right sensor sees the east wall, back sees the south wall.
    vex::wait(1000,msec);
  setPose({FIELD_HALF - rightDistSensor.objectDistance(inches) - 4.0,
           -FIELD_HALF + backRightDistSensor.objectDistance(inches) + 9.0,
           0.0});
  thread spinToggle = thread([]() {
    spinMechanism(activeToggle, TOGGLE_VOLTS); // start the toggle spinning in
    vex::wait(500, msec); // let it run for 3.0 s, then stop
    spinMechanism(activeToggle, 0.0);
  });
  spinToHeight(8.0, 600); // raise the lift for 0.5 s
  
  // thread raiselift = thread([]() {
  //   runLiftFor(LIFT_VOLTS, 500); // raise the lift for 0.5 s
  // // // });
  // moveToPoint(-24.0, -48.0, 500);
  moveToPoint(-24.0, -48.0, 1000);
  spinToHeight(4.0, 600); // lower the lift for 0.5 s
  clawOpen();
  moveToPoint(-10.0, -48.0, 500);
  turnToPoint(-26.0,-68.0);
  clawStop();
  spinToHeight(0.0, 600); // lower the lift for 0.5 s
    clawIntake();
  moveToPoint(-26.0,-68.0, 800); //grab pin on wall
  clawStop();
  moveToPoint(-10.0,-48.0, 800);
  spinToHeight(15.0, 600); // raise the lift for 0.5 s
  turnToPoint(-24.0,-48.0);
  // thread lowerlift = thread([]() {
  //   vex::wait(300, msec);
  //   runLiftFor(-LIFT_VOLTS, 1000); // lower the lift for 1 s
  // });
  moveToPoint(-24,-48.0, 800);
  spinToHeight(10.0, 600); // lower the lift for 0.5 s
  clawOpen();
  wait(500,msec);
  clawStop();
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
              1500, true);
}

// Time-based lift. liftHeightMs is where the lift is, in "ms of lifting from
// the very bottom" (see LIFT_* in TOC.hpp). Skills starts with the lift all
// the way down, so it starts at 0.
static int liftHeightMs = 0;

// Run the lift up or down for the time it takes to get from the current
// height to targetMs, then hold it there. Going to the bottom (0) allows a
// little extra time so the lift really bottoms out, which wipes out any
// timing error that has built up - the hard stop (spinLift) stops it at
// 0 deg, so the extra time can't drive it below the bottom.
static void liftTo(int targetMs) {
  int diff = targetMs - liftHeightMs;
  if (diff > 0) {
    runLiftFor(LIFT_TIMED_VOLTS, diff);
  } else if (diff < 0 || targetMs == 0) {
    int downMs = (int)(-diff * LIFT_DOWN_SCALE);
    if (targetMs == 0)
      downMs += LIFT_BOTTOM_EXTRA_MS;
    runLiftFor(-LIFT_TIMED_VOLTS, downMs);
  } else {
    spinLift(0.0);
  }
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
  disableAutoClamp(3000); // releasing: don't let the auto clamp re-grab it
  clawOpen();
  vex::wait(CLAW_OPEN_MS, msec);
  backUp(8.0); // still open
  clawStop();
  enableAutoClamp(); // released and backed away - back on
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
  vex::wait(LOAD_WAIT_MS, msec);
  backUp(CLAW_SPIN_OUT_IN); // still spinning
  clawStop();
  moveToPoint(-44.20, -54.20, 4000, true); // back out
}

// Spin the active toggle to flip the Toggle in front of us to red, then
// back off (a Toggle touching a Robot doesn't count as set - SC4).
static void spinToggle(float timeoutMs=500) {
  activeToggle.spin(directionType::fwd, TOGGLE_FLIP_VOLTS, voltageUnits::volt);
  vex::wait(timeoutMs, msec);
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
 
  // Facing the west wall. x: the front sensor sees the west Toggle, which
  // sticks out ~2" from the wall. y: the left sensor sees the south wall.
  setPose({-FIELD_HALF + 2.0 + frontLeftDistSensor.objectDistance(inches) +
               frontSensorOffset,
           -FIELD_HALF + leftDistSensor.objectDistance(inches) +
               lateralSensorOffset,
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

/*----------------------------------------------------------------------------*/
/*                       PID TUNING TEST (runs as "Do Nothing")               */
/*                                                                            */
/*  6 turns (TURN_GAINS), then 2 drives (DRIVE_GAINS), with a pause after     */
/*  each. One line per step goes on the BRAIN screen:                         */
/*    move  = how far the step asked to go (+ = right / forward)              */
/*    time  = seconds until the motion said it was done                      */
/*    over  = how far it went PAST the target (0 = never passed it)           */
/*    final = where it ended after the pause: + = stopped short of the        */
/*            target, - = ended past it                                       */
/*  Turns are in degrees, drives in inches. Fits in the 15 s autonomous part  */
/*  of a timed run. Needs a clear spot ~3 ft long with room to spin.          */
/*----------------------------------------------------------------------------*/

static const int PID_TEST_PAUSE_MS = 750; // still time after each step

// Overshoot tracker: while pidSampling is true, a background task samples
// the robot every 10 ms and keeps the biggest distance it went PAST the
// current step's target (turns: degrees, drives: inches).
static volatile bool pidSampling = false;
static volatile bool pidSampleTurn = true;
static volatile double pidTargetHeading = 0.0; // turn: target heading
static volatile double pidTurnSign = 1.0;      // turn: +1 clockwise, -1 not
static volatile double pidTargetX = 0.0;       // drive: target point
static volatile double pidTargetY = 0.0;
static volatile double pidDirX = 0.0; // drive: unit vector, way of travel
static volatile double pidDirY = 1.0;
static volatile double pidOvershoot = 0.0;

static int pidSamplerTask() {
  while (true) {
    if (pidSampling) {
      Pose p = getPose();
      double past = 0.0;
      if (pidSampleTurn) {
        // Only near the target (within 90 deg) - a 180 turn starts right
        // at the +/-180 wrap, which would otherwise look like overshoot.
        double error = wrap180(pidTargetHeading - p.theta);
        if (std::fabs(error) < 90.0)
          past = -pidTurnSign * error;
      } else {
        past = (p.x - pidTargetX) * pidDirX + (p.y - pidTargetY) * pidDirY;
      }
      if (past > pidOvershoot)
        pidOvershoot = past;
    }
    vex::wait(10, msec);
  }
  return 0;
}

static int pidStepNum = 0; // step number, also picks the brain screen row

static void pidShow(const char *kind, double move, double ms, double over,
                    double final) {
  pidStepNum++;
  Brain.Screen.setCursor(pidStepNum + 2, 1); // rows 1-2 are the header
  Brain.Screen.print("%d %-5s %6.1f %5.2fs %5.1f %6.1f", pidStepNum, kind,
                     move, ms / 1000.0, over, final);
}

// One turn step: turn to `heading` the way `dir` says, pause, report.
static void pidTestTurn(double heading, TurnDirection dir) {
  // LONGEST: resolve it to the actual way round (the same choice
  // turnToHeading makes), so over/final are measured from the right side.
  if (dir == TurnDirection::LONGEST)
    dir = wrap180(heading - getPose().theta) > 0.0
              ? TurnDirection::COUNTERCLOCKWISE
              : TurnDirection::CLOCKWISE;
  double cw = wrap360(heading - getPose().theta); // clockwise distance
  double move = dir == TurnDirection::CLOCKWISE ? cw
                : dir == TurnDirection::COUNTERCLOCKWISE
                    ? (cw == 0.0 ? 0.0 : cw - 360.0)
                    : wrap180(heading - getPose().theta);
  pidTargetHeading = heading;
  pidTurnSign = move >= 0.0 ? 1.0 : -1.0;
  pidSampleTurn = true;
  pidOvershoot = 0.0;
  pidSampling = true;

  timer t;
  turnToHeading(heading, dir, 3000); // 3 s: room for a 270 deg long turn
  double ms = t.time(msec);
  vex::wait(PID_TEST_PAUSE_MS, msec); // still sampling: coasting past counts too
  pidSampling = false;

  double final = pidTurnSign * wrap180(heading - getPose().theta);
  pidShow("turn", move, ms, pidOvershoot, final);
}

// One drive step: straight forward (+) or backward (-) `inches`, pause,
// report.
static void pidTestDrive(double inches) {
  Pose start = getPose();
  double h = toRadians(start.theta);
  double fx = std::sin(h), fy = std::cos(h); // robot forward
  double sign = inches >= 0.0 ? 1.0 : -1.0;
  pidTargetX = start.x + inches * fx;
  pidTargetY = start.y + inches * fy;
  pidDirX = sign * fx;
  pidDirY = sign * fy;
  pidSampleTurn = false;
  pidOvershoot = 0.0;
  pidSampling = true;

  timer t;
  moveToPoint(pidTargetX, pidTargetY, 4000, inches < 0.0);
  double ms = t.time(msec);
  vex::wait(PID_TEST_PAUSE_MS, msec); // still sampling: coasting past counts too
  pidSampling = false;

  Pose end = getPose();
  double final =
      (pidTargetX - end.x) * pidDirX + (pidTargetY - end.y) * pidDirY;
  pidShow("drive", inches, ms, pidOvershoot, final);
}

// Turn/drive PID test (see "PID TUNING TEST" above). Runs from doNothing().
static void driveTurnTest() {
  // (No fixed wait needed: autonomous() already waits until odometry runs.)
  setPose({0.0, 0.0, 0.0}); // "north" = whichever way the robot faces now
  vex::wait(1000, msec); // let the odometry settle before starting
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("# type    move   time  over  final");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("  (turns: degrees, drives: inches)");
  pidStepNum = 0;
  static bool samplerStarted = false;
  if (!samplerStarted) {
    thread sampler(pidSamplerTask);
    samplerStarted = true;
  }

  // Turns (TURN_GAINS)
  pidTestTurn(90.0, TurnDirection::LONGEST);         // 1: 90 right
  // pidTestTurn(0.0, TurnDirection::SHORTEST);          // 2: 90 left
  // pidTestTurn(180.0, TurnDirection::CLOCKWISE);       // 3: 180 right
  // pidTestTurn(0.0, TurnDirection::COUNTERCLOCKWISE);  // 4: 180 left
  // pidTestTurn(15.0, TurnDirection::SHORTEST);         // 5: 15 right
  // pidTestTurn(0.0, TurnDirection::SHORTEST);          // 6: 15 left

  // // Drives (DRIVE_GAINS)
  // pidTestDrive(48.0);  // 7: 24" forward
  // turnToHeading(0.0); // face north again
  // vex::wait(1000, msec);
  // pidTestDrive(-48.0); // 8: 24" back

  // The gains this run used, so a photo of the screen says what was tested.
  Brain.Screen.setCursor(pidStepNum + 4, 1);
  Brain.Screen.print("turn  kP %.2f kD %.3f", TURN_GAINS.kP, TURN_GAINS.kD);
  Brain.Screen.setCursor(pidStepNum + 5, 1);
  Brain.Screen.print("drive kP %.2f kD %.3f", DRIVE_GAINS.kP, DRIVE_GAINS.kD);
}

/*----------------------------------------------------------------------------*/
/*                       LIFT TEST (runs from doNothing)                      */
/*                                                                            */
/*  spinToHeight up and down through the lift's range, with a pause after     */
/*  each move. The robot doesn't drive. One line per step on the BRAIN        */
/*  screen:                                                                   */
/*    target = height asked for (inches)                                      */
/*    time   = seconds until spinToHeight said it was done (2.00 = timed out) */
/*    final  = height after the pause (shows sagging or drifting)            */
/*    err    = target - final: + = ended low, - = ended high                  */
/*  START THE PROGRAM WITH THE LIFT ALL THE WAY DOWN (that sets 0).           */
/*----------------------------------------------------------------------------*/

static void liftTestStep(double target) {
  timer t;
  spinToHeight(target);
  double ms = t.time(msec);
  vex::wait(PID_TEST_PAUSE_MS, msec); // watch it hold
  double final = liftHeightInches();

  pidStepNum++;
  Brain.Screen.setCursor(pidStepNum + 1, 1); // row 1 is the header
  Brain.Screen.print("%d %6.1f %5.2fs %6.2f %6.2f", pidStepNum, target,
                     ms / 1000.0, final, target - final);
}

static void liftTest() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("# target  time   final    err");
  pidStepNum = 0;

  spinToHeight(30.0); // start at the bottom, so 0 = bottom
  
  // liftTestStep(12.0);           // 1: up partway
  // liftTestStep(24.0);           // 2: up more
  // liftTestStep(LIFT_TOP_IN);    // 3: all the way up (36")
  // liftTestStep(12.0);           // 4: back down a long way
  // liftTestStep(LIFT_BOTTOM_IN); // 5: all the way down (2")

  // // The gains this run used, so a photo of the screen says what was tested.
  // Brain.Screen.setCursor(pidStepNum + 3, 1);
  // Brain.Screen.print("lift kP %.3f kI %.3f kD %.4f", LIFT_GAINS.kP,
  //                    LIFT_GAINS.kI, LIFT_GAINS.kD);
}

/*----------------------------------------------------------------------------*/
/*                    AUTO CLAMP TEST (runs from doNothing)                   */
/*                                                                            */
/*  Opens the claw (which arms the auto clamp), then for 12 s shows on the    */
/*  BRAIN screen what the claw sensor sees and whether the clamp fired.       */
/*  Hold a pin in front of the claw and watch:                                */
/*    - "sensor" shows the distance it reads (or NOTHING if it sees nothing)  */
/*    - "CLAMPED" appears when the auto clamp closes the claw                 */
/*  If it reads a distance but never clamps, set CLAW_CLAMP_IN (TOC.hpp) a     */
/*  little above that distance.                                               */
/*----------------------------------------------------------------------------*/

static void clawTest() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("AUTO CLAMP TEST - hold a pin in front");
  clawOpen(); // arms the auto clamp
  bool clamped = false;

  timer t;
  while (t.time(msec) < 12000) {
    Brain.Screen.setCursor(3, 1);
    if (clawDistSensor.isObjectDetected())
      Brain.Screen.print("sensor: %5.2f in          ",
                         clawDistSensor.objectDistance(inches));
    else
      Brain.Screen.print("sensor: NOTHING           ");
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("clamps at <= %.2f in", CLAW_CLAMP_IN);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("armed: %s   ", clampArmed ? "YES" : "no");

    // The clamp disarms itself when it fires - that's how we know.
    if (!clamped && !clampArmed) {
      clamped = true;
      Brain.Screen.setCursor(7, 1);
      Brain.Screen.print("CLAMPED at %.1f s", t.time(msec) / 1000.0);
    }
    vex::wait(50, msec);
  }
  clawStop();
}

// NOT a do-nothing anymore: runs a test. Pick a real auton on the brain
// before a match.
static void doNothing() {
  clawTest();
  // liftTest();       // the lift PID test
  // driveTurnTest();  // the turn/drive PID test
}

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
    "Match Bottom", // 1
    "Skills",      // 2
    "Do Nothing",  // 3
    "Test",        // 4
};
static const int AUTON_COUNT = sizeof(AUTON_NAMES) / sizeof(AUTON_NAMES[0]);

static const int SCREEN_WIDTH = 480;  // V5 brain screen size in pixels
static const int SCREEN_HEIGHT = 272;
static const int ROW_HEIGHT = SCREEN_HEIGHT / AUTON_COUNT;

// Index into AUTON_NAMES: the DEFAULT auton.
// A TIMED RUN ALWAYS RUNS THIS ONE: it restarts the program and starts
// autonomous as soon as the IMU finishes calibrating - the same moment the
// selector appears - so there's no time to tap a different one. To test an
// auton with a timed run, set its number here. (In a real match there's time
// to tap one on the brain before the match starts.)
//   0 = Match Left, 1 = Match Bottom, 2 = Skills, 3 = Do Nothing (tests),
//   4 = Test
static int selectedAuton = 0;

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

// Set at the end of pre_auton, once the IMU is calibrated and odometry is
// running. VEXcode starts autonomous the moment a timed run (or the field)
// enables it - it does NOT wait for pre_auton like PROS waits for
// initialize() - so autonomous() waits for this instead.
static volatile bool robotReady = false;

// Background task: shows the odometry pose on the controller screen the
// whole time - while the IMU calibrates, during autonomous, and during
// driver control - so it's visible during a timed run too. The controller
// screen is slow and drops updates if it's sent too much, so it writes one
// line every 100 ms.
//   Line 1: X: <x>  Y: <y>    (inches, field center = 0, 0)
//   Line 2: H: <heading>  <gear>  (degrees, clockwise, 0 = north)
//   Line 3: Lift: <in> <deg>      (height above the bottom, inches and
//                                  lift encoder degrees)
// Until pre_auton finishes, line 1 says "Calibrating IMU" instead.
static int controllerDisplayTask() {
  Controller.Screen.clearScreen();
  int line = 1; // which line to update this time (1, 2, 3, 1, ...)
  while (true) {
    if (!robotReady) {
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Calibrating IMU  ");
      vex::wait(100, msec);
      continue;
    }
    Pose pose = getPose();
    Controller.Screen.setCursor(line, 1);
    if (line == 1)
      Controller.Screen.print("X:%6.1f Y:%6.1f  ", pose.x, pose.y);
    else if (line == 2)
      Controller.Screen.print("H:%6.1f  %s     ", pose.theta,
                              getGear() == Gear::HIGH ? "400" : "200");
    else // lift height (inches, then degrees for LIFT_DEG_PER_INCH)
      Controller.Screen.print("Lift:%5.1fin %5.0fd", liftHeightInches(),
                              lift.position(rotationUnits::deg));
    line = line % 3 + 1;
    vex::wait(100, msec);
  }
  return 0;
}

// Runs once at power-on. Calibrate sensors, start odometry, show selector.
void pre_auton(void) {
  // Lift hard stop: wherever the lift is right now is 0 deg, and it can
  // never be lowered below that - so START THE PROGRAM WITH THE LIFT ALL THE
  // WAY DOWN. Done first, before the ~2 s IMU calibration, so the lift
  // can't have moved yet.
  lift.resetPosition();

  // Start the controller display first, so it's already running if a timed
  // run starts autonomous while the IMU is still calibrating.
  thread displayThread(controllerDisplayTask);
  // Auto clamp: only does anything during autonomous (see autoClampTask).
  thread clampThread(autoClampTask);

  setGear(Gear::HIGH); // piston retracted = 400 rpm

  // The robot must sit still while the inertial sensor calibrates (~2 s).
  // The short wait gives isCalibrating() time to turn true, so the loop
  // can't exit before calibration has actually started.
  imu.calibrate();
  vex::wait(100, msec);
  while (imu.isCalibrating())
    vex::wait(20, msec);

  startOdometry();
  robotReady = true; // autonomous can start now

  drawSelector();
  Brain.Screen.pressed(onScreenPressed);
}

// Run whichever auton is selected on the brain screen.
// Case numbers must match the positions in AUTON_NAMES.
void autonomous(void) {
  // A timed run starts autonomous the moment the program starts, while
  // pre_auton is still calibrating the IMU (~2 s). Wait for it to finish:
  // otherwise the auton drives with no odometry running, and
  // startOdometry() then resets the pose to (0, 0) partway through. In a
  // real match the program is started before the match, so this wait is 0.
  thread displayThread(controllerDisplayTask);
  while (!robotReady)
    vex::wait(10, msec);

  // Show which auton is running, so it's obvious if the wrong one started.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  if (selectedAuton >= 0 && selectedAuton < AUTON_COUNT)
    Brain.Screen.print("AUTON: %s", AUTON_NAMES[selectedAuton]);

  switch (selectedAuton) {
  case 0:
    matchLeft();
    break;
  case 1:
    matchBottom();
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

// Driver control.
//   Left stick Y  : forward / back      Right stick X : turn
//   R1 / R2       : raise / lower lift  L1 / L2       : open / spin claw
//   A             : toggle drive gear (200 <-> 400 rpm)
//   Up            : heading lock on / off
//   Y / X (hold)  : spin active toggle one way / the other way
//   B             : wall reset
void usercontrol(void) {
  // Heading lock state.
  double targetHeading = imu.rotation(rotationUnits::deg);
  int settleMs = 0;     // time left before the lock re-engages
  bool lockOn = true;   // toggled with Up
  bool upWasPressed = false;

  while (true) {
    // Arcade drive.
    double forward = stickVolts(Controller.Axis3.position());
    double turn = stickVolts(Controller.Axis1.position());

    // Up toggles the heading lock (on the press, not while held).
    bool upPressed = Controller.ButtonUp.pressing();
    if (upPressed && !upWasPressed)
      lockOn = !lockOn;
    upWasPressed = upPressed;

    // Heading lock: while the driver isn't turning, hold the heading they
    // left off at. After a turn, wait a moment for the robot to stop
    // rotating, then lock onto wherever it ended up.
    double heading = imu.rotation(rotationUnits::deg);
    if (turn != 0.0)
      settleMs = HEADING_LOCK_SETTLE_MS;
    if (!lockOn || !imu.installed()) {
      // Lock off or IMU unplugged: plain arcade, re-lock from scratch later.
      settleMs = HEADING_LOCK_SETTLE_MS;
    } else if (settleMs > 0) {
      settleMs -= LOOP_MS;
      targetHeading = heading;
    } else {
      double error = targetHeading - heading;
      if (std::fabs(error) > HEADING_LOCK_DEADBAND)
        turn = HEADING_LOCK_KP * error;
    }
    arcadeVolts(forward, turn);

    // Lift: R1 up, R2 down, always full power (LIFT_VOLTS). With no button
    // it holds (coast would let it fall back). spinLift is the hard stop:
    // R2 does nothing once the lift is back down at 0 deg.
    spinLift(buttonVolts(Controller.ButtonR1.pressing(),
                         Controller.ButtonR2.pressing(), LIFT_VOLTS));

    // Claw: L2 spins it in (wheels pull the object in), L1 spins it out
    // (ratchet locks the wheels, so the jaws open). Not spinMechanism,
    // because the claw must COAST when released so its bands can close it
    // (holding would keep the jaws open).
    if (Controller.ButtonL1.pressing()) {
      claw.spin(directionType::rev, CLAW_RELEASE_VOLTS, voltageUnits::volt);
      claw.setStopping(coast);
    }
    else if (Controller.ButtonL2.pressing()){
      claw.spin(directionType::fwd, CLAW_GRAB_VOLTS, voltageUnits::volt);
      claw.setStopping(coast);
    }
    else
      claw.stop(); // let the bands close it
    // Active toggle: Y spins it forward (the way auton's spinToggle does), X
    // spins it the other way, and it coasts when neither is held.
    double toggleVolts = buttonVolts(Controller.ButtonY.pressing(),
                                     Controller.ButtonX.pressing(),
                                     TOGGLE_VOLTS);
    if (toggleVolts == 0.0)
      activeToggle.stop(brakeType::coast);
    else
      activeToggle.spin(directionType::fwd, toggleVolts, voltageUnits::volt);

    // (The pose is shown on the controller by controllerDisplayTask, which
    // runs the whole time - see pre_auton.)

    vex::wait(10, msec);
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // One-press buttons (registered once here so they never double up).
  Controller.ButtonA.pressed(toggleGear);
  Controller.ButtonB.pressed(driverWallReset); // (Y spins the toggle)

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    vex::wait(100, msec);
  }
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       EKF.cpp                                                   */
/*    Description:  Extended Kalman Filter for robot position.                */
/*                                                                            */
/*    The filter keeps a best guess of the robot's state [x, y, theta] AND    */
/*    how unsure it is about that guess (the covariance matrix P).            */
/*                                                                            */
/*      predict()        - drive encoders say "we moved this much". The      */
/*                         guess moves, and uncertainty grows.               */
/*      updateHeading()  - inertial sensor says "heading is this". The       */
/*                         guess is pulled toward it, weighted by how much   */
/*                         we trust each source, and uncertainty shrinks.    */
/*                                                                            */
/*    Internally theta is stored in radians. Everything outside this file     */
/*    uses degrees (see Pose in TOC.hpp).                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "TOC.hpp"

#include <cmath>

// Index of each value inside the state vector.
static const int X = 0;
static const int Y = 1;
static const int THETA = 2;

// Jump straight to a known pose and say we are 100% sure of it.
void EKF::reset(const Pose &pose) {
  state[X] = pose.x;
  state[Y] = pose.y;
  state[THETA] = toRadians(pose.theta);

  // Zero covariance = no uncertainty. Noise is added back as we drive.
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      P[i][j] = 0.0;
}

// Prediction step: move the estimate using how far each side's wheels went.
void EKF::predict(double dLeft, double dRight) {
  // Distance the robot center moved, and how much it turned (clockwise +).
  double distance = (dLeft + dRight) / 2.0;
  double dTheta = (dLeft - dRight) / TRACK_WIDTH;

  // Use the heading halfway through the step: much more accurate on arcs.
  double midTheta = state[THETA] + dTheta / 2.0;
  double s = std::sin(midTheta);
  double c = std::cos(midTheta);

  // Move the state. Heading 0 = +Y and clockwise, so x uses sin, y uses cos.
  state[X] += distance * s;
  state[Y] += distance * c;
  state[THETA] += dTheta;

  // F = Jacobian of the motion model: how a small heading error turns into
  // position error. (Rows/cols are x, y, theta.)
  double F[3][3] = {
      {1, 0, distance * c},
      {0, 1, -distance * s},
      {0, 0, 1},
  };

  // FP = F * P
  double FP[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      FP[i][j] = 0.0;
      for (int k = 0; k < 3; k++)
        FP[i][j] += F[i][k] * P[k][j];
    }

  // P = FP * F^T  (spreads existing uncertainty through the motion)
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      P[i][j] = 0.0;
      for (int k = 0; k < 3; k++)
        P[i][j] += FP[i][k] * F[j][k];
    }

  // P += Q  (new uncertainty from this step: more driving = less sure)
  P[X][X] += EKF_Q_XY_PER_IN * std::fabs(distance) + EKF_Q_MIN;
  P[Y][Y] += EKF_Q_XY_PER_IN * std::fabs(distance) + EKF_Q_MIN;
  P[THETA][THETA] += EKF_Q_THETA_PER_RAD * std::fabs(dTheta) + EKF_Q_MIN;
}

// Correction step: blend in a heading measurement from the inertial sensor.
void EKF::updateHeading(double measuredHeadingDeg) {
  // Innovation = measurement - prediction, wrapped so 359 vs 1 is 2 degrees.
  double innovation =
      toRadians(wrap180(measuredHeadingDeg - toDegrees(state[THETA])));

  // S = how uncertain the innovation is (our uncertainty + sensor noise).
  double S = P[THETA][THETA] + EKF_R_IMU;

  // K = Kalman gain: how much each state value should move toward the
  // measurement. Heading measurements also fix x/y if they are correlated.
  double K[3];
  for (int i = 0; i < 3; i++)
    K[i] = P[i][THETA] / S;

  // Move the state toward the measurement.
  for (int i = 0; i < 3; i++)
    state[i] += K[i] * innovation;

  // Shrink the uncertainty: P = (I - K H) P, where H picks out theta.
  double thetaRow[3] = {P[THETA][X], P[THETA][Y], P[THETA][THETA]};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      P[i][j] -= K[i] * thetaRow[j];
}

// Current best estimate, converted to the degrees used everywhere else.
Pose EKF::getPose() const {
  return {state[X], state[Y], wrap360(toDegrees(state[THETA]))};
}

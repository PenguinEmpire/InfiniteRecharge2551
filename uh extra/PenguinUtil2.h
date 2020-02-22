#pragma once

#include <math.h>
#include <wpi/math>

#include <units/units.h>
#include "frc/geometry/Rotation2d.h"

namespace PenguinUtil2 {
  double linearMap(double n, double start1, double stop1, double start2, double stop2);

  double deadband(double input, double deadband = 0.05);
  double tooSmartDeadband(double input, double deadbandStart, double deadbandEnd, double leftRangeStart, double leftRangeEnd, double valueInDeadband, double rightRangeStart, double rightRangeEnd);
  double smartDeadband(double input, double deadbandStart, double deadbandEnd, double valueAtDeadBandEdge = 0.1);
  
  double PI;
  double TWO_PI;
  units::radian_t PI_RAD;
  units::radian_t TWO_PI_RAD;
  frc::Rotation2d ZERO_ROT;
  frc::Rotation2d PI_ROT;
  frc::Rotation2d TWO_PI_ROT;
} // PenguinUtil namespace
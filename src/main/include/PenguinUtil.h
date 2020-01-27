#pragma once

#include <math.h>

namespace PenguinUtil {
  double linearMap(double n, double start1, double stop1, double start2, double stop2) {
    return (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
  }

  double deadband(double input, double deadband = 0.05) {
    if (fabs(input < deadband)) {
      return 0;
    } else {
      return input;
    }
  }

  double tooSmartDeadband(double input, double deadbandStart, double deadbandEnd, double leftRangeStart, double leftRangeEnd, double valueInDeadband, double rightRangeStart, double rightRangeEnd) {
    if (deadbandStart <= input && input <= deadbandEnd) {
      return valueInDeadband;
    } else if (-1 <= input && input < deadbandStart) {
      return linearMap(input, -1, deadbandStart, leftRangeStart, leftRangeEnd);
    } else if (deadbandEnd < input && input <= 1) {
      return linearMap(input, deadbandEnd, 1, rightRangeStart, rightRangeEnd);
    } else {
      return 0.0;
      printf("Line %i: !!! Invalid input (abs(input) > 1) being passed to `tooSmartDeadband`. Returned 0.", __LINE__);
    }
  }

  double smartDeadband(double input, double deadbandStart, double deadbandEnd, double valueAtDeadBandEdge) {
    return tooSmartDeadband(input, deadbandStart, deadbandEnd, -1, -valueAtDeadBandEdge, 0, valueAtDeadBandEdge, 1);
  }

}
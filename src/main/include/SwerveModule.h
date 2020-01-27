/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/geometry/Translation2d.h"

class SwerveModule {
 public:
  SwerveModule(frc::Translation2d pos);


 private:
  frc::Translation2d modulePosition;
};

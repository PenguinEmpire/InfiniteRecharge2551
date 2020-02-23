/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "units/units.h"

#include "frc/controller/PIDController.h"

#include "Limelight.h"
  
class LimelightAutonomous {
 public:
  LimelightAutonomous(Limelight* limelight);
  ~LimelightAutonomous();

  Limelight* m_limelight;

  frc2::PIDController m_pidController{1, 0, 0};

  units::radians_per_second_t Run(units::radian_t currentAngle);
};

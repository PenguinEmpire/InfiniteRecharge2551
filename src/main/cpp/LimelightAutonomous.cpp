/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LimelightAutonomous.h"

LimelightAutonomous::LimelightAutonomous(Limelight* limelight)
  : m_limelight{limelight} {
    m_pidController.SetSetpoint(0);
    m_pidController.SetTolerance(2); // degrees
}

LimelightAutonomous::~LimelightAutonomous() {
  m_pidController.~PIDController();
}

units::radians_per_second_t LimelightAutonomous::Run(units::radian_t currentAngle) {
  const LimelightValues vals = m_limelight->GetInfo();

  return units::radians_per_second_t(
    m_pidController.Calculate(vals.tx)
  );
}

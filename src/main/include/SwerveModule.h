/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>

#include "frc/geometry/Translation2d.h"

#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"

#include "TurnEncoder.h"

class SwerveModule {
 public:
  SwerveModule(frc::Translation2d pos,
               int analogEncoderPort,
               units::radian_t analogEncoderOffset,
               int driveMotorCANID,
               int turnMotorCANID);

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turnMotor;

  double GetDriveDistance() {
    return m_driveEncoder.GetPosition();
  }
  double GetDriveVelocity() {
    return m_driveEncoder.GetVelocity();
  }
  units::radian_t GetAngle() {
    return m_turnEncoder.getAngle_SDS();
  }
  

 private:
  TurnEncoder m_turnEncoder;
  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

  frc::Translation2d m_modulePosition;

  // copied from SDS
  const double DRIVE_REDUCTION = 8.31 / 1.0; // (gear ratio)
  const double WHEEL_DIAMETER = 4.0; // (in)

};
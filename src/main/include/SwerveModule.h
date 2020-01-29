/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <units/units.h>

#include "frc/geometry/Translation2d.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>

#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"

#include "TurnEncoder.h"
#include "SwerveModuleName.h"

class SwerveModule {
 public:
  SwerveModule(frc::Translation2d pos,
               int analogEncoderPort,
               units::radian_t analogEncoderOffset,
               int driveMotorCANID,
               int turnMotorCANID,
               SwerveModuleName moduleName);

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turnMotor;

  /**
   * This is called `setTargetVelocity` in the SDS code (which I use the algorithm from) and `SetDesiredState` in the WPILib code. Sets member variables SDS_targetSpeed and SDS_targetAngle.
   * @param state: the desired frc::SwerveModuleState
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  double GetDriveDistance() {
    return m_driveEncoder.GetPosition();
  }
  double GetDriveVelocity() {
    return m_driveEncoder.GetVelocity();
  }
  units::radian_t GetAngle() {
    return m_turnEncoder.GetAngle_SDS();
  }

  units::radian_t GetCurrentAngle();
  void PutDiagnostics();
  void UpdateState();
  void SDS_UpdateSensors();

  SwerveModuleName m_moduleName;


  double SDS_targetSpeed;
  double SDS_targetAngle;

 private:
  TurnEncoder m_turnEncoder;
  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

  frc2::PIDController m_turnPIDController{
    0.5, 0.0, 0.0001
  };

  frc::Translation2d m_modulePosition;

  // copied from SDS
  const double DRIVE_REDUCTION = 8.31 / 1.0; // (gear ratio)
  const double WHEEL_DIAMETER = 4.0; // (in)
  const double DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * wpi::math::pi)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);

};
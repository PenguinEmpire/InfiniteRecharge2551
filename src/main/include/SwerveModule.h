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
#include "frc/Vector2d.h"
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
  double SDS_currentAngle;




  TurnEncoder m_turnEncoder;
  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

  frc2::PIDController m_turnPIDController{
    0.5, 0.0, 0.0001
  };

  frc::Translation2d m_modulePosition;

  // SDS stuff
  frc::Vector2d SDS_modulePosition;
  units::radian_t SDS_currentAngle2 = 0_rad;
  units::inch_t SDS_currentDistance = 0_in;
  units::meters_per_second_t SDS_targetSpeed2 = 0_mps;
  units::radian_t SDS_targetAngle2 = 0_rad;
  frc::Vector2d SDS_currentPosition{0, 0};
  units::inch_t SDS_previousDistance;
  units::radian_t SDS_ReadAngle() {
    return m_turnEncoder.GetAngle_SDS();
  }
  double SDS_ReadDistance() { // see `SwerveModule::SDS_UpdateSensors()`
    return m_driveEncoder.GetPosition();
  }
  void SDS_SetTargetAngle(units::radian_t angle);
  frc2::PIDController angleMotorController{1.5, 0.0, 0.5};
  void SDS_SetDriveOutput(double output) {
    m_driveMotor.Set(output);
  }
  frc::Vector2d SDS_GetModulePosition() {
    return SDS_modulePosition;
  }
  units::radian_t SDS_GetCurrentAngle() {
    return SDS_currentAngle2;
  }
  units::inch_t SDS_GetCurrentDistance() {
    return SDS_currentDistance();
  }
  units::meters_per_second_t SDS_GetCurrentVelocity() {
    return SDS_velocity;
  }
  units::current::ampere_t SDS_GetDriveCurrent() {
    return SDS_currentDraw;
  }
  frc::SwerveModuleState SDS_GetTargetVelocity() {
    frc::SwerveModuleState ret;
    ret.angle = SDS_targetAngle2;
    ret.speed = SDS_targetSpeed2;
    return ret;
  }
  // frc::Vector2d SDS_GetCurrentPosition(); // AFAICT used exactly zero places
  // void SDS_ResetKinematics(); // Also not used. Set SDS_currentPosition to the <0, 0> vector
  







  // copied from SDS
  const double DRIVE_REDUCTION = 8.31 / 1.0; // (gear ratio)
  const double WHEEL_DIAMETER = 4.0; // (in)
  const double DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * wpi::math::pi)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);

};
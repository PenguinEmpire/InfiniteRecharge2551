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
#include "frc/geometry/Rotation2d.h"
#include "frc/drive/Vector2d.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>

#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "rev/CANPIDController.h"

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

  /** This is called `setTargetVelocity` in the SDS code (which I use the algorithm from) and `SetDesiredState` in the WPILib code. Sets member variables SDS_targetSpeed and SDS_targetAngle.
   * @param state: the desired frc::SwerveModuleState
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  double GetDriveDistance() {
    return m_driveEncoder.GetPosition();
  }
  double GetDriveVelocity() {
    return m_driveEncoder.GetVelocity();
  }

  units::radian_t GetCurrentAngle();
  void PutDiagnostics();
  void SDS_UpdateSensors();

  SwerveModuleName m_moduleName;

  TurnEncoder m_turnEncoder;
  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

  frc::Translation2d m_modulePosition;

  // SDS stuff
  frc::Vector2d SDS_modulePosition;
  units::radian_t SDS_currentAngle2 = 0_rad;
  // units::inch_t SDS_currentDistance = 0_in;
  double SDS_currentDistance2 = 0;
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
  // void SDS_SetDriveOutput(double output) {m_driveMotor.Set(output);}
  frc::Vector2d SDS_GetModulePosition() {
    return SDS_modulePosition;
  }
  units::radian_t SDS_GetCurrentAngle() {
    return SDS_currentAngle2;
  }
  double SDS_GetCurrentDistance() {
    return SDS_currentDistance2;
  }
  // units::meters_per_second_t SDS_GetCurrentVelocity() {return SDS_velocity;}
  // units::current::ampere_t SDS_GetDriveCurrent() {return SDS_currentDraw;}
  // frc::SwerveModuleState SDS_GetTargetVelocity() {frc::SwerveModuleState ret; ret.angle = SDS_targetAngle2; ret.speed = SDS_targetSpeed2; return ret;}
  // frc::Vector2d SDS_GetCurrentPosition(); // AFAICT used exactly zero places
  // void SDS_ResetKinematics(); // Also not used. Set SDS_currentPosition to the <0, 0> vector
  void SDS_UpdateSensors2() {
    SDS_currentAngle2 = SDS_ReadAngle();
    SDS_currentDistance2 = SDS_ReadDistance();
    // SDS_currentDraw = SDS_ReadCurrentDraw(); // SDS_ReadCurrentDraw isn't implemented; probably because currentDraw isn't used anywhere
    // SDS_velocity = SDS_ReadVelocity(); // (TODO?: might need to put this back)
  }
  // void SDS_UpdateKinematics(double robotRotation) { [stuff] } // Seems unused
  void SDS_UpdateState() {
    double targetAngle_ = SDS_targetAngle2.to<double>();
    double targetSpeed_ = SDS_targetSpeed2.to<double>();
    // double currentAngle_ = SDS_GetCurrentAngle().to<double>();
    /*
        double delta = targetAngle_ - currentAngle_;
        if (delta >= wpi::math::pi) {
            targetAngle_ -= 2.0 * wpi::math::pi;
        } else if (delta < -wpi::math::pi) {
            targetAngle_ += 2.0 * wpi::math::pi;
        }

        delta = targetAngle_ - currentAngle_;
        if (delta > wpi::math::pi / 2.0 || delta < -wpi::math::pi / 2.0) {
          // Only need to add pi here because the target angle will be put back into the range [0, 2pi)
          targetAngle_ += wpi::math::pi;

          targetSpeed_ *= -1.0;
        }

        targetAngle_ = fmod(targetAngle_, 2.0 * wpi::math::pi);
        if (targetAngle_ < 0.0) {
            targetAngle_ += 2.0 * wpi::math::pi;
        }
    */

    frc::SmartDashboard::PutNumber(m_moduleName.GetFullTitle() + " targetAngle_", targetAngle_);
    frc::SmartDashboard::PutNumber(m_moduleName.GetFullTitle() + " targetSpeed_", targetSpeed_);
    SDS_SetTargetAngle(units::radian_t(targetAngle_));
    m_driveMotor.Set(targetSpeed_);

    /** TODO: updateCallbacks, from the SwerveModuleImpl
     * which is this:
     * `updateCallbacks.add((module, dt) -> motor.set(controller.calculate(module.getCurrentAngle(), dt)))`
     * in one of the angleMotor constructors.
     * Not sure if `SDS_SetTargetAngle` covers that.
     * 
     * Something like this?:
     *  m_turnMotor.Set(m_turnPIDController.Calculate(SDS_GetCurrentAngle()));
     */    
  }
  // units::current::ampere_t SDS_currentDraw = units::current::ampere_t(0);
  // units::meters_per_second_t SDS_velocity = 0_mps;
  // units::current::ampere_t SDS_GetCurrentDraw() {return units::current::ampere_t(m_driveMotor.GetOutputCurrent());}
  double SDS_ReadVelocity() {
    return m_driveEncoder.GetVelocity();
  }
  frc2::PIDController SDS_DEFAULT_ONBOARD_NEO_ANGLE_PIDController{0.5, 0.0, 0.0001};
  frc2::PIDController m_turnPIDController{0.5, 0.0, 0.5};
  frc2::PIDController SDS_DEFAULT_CAN_SPARK_MAX_ANGLE_PIDController{1.5, 0.0, 0.5};
  rev::CANPIDController SDS_angleMotorPIDController = m_turnMotor.GetPIDController();


  // copied from SDS
  const double SDS_DRIVE_REDUCTION = 8.31 / 1.0; // (gear ratio)
  const double SDS_WHEEL_DIAMETER = 4.0; // (in)
  const double SDS_DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * wpi::math::pi)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);
};
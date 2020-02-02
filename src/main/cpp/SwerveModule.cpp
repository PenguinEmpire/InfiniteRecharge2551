/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <wpi/math>
#include "frc/smartdashboard/SmartDashboard.h"

#include <units/units.h>

#include "SwerveModule.h"

using SD = frc::SmartDashboard;

SwerveModule::SwerveModule(frc::Translation2d pos, int analogEncoderPort, units::radian_t analogEncoderOffset, int driveMotorCANID, int turnMotorCANID, SwerveModuleName moduleName)
  : m_driveMotor(driveMotorCANID, rev::CANSparkMax::MotorType::kBrushless), 
    m_turnMotor(turnMotorCANID, rev::CANSparkMax::MotorType::kBrushless),
    m_moduleName{moduleName},
    m_turnEncoder{analogEncoderPort, analogEncoderOffset, m_turnMotor.GetEncoder()},
    m_modulePosition{pos} {

  m_driveEncoder.SetPositionConversionFactor(SDS_WHEEL_DIAMETER * wpi::math::pi / SDS_DRIVE_REDUCTION); // so this is meters, right?
  m_driveEncoder.SetVelocityConversionFactor(SDS_WHEEL_DIAMETER * wpi::math::pi / SDS_DRIVE_REDUCTION * (1.0 / 60.0)); // SDS: "RPM to units per sec"

  // SDS_DEFAULT_CAN_SPARK_MAX_ANGLE_PIDController.EnableContinuousInput(0, 2 * wpi::math::pi); // TODO: necessay/correct? If so, [-pi, pi) instead? // --> think this is correct // nevermind?
  // SDS_DEFAULT_ONBOARD_NEO_ANGLE_PIDController.EnableContinuousInput(0, 2 * wpi::math::pi);
  // m_turnPIDController.EnableContinuousInput(0, 2 * wpi::math::pi);
  SDS_DEFAULT_CAN_SPARK_MAX_ANGLE_PIDController.EnableContinuousInput(-wpi::math::pi, wpi::math::pi); // TODO: necessay/correct? If so, [-pi, pi) instead? // --> think this is correct // nevermind?
  SDS_DEFAULT_ONBOARD_NEO_ANGLE_PIDController.EnableContinuousInput(-wpi::math::pi, wpi::math::pi);
  m_turnPIDController.EnableContinuousInput(-wpi::math::pi, wpi::math::pi);
  // m_turnPIDController.SetTolerance(0.5);

  m_turnEncoder.builtInMotorEncoder.SetPosition(m_turnEncoder.GetAngle_SDS().to<double>());

  SDS_angleMotorPIDController.SetP(1.5);
  SDS_angleMotorPIDController.SetI(0);
  SDS_angleMotorPIDController.SetD(0.5);
}

void SwerveModule::PutDiagnostics() {
  using SD = frc::SmartDashboard;

  SD::PutNumber(m_moduleName.GetFullTitle() + " SDS_GetCurrentAngle", SDS_GetCurrentAngle().to<double>());
  SD::PutNumber(m_moduleName.GetFullTitle() + " angleMotorEncoder", m_turnEncoder.builtInMotorEncoder.GetPosition());
  // SD::PutNumber(m_moduleName.GetFullTitle() + " driveMotorEncoderPosition", m_driveEncoder.GetPosition());
  // SD::PutNumber(m_moduleName.GetFullTitle() + " driveMotorEncoderVelocity", m_driveEncoder.GetVelocity());



}

// [almost certainly depr] void SwerveModule::UpdateState() {m_turnPIDController.SetSetpoint(adjustedTargetAngle); m_turnMotor.Set(m_turnPIDController.Calculate(m_turnEncoder.GetAngle_SDS().to<double>()));}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
  // units::meters_per_second_t speed = state.speed;
  // frc::Rotation2d angle = state.angle;
  /*
    const frc::Rotation2d rot_pi = frc::Rotation2d(units::radian_t(wpi::math::pi));

    if (speed < 0_mps) {
      speed *= -1;
      angle.RotateBy(rot_pi);
    }
    
    // TODO : I think all of this is unnecessary?
      /** SDS has:
       *   angle %= 2.0 * Math.PI;
           if (angle < 0.0) {
               angle += 2.0 * Math.PI;
          }
      *
      * Assuming/hoping that frc::Rotation2d::Degrees() automatically returns the terminal (?) angle (within `[0, 2pi)`).
      * Update: pretty sure it doesn't.
      * /

      double temp_angle = angle.Radians().to<double>();
      temp_angle = fmod(temp_angle, 2 * wpi::math::pi);
      angle = frc::Rotation2d(units::degree_t(temp_angle));
      if (angle.Radians() < 0_rad) {
        angle.RotateBy(rot_pi.operator*(2));
      }
  */
  // SDS_targetSpeed2 = speed;           // in SDS, under `synchronized (stateMutex)`
  // SDS_targetAngle2 = angle.Radians(); // ditto

  double speed_ = state.speed.to<double>();
  double angle_ = state.angle.Radians().to<double>();

  // frc::Rotation2d desiredAngle = state.angle;
  // desiredAngle += frc::Rotation2d(units::radian_t(0));
  // double angle_ = desiredAngle.Radians().to<double>();

  // if (angle_ < 0) {
  //   angle_ += 2 * wpi::math::pi;
  // }

  /*
    // copied from `UpdateState`
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

  frc::SmartDashboard::PutNumber(m_moduleName.GetFullTitle() + "Desired State : speed", speed_);
  frc::SmartDashboard::PutNumber(m_moduleName.GetFullTitle() + "Desired State : angle", angle_);
  

  m_driveMotor.Set(speed_);
  m_turnMotor.Set(m_turnPIDController.Calculate(m_turnEncoder.GetAngle_SDS().to<double>(), angle_)); // used to be `SDS_SetTargetAngle(units::radian_t(targetAngle_));`
}

void SwerveModule::SDS_UpdateSensors() {
  /*
    SDS_currentAngle = m_turnEncoder.GetAngle_SDS().to<double>();
      // use in steeringMotor.set(angleController.calculate(getCurrentAngle(), dt)); -- dt is .02 seconds
      // 


    // SDS_currentDistance = ReadDistance();
      // driveEncoder.GetPosition() (with the position conversion factor set to (wheelDiameter * Math.PI / reduction))
      // driveEncoder.GetPosition() * (1.0 / SDS_DEFAULT_DRIVE_ROTATIONS_PER_UNIT)
      // pretty sure these are not the same, and I don't know which one wins
      // currentDistance is used in `updateKinematics` to update `currentPosition`, but afaict `currentPosition` isn't used anywhere???

    // SDS_currentDraw = ReadCurrentDraw();
      // driveMotor.getOutputCurrent()
      // I think also not used anywhere??
    // SDS_velocity = ReadVelocity();
      // driveEncoder.GetVelocity() (with the velocity conversion factor set to (wheelDiameter * Math.PI / reduction * (1.0 / 60.0)) )
      // I think also not used anywhere?
  */
  SDS_currentAngle2 = SDS_ReadAngle();
  SDS_currentDistance2 = SDS_ReadDistance();
  // SDS_currentDraw = SDS_ReadCurrentDraw() // guess it doesn't exist because it wasn't used?
  // SDS_velocity = SDS_ReadVelocity() // also can't find it being used
}

void SwerveModule::SDS_SetTargetAngle(units::radian_t angle) {
  double targetAngle = angle.to<double>();
/*
  double currentAngle = m_turnEncoder.builtInMotorEncoder.GetPosition();
  double currentAngleMod = fmod(currentAngle, 2 * wpi::math::pi);
  if (currentAngleMod < 0) {
    currentAngleMod += 2 * wpi::math::pi;
  }
  double newTarget = targetAngle + currentAngle - currentAngleMod;
  if (targetAngle - currentAngleMod > wpi::math::pi) {
    newTarget -= 2.0 * wpi::math::pi;
  } else if (targetAngle - currentAngleMod < wpi::math::pi) {
    newTarget += 2.0 * wpi::math::pi;
  }
*/
  // SDS_angleMotorPIDController.SetReference(newTarget, rev::ControlType::kPosition);
  // m_turnMotor.Set(m_turnPIDController.Calculate(m_turnEncoder.GetAngle_SDS().to<double>(), targetAngle.to<double>()));
}
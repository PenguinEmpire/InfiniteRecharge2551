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

SwerveModule::SwerveModule(frc::Translation2d pos, int analogEncoderPort, units::radian_t analogEncoderOffset, int driveMotorCANID, int turnMotorCANID, SwerveModuleName moduleName)
  : m_driveMotor(driveMotorCANID, rev::CANSparkMax::MotorType::kBrushless), 
    m_turnMotor(turnMotorCANID, rev::CANSparkMax::MotorType::kBrushless),
    m_moduleName{moduleName},
    m_turnEncoder{analogEncoderPort, analogEncoderOffset, m_turnMotor.GetEncoder()},
    m_modulePosition{pos} {

  // TODO: Figure out why this broke it
  // m_turnMotor.RestoreFactoryDefaults();
  // m_driveMotor.RestoreFactoryDefaults();
   
  // TODO: current limits.
  // m_turnMotor.SetSmartCurrentLimit();
  // m_driveMotor.SetSmartCurrentLimit();
  // m_turnMotor.SetSecondaryCurrentLimit();
  // m_driveMotor.SetSecondaryCurrentLimit();

  m_driveEncoder.SetPositionConversionFactor(SDS_WHEEL_DIAMETER * PenguinUtil::PI / SDS_DRIVE_REDUCTION); // so this is meters, right?
  m_driveEncoder.SetVelocityConversionFactor(SDS_WHEEL_DIAMETER * PenguinUtil::PI / SDS_DRIVE_REDUCTION * (1.0 / 60.0)); // SDS: "RPM to units per sec"

  m_onboardTurnMotorPIDController.SetP(1.5);
  m_onboardTurnMotorPIDController.SetI(0);
  m_onboardTurnMotorPIDController.SetD(0.5);

  ReadSensors();

  // builtInMotorEncoder.SetPosition(m_currentAngle.to<double>()); // moved to the constuctor for TurnEncoder
}

void SwerveModule::PutDiagnostics() {
  using SD = frc::SmartDashboard;

  SD::PutNumber(m_moduleName.GetAbbrUpper() + " angle (analog)", units::radian_t(m_currentAngle).to<double>());
  SD::PutNumber(m_moduleName.GetAbbrUpper() + " angleMotorEncoder", m_turnEncoder.builtInMotorEncoder.GetPosition());
  // SD::PutNumber(m_moduleName.GetAbbrUpper() + " driveMotorEncoderPosition", m_driveEncoder.GetPosition());
  // SD::PutNumber(m_moduleName.GetAbbrUpper() + " driveMotorEncoderVelocity", m_driveEncoder.GetVelocity());

  SD::PutString(m_moduleName.GetAbbrUpper() + " z-----sep", "");

}

/** This is a function to do the calculations to solve the "rotate 180 or drive backwards?" problem.
 * It's badly named; we're not normalizing anything.
 * I *think* it modifies the `SwerveModuleState` in-place (using pass-by reference).
 * @param state The `frc::SwerveModuleState` to modify to make things more efficient.
 * @return Nothing: this function modifies `state` in-place (hopefully).
 */
void SwerveModule::Solve180Problem1(frc::SwerveModuleState& state) {
  units::meters_per_second_t targetSpeed = state.speed;
  frc::Rotation2d angle = state.angle;

  double angle_ = angle.Radians().to<double>();

  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " pre-norm angle", angle_);
  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " pre-norm speed", targetSpeed.to<double>());


  // // From SDS' `targetAngleConsumer`
  // double currentAngle = m_turnEncoder.builtInMotorEncoder.GetPosition();
  // double currentAngleMod = fmod(currentAngle, 2 * wpi::math::pi);
  // if (currentAngleMod < 0) {
  //   currentAngleMod += 2 * wpi::math::pi;
  // }
  // double newTarget = angle_ + currentAngle - currentAngleMod;
  // if (angle_ - currentAngleMod > wpi::math::pi) {
  //   newTarget -= 2 * wpi::math::pi;
  // } else if (angle_ - currentAngleMod < -wpi::math::pi) {
  //   newTarget += 2 * wpi::math::pi;
  // }
  // angle = frc::Rotation2d(units::radian_t(newTarget));


  // frc::Rotation2d currentAngleRot = frc::Rotation2d(m_currentAngle);
  frc::Rotation2d currentAngleRot = frc::Rotation2d(units::radian_t(m_turnEncoder.builtInMotorEncoder.GetPosition()));
  // units::degree_t angleDif = (currentAngleRot - angle).Degrees();
  if (units::math::abs((currentAngleRot - angle).Degrees()) > 90_deg) {
    frc::SmartDashboard::PutBoolean("adjusting angle", true);
    angle.RotateBy(frc::Rotation2d(units::radian_t(wpi::math::pi)));
    targetSpeed *= -1;
  } else {
    frc::SmartDashboard::PutBoolean("adjusting angle", false);
  }

  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " post-norm (in norm) : angle", angle.Radians().to<double>());
  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " post-norm (in norm) : speed", targetSpeed.to<double>());


  state.speed = targetSpeed;
  state.angle = angle;
}

void SwerveModule::Solve180Problem2(frc::SwerveModuleState& state) {
  frc::SwerveModuleState tempState;

  units::meters_per_second_t targetSpeed = state.speed;
  frc::Rotation2d targetAngle = state.angle;

  tempState.speed = targetSpeed;
  tempState.angle = targetAngle;
  PutSwerveModuleState("0th norm", tempState);

  // 1st norm: Section in SDS' `SwerveModule.java::setTargetVelocity`

  if (targetSpeed < 0_mps) {
    targetSpeed *= -1;
    targetAngle += frc::Rotation2d(180_deg);
    // targetAngle = targetAngle + frc::Rotation2d(180_deg);
  }
  // if (targetAngle.Degrees() < 0_deg) {
  //   targetAngle.RotateBy()
  // }

  // Then SDS has:
    // angle %= 2.0 * Math.PI;
    // if (angle < 0.0) {
    //     angle += 2.0 * Math.PI;
    // }
  // But I think `frc::Rotation2d::operator+=()` does that automatically

  tempState.speed = targetSpeed;
  tempState.angle = targetAngle;
  PutSwerveModuleState("1st norm", tempState);


  // 2nd norm: Section in SDS' `SwerveModule.java::updateState`

  units::radian_t targetAngle2_r = targetAngle.Radians();

  units::radian_t currentAngle = m_currentAngle;

  units::radian_t delta = targetAngle2_r - currentAngle;
  if (delta >= 180_deg) {
    targetAngle2_r -= 360_deg;
  } else if (delta <= -180_deg) {
    targetAngle2_r += 360_deg;
  }

  units::radian_t delta2 = targetAngle2_r - currentAngle;
  if (delta2 > 180_deg / 2 || delta2 < -180_deg / 2) {
    targetAngle2_r += 180_deg;
    targetSpeed *= -1;
  }

  frc::Rotation2d targetAngle2 = frc::Rotation2d(targetAngle2_r);
  targetAngle2 += frc::Rotation2d(); // put in range [0, 2pi) (I think)

  tempState.speed = targetSpeed;
  tempState.angle = targetAngle2;
  PutSwerveModuleState("2nd norm", tempState);

  
  // 3rd norm: Section in SDS' `Mk2SwerveModuleBuilder.java::angleMotor::targetAngleconsumer`

  units::radian_t targetAngle3_r = targetAngle2.Radians();

  units::radian_t currentMotorAngle_r = units::radian_t(m_turnEncoder.builtInMotorEncoder.GetPosition());
  units::radian_t currentMotorAngleMod_r = (frc::Rotation2d(currentMotorAngle_r) + frc::Rotation2d()).Radians();
  if (currentMotorAngleMod_r < 0_rad) {
    currentMotorAngleMod_r += 360_deg;
  }

  targetAngle3_r = targetAngle3_r + currentMotorAngle_r - currentMotorAngleMod_r;
  if (targetAngle3_r - currentMotorAngleMod_r > 180_deg) {
    targetAngle3_r -= 360_deg;
  } else if (targetAngle3_r - currentMotorAngleMod_r < -180_deg) {
    targetAngle3_r += 360_deg;
  }
  frc::Rotation2d targetAngle3 = frc::Rotation2d(targetAngle3_r);

  tempState.speed = targetSpeed;
  tempState.angle = targetAngle3;
  PutSwerveModuleState("3rd norm", tempState);  

  // 4th norm: fixing jitter
  
  units::radian_t targetAngle4_r = targetAngle3_r;

  if (units::math::abs(targetAngle4_r - PenguinUtil::TWO_PI_RAD) < 0.1_deg) {
    targetAngle4_r = 0_rad;
  }
  frc::Rotation2d targetAngle4 = frc::Rotation2d(targetAngle4_r);

  tempState.speed = targetSpeed;
  tempState.angle = targetAngle4;
  PutSwerveModuleState("4th norm", tempState);

  state.speed = targetSpeed;
  state.angle = targetAngle4;
}

/** Deprecated, I guess? Sets the module to a constant speed and direction. Just testing that the pass-by-reference thing works. It does!
 */
void SwerveModule::ToConstantState3(frc::SwerveModuleState& state) {
  units::meters_per_second_t targetSpeed = state.speed;
  frc::Rotation2d angle = state.angle;

  targetSpeed = 0.1_mps;
  angle = frc::Rotation2d(60_deg);

  state.speed = targetSpeed;
  state.angle = angle;
}
  
void SwerveModule::Solve180Problem4(frc::SwerveModuleState& state) {
  PutSwerveModuleState("(4.1) pre-norm", state);

  units::meters_per_second_t targetSpeed = state.speed;
  frc::Rotation2d targetAngle = state.angle;

  // units::radian_t currentAngle = units::radian_t(m_turnEncoder.builtInMotorEncoder.GetPosition());
  // units::radian_t currentAngle = m_currentAngle;
  units::radian_t currentAngle = m_turnEncoder.GetAngle_SDS(false);
  units::radian_t targetAngle_r = targetAngle.Radians();

  units::radian_t delta = currentAngle - targetAngle_r;
  delta = units::math::abs(PenguinUtil::piNegPiClamp(units::math::abs(delta)));
  if (delta > 90_deg) {
    targetSpeed *= -1;
    targetAngle += PenguinUtil::PI_ROT;
  }

  while (targetAngle.Radians() > PenguinUtil::PI_RAD) {
    targetAngle -= PenguinUtil::TWO_PI_ROT;
  }
  while (targetAngle.Radians() < -PenguinUtil::PI_RAD) {
    targetAngle += PenguinUtil::TWO_PI_ROT;
  }
  
  state.speed = targetSpeed;
  state.angle = targetAngle;

  PutSwerveModuleState("(4.2) post-norm", state);
}

void SwerveModule::Solve180Problem5_RestrictToHalfPi(frc::SwerveModuleState& state) {
  PutSwerveModuleState("(5.1) pre-norm", state);

  units::meters_per_second_t targetSpeed = state.speed;
  frc::Rotation2d targetAngle = state.angle;

  units::radian_t targetAngle_r = targetAngle.Radians();
  if (targetAngle_r > PenguinUtil::PI_RAD / 2) {
    targetAngle -= PenguinUtil::PI_ROT;
    targetSpeed *= -1;
  } else if (targetAngle_r < -PenguinUtil::PI_RAD / 2) {
    targetAngle += PenguinUtil::PI_ROT;
    targetSpeed *= -1;
  }

  state.speed = targetSpeed;
  state.angle = targetAngle;

  PutSwerveModuleState("(5.2) post-norm", state);
}

void SwerveModule::ToConstantBackState6(frc::SwerveModuleState& state, bool left_or_right) {
  PutSwerveModuleState("(6.1) pre-norm", state);

  if (left_or_right) {
    state.angle = -PenguinUtil::PI_ROT + frc::Rotation2d(20_deg);
  } else {
    state.angle = -PenguinUtil::PI_ROT - frc::Rotation2d(20_deg);
  }

  state.speed = 0.1_mps;

  PutSwerveModuleState("(6.2) post-norm", state);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& state, bool left_or_right) {
  Solve180Problem4(state);

  const double speed = state.speed.to<double>();
  const double angle = state.angle.Radians().to<double>();

  PutSwerveModuleState("to-motor", angle, speed);

  m_driveMotor.Set(speed);
  m_onboardTurnMotorPIDController.SetReference(angle, rev::ControlType::kPosition);
}

void SwerveModule::SetDirectly(double angle, double speed) {
  m_driveMotor.Set(speed);
  m_onboardTurnMotorPIDController.SetReference(angle, rev::ControlType::kPosition);
}

void SwerveModule::ReadSensors() {
  /* Original discussion when I was porting of the four different variable writes in this function, and where each of them are coming from/going to

    SDS_currentAngle = m_currentAngle.to<double>();
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

  m_currentAngle = m_turnEncoder.GetAngle_SDS(); // m_currentAngle is only set here, and always means this
  // SDS_currentDistance2 = SDS_ReadDistance(); // `SDS_ReadDistance` was `m_driveEncoder.GetPosition()`
}

void SwerveModule::PutSwerveModuleState(std::string info, frc::SwerveModuleState& state) {
  double speed_ = state.speed.to<double>();
  double angle_ = state.angle.Radians().to<double>();

  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " " + info + " speed", speed_);
  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " " + info + " angle", angle_);
}

void SwerveModule::PutSwerveModuleState(std::string info, double angle, double speed) {
  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " true " + info + " speed", speed);
  frc::SmartDashboard::PutNumber(m_moduleName.GetAbbrUpper() + " true " + info + " angle", angle);
}


void SwerveModule::UpdateAnalogOffset() {
  m_turnEncoder.UpdateOffset();
}
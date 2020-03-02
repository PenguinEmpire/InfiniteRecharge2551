#pragma once

#include <string>
#include <math.h>

#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Limelight.h"

#include "AHRS.h"
#include "ctre/Phoenix.h"

#include "frc/Joystick.h"
#include "frc/SPI.h"
#include "frc2/Timer.h"
#include "frc/Spark.h"

// #include "frc/trajectory/TrajectoryGenerator.h"
// #include "frc/trajectory/Trajectory.h"
// #include "frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h"
// #include "frc/trajectory/TrajectoryConfig.h"

#include "LimelightAutonomous.h"
#include "PenguinUtil.h"
#include "SwerveDrive.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string trajectoryAutoName = "trajectoryAutoTest";
  const std::string limelightAutoName = "limelightAutoTest";
  std::string m_autoSelected;


  Limelight limelight;

  frc2::Timer m_timer;
  units::second_t m_currentTime = units::second_t(0);

  frc::Joystick m_leftJoystick{0};
  frc::Joystick m_rightJoystick{1};
  frc::Joystick m_gamerJoystick{2};
  
  SwerveDrive m_drivetrain;

  void ProcessJoysticks();
  void Drive();
  void ConfigESCs();

  WPI_TalonSRX elevator{PenguinConstants::CAN::ELEVATOR_MASTER};
  WPI_VictorSPX elevatorHelper{PenguinConstants::CAN::ELEVATOR_SLAVE};
  WPI_TalonSRX intake{PenguinConstants::CAN::INTAKE};
  WPI_TalonSRX belt{PenguinConstants::CAN::BELT};
  WPI_TalonSRX aimer{PenguinConstants::CAN::AIMER};
  WPI_TalonSRX shooter{PenguinConstants::CAN::SHOOTER};
  WPI_TalonSRX centerer{PenguinConstants::CAN::CENTERER};
  
  std::unique_ptr<WPI_TalonSRX> elevator_ = std::make_unique<WPI_TalonSRX>(PenguinConstants::CAN::ELEVATOR_MASTER);
  std::unique_ptr<WPI_VictorSPX> elevatorHelper_ = std::make_unique<WPI_VictorSPX>(PenguinConstants::CAN::ELEVATOR_SLAVE);
  std::unique_ptr<WPI_TalonSRX> intake_ = std::make_unique<WPI_TalonSRX>(PenguinConstants::CAN::INTAKE);
  std::unique_ptr<WPI_TalonSRX> belt_ = std::make_unique<WPI_TalonSRX>(PenguinConstants::CAN::BELT);
  std::unique_ptr<WPI_TalonSRX> aimer_ = std::make_unique<WPI_TalonSRX>(PenguinConstants::CAN::AIMER);
  std::unique_ptr<WPI_TalonSRX> shooter_ = std::make_unique<WPI_TalonSRX>(PenguinConstants::CAN::SHOOTER);
  std::unique_ptr<WPI_TalonSRX> centerer_ = std::make_unique<WPI_TalonSRX>(PenguinConstants::CAN::CENTERER);
  
  std::shared_ptr<WPI_TalonSRX> elevator_s = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::ELEVATOR_MASTER);
  std::shared_ptr<WPI_VictorSPX> elevatorHelper_s = std::make_shared<WPI_VictorSPX>(PenguinConstants::CAN::ELEVATOR_SLAVE);
  std::shared_ptr<WPI_TalonSRX> intake_s = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::INTAKE);
  std::shared_ptr<WPI_TalonSRX> belt_s = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::BELT);
  std::shared_ptr<WPI_TalonSRX> aimer_s = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::AIMER);
  std::shared_ptr<WPI_TalonSRX> shooter_s = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::SHOOTER);
  std::shared_ptr<WPI_TalonSRX> centerer_s = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::CENTERER);

  LimelightAutonomous limelightAuto{&limelight};

 private:

  // frc::TrajectoryConfig trajectoryConfig{m_drivetrain.K_MAX_VELOCITY, m_drivetrain.K_MAX_ACCELERATION};
  // frc::Trajectory exampleTrajectory{std::vector<frc::Trajectory::State>()};
};

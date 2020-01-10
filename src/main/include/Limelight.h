/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "PenguinConstants.h"

/** 
 * Limelight documentation is available [here](http://docs.limelightvision.io/en/latest/index.html).
 */

class Limelight {
 public:
  Limelight();
  void SetVisionCamMode();
  void SetDriveCamMode();
  LimelightValues GetInfo();

 private:
  std::shared_ptr<NetworkTable> table;
};

struct LimelightValues {
  // LimelightValues();
  LimelightValues(double _tx, double _ty) :
    tx{_tx}, ty{_ty} {}
  double tx;
  double ty;
};
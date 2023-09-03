// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
  DifferentialDriveOdometry m_odometry;
  DriveTrain m_driveTrain;
  /** Creates a new Odometry. */
  public Odometry(DriveTrain drive) {
    m_driveTrain = drive; 
    m_odometry = new DifferentialDriveOdometry(null
    , m_driveTrain.getLeftMotorsEncoder()
    , m_driveTrain.getRightMotorsEncoder());
  }

  @Override
  public void periodic() {
    m_odometry.update(null
    , m_driveTrain.getLeftMotorsEncoder()
    , m_driveTrain.getRightMotorsEncoder());

    // This method will be called once per scheduler run
  }
}

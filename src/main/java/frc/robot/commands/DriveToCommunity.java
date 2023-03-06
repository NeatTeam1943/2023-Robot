// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToCommunity extends CommandBase {
  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  private boolean m_passChargeStation;
  private boolean m_hasSafeDistance;
  private double m_setpoint;

  /** Creates a new DriveToCommunity. */
  public DriveToCommunity(DriveTrain drive) {
    m_drive = drive;
    m_imu = drive.getIMU();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.calibrateIMU();

    m_passChargeStation = false;
    m_hasSafeDistance = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double m_voltage = 0.15;
    final double m_threshold = 1; // Just should be higher than zero
    final double m_angleX = m_imu.getGyroAngleX(); // Create a cleaner boolean value to check for stabilizion 

    boolean m_isStable = (-1 < m_angleX && m_angleX < 1); 

    m_drive.arcadeDrive(-m_voltage, 0, false);

    if (m_angleX > m_threshold) { // Should not check for something that can't happen
      m_passChargeStation = true;
    }

    if (m_passChargeStation && m_isStable && !m_hasSafeDistance) { // Fix naming
      m_hasSafeDistance = true;
      m_setpoint = m_drive.getDistance() + 0.25; // Fix formatting
    }

    System.out.println("Driving to community, AngleX: " + m_angleX + ", Did pass: " + m_passChargeStation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_drive.getDistance() > m_setpoint;
    return m_hasSafeDistance; 
  }
}

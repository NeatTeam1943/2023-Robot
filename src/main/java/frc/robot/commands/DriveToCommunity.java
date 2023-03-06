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

  private boolean m_didPassChargeStation;
  private boolean m_startMoving;
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
    m_didPassChargeStation = false;
    m_startMoving = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double voltage = 0.15;
    final double threshold = 3;
    final double angleX = m_imu.getGyroAngleX();

    m_drive.arcadeDrive(-voltage, 0, false);

    if (!m_didPassChargeStation && angleX > threshold) {
      m_didPassChargeStation = true;
    }

    if (m_didPassChargeStation && (-1 < angleX && angleX < 1) && !m_startMoving){
      m_startMoving = true;
      m_setpoint = m_drive.getDistance()+0.98;
    }

    System.out.println("Driving to community, AngleX: " + angleX + ", Did pass: " + m_didPassChargeStation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getDistance() > m_setpoint;
  }
}

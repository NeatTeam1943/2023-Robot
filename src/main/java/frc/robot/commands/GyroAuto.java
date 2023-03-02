// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GyroAuto extends CommandBase {

  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  /** Creates a new GyroAuto. */
  public GyroAuto(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveTrain;
    m_imu = m_drive.getIMU();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: Make sure this works with other commands
    m_imu.calibrate();
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(-0.15, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getDistance() > 1.5 && (-1 < m_imu.getGyroAngleX() && m_imu.getGyroAngleX() < 1);
  }
}

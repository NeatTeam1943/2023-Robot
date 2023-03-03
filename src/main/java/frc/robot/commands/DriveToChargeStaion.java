// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToChargeStaion extends CommandBase {

  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  /** Creates a new GyroAuto. */
  public DriveToChargeStaion(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveTrain;
    m_imu = m_drive.getIMU();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_imu.calibrate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(-0.15, 0, false);
    System.out.println("Driving, angle: " + m_imu.getGyroAngleX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_imu.getGyroAngleX() < -10 || m_imu.getGyroAngleX() > 10;
  }
}

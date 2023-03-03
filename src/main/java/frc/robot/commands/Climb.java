// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Climb extends CommandBase {

  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  private double m_setpoint;

  /** Creates a new GyroAuto. */
  public Climb(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveTrain;
    m_imu = m_drive.getIMU();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setpoint = m_drive.getDistance() + 0.93;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double voltage = 0.13;

    m_drive.arcadeDrive(-voltage, 0, false);

    System.out.println("Climbing, angle: " + m_imu.getGyroAngleX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double angleX = m_imu.getGyroAngleX();
    final double threshold = 1.1;

    // return angleX > -threshold && angleX < threshold;

    return m_drive.getDistance() > m_setpoint; 
  }
}

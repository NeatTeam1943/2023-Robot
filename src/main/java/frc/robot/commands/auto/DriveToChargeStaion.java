// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToChargeStaion extends CommandBase {

  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  private boolean m_inverted;

  /** Creates a new GyroAuto. */
  public DriveToChargeStaion(DriveTrain driveTrain, boolean inverted) {
    m_drive = driveTrain;
    m_imu = m_drive.getIMU();

    m_inverted = inverted;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== Start DriveToChargeStation( " + m_inverted + " ) ==========");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double voltage = 0.15;

    if (m_inverted) {
      m_drive.arcadeDrive(voltage, 0, false);
    } else {
      m_drive.arcadeDrive(-voltage, 0, false);
    }

    System.out.println("Current angleY: " + m_imu.getGyroAngleY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
    System.out.println("========== Finished DriveToChargeStation() ==========");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_imu.getGyroAngleY() < -10 || m_imu.getGyroAngleY() > 10;
  }
}
